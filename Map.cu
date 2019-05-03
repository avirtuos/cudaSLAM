
#include "Map.h"

Map::Map(int width_arg, int height_arg, int scan_buffer_size)
{
    map_init = false;
    width = width_arg;
    height = height_arg;

    //Allocate scan buffer on the device as well as the size of scan
    const unsigned int scan_bytes = scan_buffer_size * sizeof(TelemetryPoint);
    checkCuda(cudaMalloc((void **)&scan_buffer_d, scan_bytes));
    checkCuda(cudaMalloc((void **)&scan_size_d, sizeof(int)));

    //Allocate sim buffer on device
    int sim_size_h = 360 * scan_buffer_size;
    const unsigned int sim_bytes = sim_size_h * sizeof(SimTelemetryPoint);
    checkCuda(cudaMalloc((void **)&sim_buffer_d, sim_bytes));
    checkCuda(cudaMalloc((void **)&sim_size_d, sizeof(int)));
    cudaMemcpy(sim_size_d, &sim_size_h, sizeof(int), cudaMemcpyHostToDevice);

    //Allocate pinned memory on the host and device for the current map
    map_bytes = width * height * sizeof(MapPoint);
    checkCuda(cudaMallocHost((void **)&map_h, map_bytes));
    checkCuda(cudaMalloc((void **)&map_d, map_bytes));
    checkCuda(cudaMalloc((void **)&width_d, sizeof(int)));
    checkCuda(cudaMalloc((void **)&height_d, sizeof(int)));

    //copy the size of the map to the device
    //todo: in the future this should probably be auto-expandable based on the size of the mapped area
    //but evne using a static size is fine for area 30m x 30m which is more than enough for most hobby applications
    cudaMemcpy(width_d, &width, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(height_d, &height, sizeof(int), cudaMemcpyHostToDevice);

    map_update_dim = 512;
    const unsigned int n = map_update_dim * map_update_dim;
    checkCuda(cudaMalloc((void **)&result_d, n * sizeof(LocalizedOrigin)));

    localized_size = 10000;
    checkCuda(cudaMallocHost((void **)&localized_result_h, localized_size * sizeof(LocalizedOrigin)));
    checkCuda(cudaMalloc((void **)&localized_result_d, localized_size * sizeof(LocalizedOrigin)));
}

Map::~Map()
{
    cudaFreeHost(map_h);
    cudaFree(map_d);
    cudaFree(width_d);
    cudaFree(height_d);
    cudaFree(scan_buffer_d);
    cudaFree(scan_size_d);
    cudaFree(sim_buffer_d);
    cudaFree(sim_size_d);
    cudaFree(result_d);
    cudaFreeHost(localized_result_h);
    cudaFree(localized_result_d);
}


__global__
void cudeGenerateParticleFilter(SimTelemetryPoint *sim_buffer, int *sim_size, TelemetryPoint *scan_buffer, int *scan_size)
{
    extern __shared__ TelemetryPoint scan_buffer_s[];

    for(int16_t i = threadIdx.x; i < *scan_size; i += blockDim.x)
    {
        scan_buffer_s[i] = scan_buffer[i];
    }
    __syncthreads();

    int sim_num = blockIdx.x * blockDim.x + threadIdx.x;

    if(sim_num > *sim_size)
    {
        return;
    }

    int increment = gridDim.x * blockDim.x;
    for(int i = sim_num; i < 360 *  *scan_size; i += increment)
    {
        int scan_num = i % *scan_size;
        float distance = scan_buffer_s[scan_num].distance;
        float angle_num = scan_buffer_s[scan_num].angle + floorf(i / *scan_size);

        sim_buffer[i].x = roundf(__sinf (angle_num) * distance);
        sim_buffer[i].y = roundf(__cosf (angle_num) * distance);
    }
}



//TODO: this needs parrallism, like a map/reduce paradim. There were examples of this in the book where you use nested loops and sync threads.
__global__
void cudaLocalizeParticleFilter_slow(LocalizedOrigin *result, int result_size)
{
    LocalizedOrigin best;
    best.score = -1;
    for(int i = 0; i < result_size; i++)
    {
        if(result[i].score > best.score)
        {
            best.x_offset = result[i].x_offset;
            best.y_offset = result[i].y_offset;
            best.angle_offset = result[i].angle_offset;
            best.score = result[i].score;
        }
    }

    printf("BEST: x: %d  y: %d  a: %.2f  s: %d \n", best.x_offset, best.y_offset, best.angle_offset, best.score);
}

//TODO: this needs parrallism, like a map/reduce paradim. There were examples of this in the book where you use nested loops and sync threads.
__global__
void cudaLocalizeParticleFilter(LocalizedOrigin *output, int max_output_size, LocalizedOrigin *input, int input_size)
{
    extern __shared__ LocalizedOrigin localization[];

    if(blockDim.x >= max_output_size ||  gridDim.x >= max_output_size ){
        //kernel config exceeds buffer sizes
        if(blockIdx.x * blockDim.x + threadIdx.x == 0) {
            printf("Exiting due to insufficient buffer size");
        }
        return;
    }

    int tid = threadIdx.x;
    int offset = blockIdx.x * blockDim.x + threadIdx.x;

    LocalizedOrigin best;
    best.score = -1;

    //do the first round.
    int increment = gridDim.x * blockDim.x;
    for(int i = offset; i < input_size; i += increment)
    {
        if(input[i].score > best.score)
        {
            best = input[i];
        }
    }

    localization[tid] = best;
    __syncthreads();

    for (unsigned int s=blockDim.x/2; s>0; s>>=1) {
        if (tid < s && localization[tid].score < localization[tid+s].score) {
            localization[tid] = localization[tid+s];
        }
        __syncthreads();
    }

    if (tid == 0) output[blockIdx.x] = localization[0];
}


//TODO: There is opportunity to speed this up using hints from odometry or even just simple distance traveled estimates.
__global__
void cudaRunParticleFilter(int search_distance, LocalizedOrigin *result, SimTelemetryPoint *sim_buffer, int *sim_size, TelemetryPoint *scan_buffer, int *scan_size, MapPoint *map, int *map_width, int *map_height)
{
    extern __shared__ SimTelemetryPoint sim_buffer_s[];
    int offset = blockIdx.x * blockDim.x + threadIdx.x;

    uint16_t e_search_distance = search_distance / 2;
    int16_t x_offset = (-1 * e_search_distance) + offset % (e_search_distance * 2);
    int16_t y_offset = (-1 * e_search_distance) + floorf(offset / (e_search_distance * 2));

    //int max_pos = *map_width * *map_height;

    int e_width = (*map_width / 2);
    int e_height = (*map_height / 2);

    int ne_width = -1*e_width;
    int ne_height = -1*e_height;

    LocalizedOrigin best;
    best.score = 0;

    int l_scansize = *scan_size;

    //Try various angles - TODO: find better sampling technique here possibly even re-sampling
    for(uint16_t angle_offset = 0; angle_offset < 360; angle_offset+=2)
    {
        //For each point see if we have a hit
        uint16_t score = 0;
        uint16_t score2 = 0;
        int scan_point_offset = angle_offset * l_scansize;

        for(int i = threadIdx.x; i < l_scansize; i += blockDim.x)
        {
            sim_buffer_s[i] = sim_buffer[scan_point_offset + i];
        }

        int scan_point_offset2 = angle_offset+1 * l_scansize;
        for(int i = l_scansize + threadIdx.x; i < 2*l_scansize; i += blockDim.x)
        {
            sim_buffer_s[i] = sim_buffer[scan_point_offset2 + i];
        }

        __syncthreads();

        if(x_offset < e_search_distance && y_offset < e_search_distance)
        {
            //#pragma unroll
            for(uint16_t scan_point = 0; scan_point < l_scansize; scan_point++)
            {
                SimTelemetryPoint sim_point = sim_buffer_s[scan_point];
                SimTelemetryPoint sim_point2 = sim_buffer_s[scan_point+l_scansize];

                int16_t x = x_offset + sim_point.x;
                int16_t y = y_offset + sim_point.y;
                int16_t x2 = x_offset + sim_point2.x;
                int16_t y2 = y_offset + sim_point2.y;

                if(!(x >= e_width || x <= ne_width || y >= e_height || y <= ne_height))
                {
                    int l_height = (e_height + y);
                    int l_width = (e_width + x);
                    int l2_height = l_height * *map_width;
                    int pos =  l2_height + l_width;

                    //MapPoint *map_point = map + pos;
                    score += map[pos].occupancy/3;
                }

                if(!(x2 >= e_width || x2 <= ne_width || y2 >= e_height || y2 <= ne_height))
                {
                    int l_height = (e_height + y2);
                    int l_width = (e_width + x2);
                    int l2_height = l_height * *map_width;
                    int pos =  l2_height + l_width;

                    //MapPoint *map_point = map + pos;
                    score2 += map[pos].occupancy/3;
                }
            }
        }

        if(score > score2 && best.score < score)
        {
            best.x_offset = x_offset;
            best.y_offset = y_offset;
            best.angle_offset = angle_offset;
            best.score = score;
        } else if(score2 > score && best.score < score2)
        {
            best.x_offset = x_offset;
            best.y_offset = y_offset;
            best.angle_offset = angle_offset+1;
            best.score = score2;
        }

        __syncthreads();
    }

    result[offset] = best;
}


//TODO: So far we've only been working on Localization, we need to start thinking about mapping or rather
//when to update the map with newly scanned points. I suspect that cold start might be a special case but
//it needs more thinking. I like the idea of the map being fully mutable, not just additive which is what
//I've seen from other SLAM impls.
__global__
void cudaUpdateMap(TelemetryPoint *scan_buffer, int *scan_size, MapPoint *map, int *map_width, int *map_height, LocalizedOrigin origin)
{
    int offset = blockIdx.x * blockDim.x + threadIdx.x;

    int map_size = *map_width * *map_height;
    /*for(int i = offset; i < map_size; i += gridDim.x * blockDim.x)
    {
        if(map[i].occupancy > 0)
        {
            //printf("MAP: i: %d, s: %d \n", map[i].occupancy);
            map[i].occupancy -= 1;
        }
    }

    //synchronizing across blocks would be better - TODO: get grid_group and sync working. Was facing linking errors with this.
    __syncthreads();
    */

    if(offset >= *scan_size)
    {
        return;
    }

    TelemetryPoint *cur_point = scan_buffer + offset;
    float distance = cur_point->distance;
    float angle_num = cur_point->angle + origin.angle_offset;

    int x = roundf(__sinf (angle_num) * distance) + origin.x_offset;
    int y = roundf(__cosf (angle_num) * distance) + origin.y_offset;
    
    int pos = ((*map_height / 2 + y) * *map_width) + (*map_height / 2 + x);

    if(pos > map_size)
    {
        return;
    }

    //printf("Point: x: %d y: %d, a: %.2f p: %d\n", cur_point->x, cur_point->y, cur_point->angle, pos);
    MapPoint *cur_map = map + pos;
    if(cur_map->occupancy < 3)
    {
        cur_map->occupancy++;
    }
    //printf("MAP: o: %d p: %d - x: %d. y: %d a:%.2f, q: %d\n", offset, pos, cur_point->x, cur_point->y, cur_point->angle, cur_point->quality);
}


TelemetryPoint Map::update(int32_t search_distance, TelemetryPoint scan_data[], int scan_size)
{

    cudaProfilerStart();
    cudaEvent_t startEvent, stopEvent;
    checkCuda( cudaEventCreate(&startEvent) );
    checkCuda( cudaEventCreate(&stopEvent) );
    checkCuda( cudaEventRecord(startEvent, 0) );

    const unsigned int bytes = scan_size * sizeof(TelemetryPoint);
    cudaMemcpy(scan_buffer_d, scan_data, bytes, cudaMemcpyHostToDevice);
    cudaMemcpy(scan_size_d, &scan_size, sizeof(int), cudaMemcpyHostToDevice);

    cudeGenerateParticleFilter <<< (scan_size * 360 / 1024) + 1, 1024, scan_size *sizeof(TelemetryPoint) >>> (sim_buffer_d, sim_size_d, scan_buffer_d, scan_size_d);
    cudaDeviceSynchronize();
    cudaRunParticleFilter <<< ceil((search_distance*search_distance)/256.0), 256, 2*scan_size *sizeof(SimTelemetryPoint)>>>(search_distance, result_d, sim_buffer_d, sim_size_d, scan_buffer_d, scan_size_d, map_d, width_d, height_d);
    cudaDeviceSynchronize();
    //shared memory must be >= threads per block
    int num_localization_blocks = 32;
    cudaLocalizeParticleFilter <<< num_localization_blocks, 128, 128*sizeof(LocalizedOrigin)>>>(localized_result_d, localized_size, result_d, map_update_dim * map_update_dim);
    cudaDeviceSynchronize();

    cudaMemcpy(localized_result_h, localized_result_d, localized_size*sizeof(LocalizedOrigin), cudaMemcpyDeviceToHost);
    checkCuda( cudaMemcpy(map_h, map_d, map_bytes, cudaMemcpyDeviceToHost));

    cudaDeviceSynchronize();

    LocalizedOrigin best;
    best.score = 0;
    best.x_offset = 0;
    best.y_offset = 0;
    best.angle_offset = 0;
    for(int i = 0; i < num_localization_blocks; i++){
        if(localized_result_h[i].score > best.score){
            best = localized_result_h[i];
        }
    }

    printf("BEST-FAST: x: %d  y: %d  a: %.2f  s: %d\n", best.x_offset, best.y_offset, best.angle_offset, best.score);

    cudaUpdateMap <<< 32, 256 >>> (scan_buffer_d, scan_size_d, map_d, width_d, height_d, best);
   
    checkCuda( cudaEventRecord(stopEvent, 0) );
    checkCuda( cudaEventSynchronize(stopEvent) );

    float time;
    checkCuda( cudaEventElapsedTime(&time, startEvent, stopEvent) );
    checkCuda( cudaEventDestroy(startEvent) );
    checkCuda( cudaEventDestroy(stopEvent) );

    printf("Map::update processed %d points and took %.2f ms\n", scan_size, time);
    cudaDeviceSynchronize();

    CheckpointWriter::checkpoint("cuda", width, height, scan_data, scan_size, map_h, &best);

    cudaProfilerStop();
    return TelemetryPoint{0, 0, 0, 0};
}