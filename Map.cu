
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
    const unsigned int sim_bytes = sim_size_h*sizeof(SimTelemetryPoint);
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
    checkCuda(cudaMalloc((void **)&result_d, n*sizeof(LocalizedOrigin)));
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
}


__global__
void cudeGenerateParticleFilter(SimTelemetryPoint *sim_buffer, int *sim_size, TelemetryPoint *scan_buffer, int *scan_size)
{
    int sim_num = blockIdx.x * blockDim.x + threadIdx.x;

    if(sim_num > 360) {
        return;
    }

    int sim_offset = (sim_num * *scan_size);
    for(int scan_point = 0; scan_point < *scan_size; scan_point++)
    {
        float distance = scan_buffer[scan_point].distance;
        float angle_radians = scan_buffer[scan_point].angle;

        int pos = sim_offset + scan_point;
        if(pos >= *sim_size){
            return;
        }

        SimTelemetryPoint *sim = sim_buffer + pos;
        sim->x = roundf(__sinf (sim_num + angle_radians) * distance);
        sim->y = roundf(__cosf (sim_num + angle_radians) * distance);
    }
}


__global__
void cudaLocalizeParticleFilter(LocalizedOrigin *result, int result_size)
{
    LocalizedOrigin best;
    best.score = -1;
    for(int i = 0; i < result_size;i++){
        if(result[i].score > best.score){
            best.x_offset = result[i].x_offset;
            best.y_offset = result[i].y_offset;
            best.angle_offset = result[i].angle_offset;
            best.score = result[i].score;
        }
    }

    printf("BEST: x: %d y: %d, a: %.2f s: %d\n", best.x_offset, best.y_offset, best.angle_offset, best.score);
}

//TODO: There is opportunity to speed this up using hints from odometry or even just simple distance traveled estimates.
__global__
void cudaRunParticleFilter(int n, LocalizedOrigin *result, SimTelemetryPoint *sim_buffer, int *sim_size, TelemetryPoint *scan_buffer, int *scan_size, MapPoint *map, int *map_width, int *map_height)
{
    int offset = blockIdx.x * blockDim.x + threadIdx.x;
    int search_distance = 256;

    int x_offset = (-1 * search_distance) + (int)threadIdx.x;
    int y_offset = (-1 * search_distance) + (int)blockIdx.x;
    long max_pos = *map_width * *map_height;


    if(x_offset > search_distance || y_offset > search_distance)
    {
        result[offset].x_offset = -1;
        result[offset].y_offset = -1;
        result[offset].angle_offset = -1;
        result[offset].score = -1;
        return;
    }

    int e_width = (*map_width / 2);
    int e_height = (*map_height / 2);

    LocalizedOrigin best;
    best.score = -1;

    //Try various angles - TODO: find better sampling technique here possibly even re-sampling
    for(int angle_offset = 0; angle_offset < 360; angle_offset++)
    {
        //For each point see if we have a hit
        int score = 0;
        int scan_point_offset = angle_offset * *scan_size;

        for(int scan_point = 0; scan_point < *scan_size; scan_point++)
        {
            //todo: pre-calculate this
            int e_scan_point = scan_point_offset + scan_point;
            int x = x_offset + sim_buffer[e_scan_point].x;
            int y = y_offset + sim_buffer[e_scan_point].y;

            int pos = ((e_height + y) * *map_width) + (e_width + x);

            if(pos < 0 || pos >= max_pos)
            {
                continue;
            }

            MapPoint *map_point = map + pos;
            if(map_point->occupancy > 0)
            {
                score++;
            }
        }

        if(best.score < score)
        {
            best.x_offset = x_offset;
            best.y_offset = y_offset;
            best.angle_offset = angle_offset;
            best.score = score;
        }
    }

    result[offset].x_offset = best.x_offset;
    result[offset].y_offset = best.y_offset;
    result[offset].angle_offset = best.angle_offset;
    result[offset].score = best.score;
}


//TODO: So far we've only been working on Localization, we need to start thinking about mapping or rather
//when to update the map with newly scanned points. I suspect that cold start might be a special case but
//it needs more thinking. I like the idea of the map being fully mutable, not just additive which is what
//I've seen from other SLAM impls.
__global__
void cudaInitMap(TelemetryPoint *scan_buffer, int *scan_size, MapPoint *map, int *map_width, int *map_height)
{
    for(int i = 0; i < *scan_size; i++)
    {
        TelemetryPoint *cur_point = scan_buffer + i;
        int pos = ((*map_height / 2 + cur_point->y) * *map_width) + (*map_height / 2 + cur_point->x);
        MapPoint *cur_map = map + pos;
        if(cur_map->occupancy < 1000)
        {
            cur_map->occupancy++;
        }
    }
}


TelemetryPoint Map::update(int32_t search_distance, TelemetryPoint scan_data[], int scan_size)
{
    cudaEvent_t startEvent, stopEvent;
    checkCuda( cudaEventCreate(&startEvent) );
    checkCuda( cudaEventCreate(&stopEvent) );
    checkCuda( cudaEventRecord(startEvent, 0) );

    const unsigned int bytes = scan_size * sizeof(TelemetryPoint);
    cudaMemcpy(scan_buffer_d, scan_data, bytes, cudaMemcpyHostToDevice);
    cudaMemcpy(scan_size_d, &scan_size, sizeof(int), cudaMemcpyHostToDevice);
    
    if(!map_init)
    {
        cudaInitMap <<< 1, 1>>>(scan_buffer_d, scan_size_d, map_d, width_d, height_d);
        map_init = true;
    }

    cudeGenerateParticleFilter<<<1,512>>>(sim_buffer_d, sim_size_d, scan_buffer_d, scan_size_d);
    cudaRunParticleFilter <<<map_update_dim, map_update_dim>>>(map_update_dim*map_update_dim, result_d, sim_buffer_d, sim_size_d, scan_buffer_d, scan_size_d, map_d, width_d, height_d);
    cudaLocalizeParticleFilter<<<1,1>>>(result_d,map_update_dim*map_update_dim);

    cudaMemcpy(map_h, map_d, map_bytes, cudaMemcpyDeviceToHost);
    
    checkCuda( cudaEventRecord(stopEvent, 0) );
    checkCuda( cudaEventSynchronize(stopEvent) );

    float time;
    checkCuda( cudaEventElapsedTime(&time, startEvent, stopEvent) );
    checkCuda( cudaEventDestroy(startEvent) );
    checkCuda( cudaEventDestroy(stopEvent) );

    printf("Map::update processed %d points and took %.2f ms\n", scan_size, time);

    //CheckpointWriter::checkpoint("cuda", 2000, 2000, scan_data, scan_size, map_h);


    return TelemetryPoint{0, 0, 0, 0, 0};
}