
#include "Map.h"

Map::Map(int width_arg, int height_arg, int scan_buffer_size)
{
    width = width_arg;
    height = height_arg;

    //Allocate scan buffer on the device as well as the size of scan
    const unsigned int scan_bytes = scan_buffer_size * sizeof(TelemetryPoint);
    checkCuda(cudaMalloc((void **)&scan_buffer_d, scan_bytes));
    checkCuda(cudaMalloc((void **)&scan_size_d, sizeof(int)));

    //Allocate pinned memory on the host and device for the current map
    map_bytes = width * height * sizeof(MapPoint);
    checkCuda(cudaMallocHost((void**)&map_h, map_bytes));
    checkCuda(cudaMalloc((void**)&map_d, map_bytes));
    checkCuda(cudaMalloc((void **)&width_d, sizeof(int)));
    checkCuda(cudaMalloc((void **)&height_d, sizeof(int)));

    //copy the size of the map to the device
    //todo: in the future this should probably be auto-expandable based on the size of the mapped area
    //but evne using a static size is fine for area 30m x 30m which is more than enough for most hobby applications
    cudaMemcpy(width_d, &width, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(height_d, &height, sizeof(int), cudaMemcpyHostToDevice);
}

Map::~Map()
{
    cudaFreeHost(map_h);
    cudaFree(map_d);
    cudaFree(width_d);
    cudaFree(height_d);
    cudaFree(scan_buffer_d);
    cudaFree(scan_size_d);
}

__global__
void cudaUpdateMap(int n, int *result, TelemetryPoint *scan_buffer, int *scan_size, MapPoint *map, int* map_width, int* map_height)
{
    int offset = blockIdx.x*blockDim.x + threadIdx.x;
    result[offset]=0;

    int x_offset = 0;
    int y_offset = 0;

    LocalizedOrigin best;
    best.score = -1;

    //Try various angles - TODO: find better sampling technique here possibly even re-sampling
    for(int angle_offset = 0; angle_offset < 360; angle_offset++){
        //For each point see if we have a hit

        LocalizedOrigin current_sim;
        current_sim.x_offset = x_offset;
        current_sim.y_offset = y_offset;
        current_sim.angle_offset = angle_offset;
        current_sim.score = 0;

        for(int scan_point = 0; scan_point < scan_size; scan_point++){
            int angle_offset = 0;

            float distance = scan_buffer[scan_point].distance;
            float angle_radians = scan_buffer[scan_point].angle;

            int x = x_offset + roundf(sin (angle_offset + (angle_radians * PI / 180)) * distance);
            int y = y_offset + roundf(cos (angle_offset + (angle_radians * PI / 180)) * distance);

            int pos = ((*map_height/2 + y) * *map_width) + (*map_width/2 + x);
            MapPoint *map_point = map+pos;
            if(map_point->occupancy > 0){
                current_sim.score++;
            }
        }

        if(best.score < current_sim.score) {
            best.score = current_sim.score;
        }
    }

    if(offset > n || offset > *scan_size) {
        return;
    }


    //printf("Hello from block %d, dim %d, thread %d, offset: %d\n", blockIdx.x, blockDim.x, threadIdx.x, offset);
    
    for(int i = offset; i < *scan_size; i = i + n){
        TelemetryPoint *cur_point = scan_buffer+i;
        int pos = ((*map_height/2 + cur_point->y) * *map_width) + (*map_height/2 + cur_point->x);
        MapPoint *cur_map = map+pos;
        if(cur_map->occupancy < 1000) {
            cur_map->occupancy++;
        }
        result[offset] = result[offset] + 1;
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

    int dim = 1024;
    int n = dim * dim;
    LocalizedOrigin *result_d;
    LocalizedOrigin *result_h;
    checkCuda(cudaMalloc((void **)&result_d, n*sizeof(LocalizedOrigin)));
    checkCuda(cudaMallocHost((void **)&result_h, n*sizeof(LocalizedOrigin)));

    cudaUpdateMap<<<dim, dim>>>(n, result_d, scan_buffer_d, scan_size_d, map_d, width_d, height_d);

    cudaMemcpy(map_h, map_d, map_bytes, cudaMemcpyDeviceToHost);
    cudaMemcpy(result_h, result_d, n*sizeof(LocalizedOrigin), cudaMemcpyDeviceToHost);

    for(int i = 0; i < n && i < scan_size; i++){
        printf("Particle Filter Result: i[%d] x[%d] y[%d] angle[%.2f] quality[%d]\n", result_h[i].x_offset, result_h[i].y_offset, result_h[i].angle_offset, result_h[i].quality);
    }

    cudaFreeHost(result_h);
    cudaFree(result_d);

    checkCuda( cudaEventRecord(stopEvent, 0) );
    checkCuda( cudaEventSynchronize(stopEvent) );

    float time;
    checkCuda( cudaEventElapsedTime(&time, startEvent, stopEvent) );
    checkCuda( cudaEventDestroy(startEvent) );
    checkCuda( cudaEventDestroy(stopEvent) );

    printf("Map::update processed %d points and took %.2f ms\n", scan_size, time);

    CheckpointWriter::checkpoint("cuda", 2000,2000, scan_data, scan_size, map_h);

    return TelemetryPoint{0, 0, 0, 0, 0};
}