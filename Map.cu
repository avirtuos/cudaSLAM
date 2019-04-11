
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
void cudaUpdateMap(int *result, TelemetryPoint *scan_buffer, int *scan_size, MapPoint *map, int* map_width, int* map_height)
{
    *result=0;
    int offset = blockIdx.x*blockDim.x + threadIdx.x;
    printf("Hello from block %d, dim %d, thread %d, offset: %d\n", blockIdx.x, blockDim.x, threadIdx.x, offset);
    
    for(int i = 0; i < *scan_size || i < 10; i = i + (1 + offset)){
        TelemetryPoint *cur_point = scan_buffer+i;
        int pos = ((*map_height/2 + cur_point->y) * *map_width) + (*map_height/2 + cur_point->x);
        MapPoint *cur_map = map+pos;
        if(cur_map->occupancy < 1000) {
            cur_map->occupancy++;
        }
        *result = *result +1; 
    }
}

TelemetryPoint Map::update(int32_t search_distance, TelemetryPoint scan_data[], int scan_size)
{
    cudaEvent_t startEvent, stopEvent;
    checkCuda( cudaEventCreate(&startEvent) );
    checkCuda( cudaEventCreate(&stopEvent) );
    checkCuda( cudaEventRecord(startEvent, 0) );

    printf("HERE1\n");
    const unsigned int bytes = scan_size * sizeof(TelemetryPoint);
    cudaMemcpy(scan_buffer_d, scan_data, bytes, cudaMemcpyHostToDevice);
    cudaMemcpy(scan_size_d, &scan_size, sizeof(int), cudaMemcpyHostToDevice);

    int *result_d;
    int *result_h;
    checkCuda(cudaMalloc((void **)&result_d, sizeof(int)));
    checkCuda(cudaMallocHost((void **)&result_h, sizeof(int)));

printf("HERE2\n");
    cudaUpdateMap<<<1, 1>>>(result_d, scan_buffer_d, scan_size_d, map_d, width_d, height_d);
printf("HERE3\n");

    cudaMemcpy(map_h, map_d, map_bytes, cudaMemcpyDeviceToHost);
    cudaMemcpy(result_h, result_d, sizeof(int), cudaMemcpyDeviceToHost);

    printf("Result: %d\n", *result_h);

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