

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "toojpeg.h"
#include "MapPoint.h"
#include "CudaUtils.h"
#include <math.h>
#include <mutex>          // std::mutex, std::unique_lock
#include <cuda_runtime_api.h>
#include <cuda.h>
#include <string.h>

using namespace std;

class Map
{

public:
    Map(int32_t width_arg, int32_t height_arg, int32_t scan_buffer);
    TelemetryPoint update(int32_t search_distance, TelemetryPoint scan_data[], int scan_size);
    ~Map();


private:
    int width;
    int height;
    MapPoint *map_h;
    MapPoint *map_d;
    int* width_d;
    int* height_d;
    TelemetryPoint *scan_buffer_d;
    int *scan_size_d;
};

Map::Map(int width_arg, int height_arg, int scan_buffer_size)
{
    width = width_arg;
    height = height_arg;

    //Allocate scan buffer on the device as well as the size of scan
    const unsigned int scan_bytes = scan_buffer_size * sizeof(TelemetryPoint);
    checkCuda(cudaMalloc((void **)&scan_buffer_d, scan_bytes));
    checkCuda(cudaMalloc((void **)&scan_size_d, sizeof(int)));

    //Allocate pinned memory on the host and device for the current map
    const unsigned int map_bytes = width * height * sizeof(MapPoint);
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
    cudaFree(scan_size_d);
    cudaFree(scan_size_d);
}

TelemetryPoint Map::update(int32_t search_distance, TelemetryPoint scan_data[], int scan_size)
{
    cudaEvent_t startEvent, stopEvent;
    checkCuda( cudaEventCreate(&startEvent) );
    checkCuda( cudaEventCreate(&stopEvent) );
    checkCuda( cudaEventRecord(startEvent, 0) );

    const unsigned int bytes = scan_size * sizeof(TelemetryPoint);
    int *h_a = (int *)malloc(bytes);
    TelemetryPoint *d_a;
    cudaMalloc((void **)&d_a, bytes);
    cudaMemcpy(d_a, scan_data, bytes, cudaMemcpyHostToDevice);

    checkCuda( cudaEventRecord(stopEvent, 0) );
    checkCuda( cudaEventSynchronize(stopEvent) );

    float time;
    checkCuda( cudaEventElapsedTime(&time, startEvent, stopEvent) );
    checkCuda( cudaEventDestroy(startEvent) );
    checkCuda( cudaEventDestroy(stopEvent) );


    printf("Map::update processed %d points and took %.2f ms\n", scan_size, time);

    return TelemetryPoint{0, 0, 0, 0, 0};
}