

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "toojpeg.h"
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
    Map(int32_t width_arg , int32_t height_arg, int32_t scan_buffer);
    TelemetryPoint update(int32_t search_distance, TelemetryPoint scan_data[], int scan_size);
    ~Map();


private:
	int32_t width;
	int32_t height;
	TelemetryPoint *scan_buffer_d;
	int *scan_size_d;
};

Map::Map(int32_t width_arg, int32_t height_arg, int32_t scan_buffer_size){
	width = width_arg;
	height = height_arg;

	const unsigned int bytes = scan_buffer_size * sizeof(TelemetryPoint);
	cudaMalloc((void**)&scan_buffer_d, bytes);
   	cudaMalloc((void**)&scan_size_d, sizeof(int));
}

Map::~Map(){
	cudaFree(scan_size_d);
	cudaFree(scan_size_d);
}

TelemetryPoint Map::update(int32_t search_distance, TelemetryPoint scan_data[], int scan_size){
	cudaEvent_t startEvent, stopEvent; 
  	checkCuda( cudaEventCreate(&startEvent) );
  	checkCuda( cudaEventCreate(&stopEvent) );
	checkCuda( cudaEventRecord(startEvent, 0) );

    const unsigned int bytes = scan_size * sizeof(TelemetryPoint);
    int *h_a = (int*)malloc(bytes);
    TelemetryPoint *d_a;
    cudaMalloc((void**)&d_a, bytes);
    cudaMemcpy(d_a, scan_data, bytes, cudaMemcpyHostToDevice);

	checkCuda( cudaEventRecord(stopEvent, 0) );
  	checkCuda( cudaEventSynchronize(stopEvent) );

  	float time;
  	checkCuda( cudaEventElapsedTime(&time, startEvent, stopEvent) );
	checkCuda( cudaEventDestroy(startEvent) );
	checkCuda( cudaEventDestroy(stopEvent) );

  	printf("Map::update took %.2f ms\n", time);

    return TelemetryPoint(0,0,0,0,0);
}