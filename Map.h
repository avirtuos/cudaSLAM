
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "toojpeg.h"
#include "MapPoint.h"
#include "CudaUtils.h"
#include "CheckpointWriter.h"
#include "TelemetryPoint.h"
#include "LocalizedOrigin.h"
#include "MapPoint.h"
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
    int map_bytes;
    TelemetryPoint *scan_buffer_d;
    int *scan_size_d;
};
