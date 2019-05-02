/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <cuda_runtime_api.h>
#include <cuda.h>
#include<cuda_profiler_api.h>
#include "LaserScan.h"
#include "Map.h"
#include "CheckpointWriter.h"
#include "TelemetryPoint.h"
#include "CudaUtils.h"
#include "MotionSystem.h"

#define DEBUG

using namespace std;
using namespace std::chrono;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}



int main(int argc, const char *argv[])
{
    int deviceCount;
    cudaGetDeviceCount(&deviceCount);
    int device;
    for (device = 0; device < deviceCount; ++device)
    {
        cudaDeviceProp deviceProp;
        cudaGetDeviceProperties(&deviceProp, device);
        printf("Device %d has compute capability %d.%d.\n", device, deviceProp.major, deviceProp.minor);
        printf("  Device name: %s\n", deviceProp.name);
        printf("  Memory Clock Rate (KHz): %d\n", deviceProp.memoryClockRate);
        printf("  Memory Bus Width (bits): %d\n", deviceProp.memoryBusWidth);
        printf("  Peak Memory Bandwidth (GB/s): %f\n\n", 2.0 * deviceProp.memoryClockRate * (deviceProp.memoryBusWidth / 8) / 1.0e6);
        printf("  Max Shared Mem Bytes Per-Block: %lu\n\n", deviceProp.sharedMemPerBlock);
        printf("  Max Threads Per-Block: %d\n\n", deviceProp.maxThreadsPerBlock);
        printf("  Warp Size: %d\n\n", deviceProp.warpSize);
    }

    const char *default_com_path = "/dev/ttyUSB0";
    char *com_path = (char*)default_com_path;
    if(argc > 1){
        com_path = (char*)argv[1];
    }
    printf("LiDAR COM Port %s \n", com_path);

    _u32 com_baudrate = 256000;
    if(argc > 1){
        com_baudrate = strtoul(argv[2], NULL, 10);
    }
    printf("LiDAR BAUD Rate %ud \n", com_baudrate);
    
    int search_distance = 100;
    if(argc > 3){
        search_distance = strtoul(argv[3], NULL, 10);
    }
    printf("SLAM search distance is %d cm\n", search_distance*10);

    int dir = 0;
    if(argc > 4){
        dir = strtoul(argv[4], NULL, 10);
        printf("DIR %d \n", dir);
    }

    //blue tooth address of arduino (or other) motion control system
    char dest[18] = "00:1B:10:80:13:ED";
    MotionSystem ms(dest);

    LaserScan laser(com_path, com_baudrate);
    laser.start();

    signal(SIGINT, ctrlc);
    high_resolution_clock::time_point t1, t2;

    //TODO: rplidar examples have this set to ~9000 but I've never seen more than ~2000 samples in a single scan, maybe we can reduce this
    const int scan_buffer_size = 3000;
    TelemetryPoint *h_scan_p = NULL;

    //We used Pinned Host Memory because it tends to be 2x faster than pageable host memory when transfering to/from
    //source: https://devblogs.nvidia.com/how-optimize-data-transfers-cuda-cc/
    checkCuda(cudaMallocHost((void **)&h_scan_p, scan_buffer_size * sizeof(TelemetryPoint)));

    int map_size = 2000;
    Map map(map_size, map_size, scan_buffer_size);

    int count = 0;
    while(count < 10)
    {
        t1 = high_resolution_clock::now();
        int num_scan_samples = laser.scan(h_scan_p, scan_buffer_size);
        t2 = high_resolution_clock::now();
        auto scan_dur = duration_cast<milliseconds>( t2 - t1 ).count();

        t1 = high_resolution_clock::now();
        CheckpointWriter::advanceCheckpoint();
        CheckpointWriter::checkpoint("scan", map_size,map_size, h_scan_p, num_scan_samples);
        map.update(search_distance, h_scan_p, num_scan_samples);
        t2 = high_resolution_clock::now();
        auto cp_dur = duration_cast<milliseconds>( t2 - t1 ).count();

        cout << "Scan Dur: " << scan_dur << " ms" << " SLAM Dur: " << cp_dur << "ms " <<  " with search area of " << (search_distance*search_distance)/(100*100) << " meters" << endl;
        if (ctrl_c_pressed)
        {
            break;
        }

        count++;

        if(dir == 1){
            ms.forward();
        } else if(dir == 2) {
            ms.backward();
        }
        /*
                int c = getchar();
                if(c == 10){
                    //no-op
                } else if(c == 32){
                    ms.stop();
                } else if(c == 119){
                    ms.forward();
                } else if(c == 115){
                    ms.backward();
                } else if(c == 97){
                    ms.left();
                } else if(c == 100){
                    ms.right();
                } else {
                    printf("Unknown key: %c %d\n",c,c);
                }
                */
    }

    laser.stop();
    ms.stop();
    cudaFreeHost(h_scan_p);
    return 0;
}

