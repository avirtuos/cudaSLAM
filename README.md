# cudaSLAM
Simultaneous Localization And Mapping using lidar and CUDA

This library is a work in progress and was started as an alternative to hector_slam and gmapping modules in ROS because I had so much trouble
getting those libraries to work well. I've also been tinkering with Nvidia's Xavier and Jetson Nano boards so this gave me a good project to
use CUDA with.

### Usage

```bash
git clone https://github.com/avirtuos/cudaSLAM.git
cd cudaSLAM
make
sudo ./cudaSLAM /dev/ttyUSB0 256000
```

The app will output a jpg file for each lazer scan and each run of the SLAM algo. Keep in mind this is still under active development and so these outputs are mostly for debuging purposes and of little value to an actual robotics system.

### Project Status
  As of 5/1/2019 I have a basic Particle Filter working for Localization. Given a lazer scan, a Jetson Xavier can locate itself within a 1 m^2 area of uncertainty in under 100ms without odometry. With basic odometry, really just a guess of distance traveled since last sample, the localization time goes down to 10ms or the rate of travel between localization atempts increased to 10 m/s.
  
  My next step is add support for 'basic' odometry where you can give the alog either an educated guess, or actual odometry from motor encoders in order to speed up the SLAM algo by narrowing its search space. I will also attempt to add a reverse re-sampling algo where by we start with an optomistically narrow search space for the partcile filter and then incease it if we do not find a reasonably accurate location. I suspect that for most hobbiest rovers this will be excellent performance with little downside.
  
  General todo items:
  
  1. Resampling - allows us to start with low resolution, high-speed, particle filters to narrow the search space before using high detail particle filters to get an accurate location.
  1. Mapping - right now we are using an extremely naieve approach to mapping wich favors recent scans but allows older scan data to decay off the map. This works fine in small environments where the laser scans see the bulk of the evnironment frequently but works poorly in large environments with many line of sight obstructions. My plan is to have scans decay unless they accumulate a sufficient ammount of reenfocement. I also plan on exposing some key APIs which the motion control system can use to take a checkpoint of the map, force and update, or generate a rollback. This, combined with auto-rollback if a 'jump' is detected should prevent issues that were common to hector_slam.

## Particle Fiters 

I found [this video and the thumbnail from it below](https://www.youtube.com/watch?v=aUkBa1zMKv4) to be extremely helpful in simplifying and understand how a particle filter can be used to help determine the location of our robot given relatively simple measuring tools like lidar and a guess of our speed or travel distance. Combined with a highly parallel processor (GPU) we can simulate Billions of Particle location hypothesis per second. 

[![IMAGE ALT TEXT HERE](https://raw.githubusercontent.com/avirtuos/cudaSLAM/master/docs/img/particple_filter.png)](https://www.youtube.com/watch?v=aUkBa1zMKv4)

#Performance Analysis

This section is mostly a place holder for now but I'll use it to keep track of nvprof results as this package matures. For those curious or planning to use these code as an example or basis for a fork. Here are some of the more recent perf results with using an RPLidar A3 w/Jetson Xavier.

```
            Type  Time(%)      Time     Calls       Avg       Min       Max  Name
 GPU activities:   51.00%  772.68ms        10  77.268ms  74.725ms  96.939ms  cudaLocalizeParticleFilter(LocalizedOrigin*, int)
                   45.74%  693.03ms        10  69.303ms  56.365ms  149.60ms  cudaRunParticleFilter(int, LocalizedOrigin*, SimTelemetryPoint*, int*, TelemetryPoint*, int*, MapPoint*, int*, int*)
                    2.23%  33.716ms        10  3.3716ms  2.9363ms  7.2335ms  cudaUpdateMap(TelemetryPoint*, int*, MapPoint*, int*, int*)
                    0.97%  14.698ms        10  1.4698ms  1.4066ms  2.0358ms  [CUDA memcpy DtoH]
                    0.06%  873.25us        10  87.324us  62.308us  249.52us  cudeGenerateParticleFilter(SimTelemetryPoint*, int*, TelemetryPoint*, int*)
                    0.00%  50.529us        23  2.1960us     416ns  9.0570us  [CUDA memcpy HtoD]
      API calls:   82.81%  1.50384s        40  37.596ms  51.747us  149.68ms  cudaDeviceSynchronize
                   13.88%  252.01ms         2  126.01ms  1.6958ms  250.32ms  cudaMallocHost
                    1.23%  22.276ms        33  675.03us  72.580us  2.3089ms  cudaMemcpy
                    0.92%  16.756ms        20  837.82us  13.697us  1.9338ms  cudaEventCreate
                    0.60%  10.906ms        40  272.64us  131.11us  784.87us  cudaLaunchKernel
                    0.37%  6.6333ms         8  829.16us  40.290us  3.3672ms  cudaMalloc
                    0.11%  1.9580ms        20  97.899us  54.307us  157.80us  cudaEventRecord
                    0.02%  377.65us        20  18.882us  6.6240us  62.883us  cudaEventDestroy
                    0.02%  376.31us        96  3.9190us  2.4640us  50.466us  cuDeviceGetAttribute
                    0.02%  321.04us        10  32.104us  19.169us  61.827us  cudaEventSynchronize
                    0.02%  293.17us        10  29.316us  14.145us  55.299us  cudaEventElapsedTime
                    0.01%  98.405us         1  98.405us  98.405us  98.405us  cudaProfilerStart
                    0.00%  69.027us         1  69.027us  69.027us  69.027us  cudaGetDeviceProperties
                    0.00%  28.706us         1  28.706us  28.706us  28.706us  cudaGetDeviceCount
                    0.00%  19.201us         2  9.6000us  8.4800us  10.721us  cuDeviceGetCount
                    0.00%  15.425us         1  15.425us  15.425us  15.425us  cuDeviceTotalMem
                    0.00%  15.138us         2  7.5690us  4.7370us  10.401us  cuDeviceGet
                    0.00%  7.9690us         1  7.9690us  7.9690us  7.9690us  cuDeviceGetUuid
                    0.00%  4.5760us         1  4.5760us  4.5760us  4.5760us  cuDeviceGetName
```

## Notes

Activating the fan on the Jetson Xavier is as simple as 
```bash
echo 100 | sudo tee /sys/kernel/debug/tegra_fan/target_pwm
```

