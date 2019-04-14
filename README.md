# cudaSLAM
Simultaneous Localization And Mapping using lidar and CUDA

This library is a work in progress and was started as an alternative to hector_slam and gmapping modules in ROS because I had so much trouble
getting those libraries to work well. I've also been tinkering with Nvidia's Xavier and Jetson Nano boards so this gave me a good project to
use CUDA with.

## Particle Fiters 

I found this video extremely helpful in simplifying and understand how a particle filter can be used to help determine the location of our robot given relatively simple measuring tools like lidar and a guess of our speed or travel distance. Combined with a highly parallel processor (GPU) we can simulate Billions of Particle location hypothesis per second. 

[![IMAGE ALT TEXT HERE](/docs/img/particple_filter.png.jpg?raw=true)](https://www.youtube.com/watch?v=aUkBa1zMKv4)
