# cudaSLAM
Simultaneous Localization And Mapping using lidar and CUDA

This library is a work in progress and was started as an alternative to hector_slam and gmapping modules in ROS because I had so much trouble
getting those libraries to work well. I've also been tinkering with Nvidia's Xavier and Jetson Nano boards so this gave me a good project to
use CUDA with.

### Project Status
  As of 4/18/2019 I have a basic Particle Filter working for Localization. Given a lazer scan, a Jetson Xavier can locate itself within a 1 m^2 area of uncertainty in under 100ms without odometry. With basic odometry, really just a guess of distance traveled since last sample, the localization time goes down to 10ms or the rate of travel between localization atempts increased to 10 m/s.
  
  My step step is to improve the performance of the last step of localization which gathers the results of the Particle Filter. This will clear the way to adding a resampling mechanism that with futher improve performance, both accuracy and speed.
  
  General todo items:
  
  1. Resampling - allows us to start with low resolution, high-speed, particle filters to narrow the search space before using high detail particle filters to get an accurate location.
  1. Mapping - right now we are using an extremely naieve approach to mapping wich favors recent scans but allows older scan data to decay off the map. This works fine in small environments where the laser scans see the bulk of the evnironment frequently but works poorly in large environments with many line of sight obstructions. My plan is to have scans decay unless they accumulate a sufficient ammount of reenfocement. I also plan on exposing some key APIs which the motion control system can use to take a checkpoint of the map, force and update, or generate a rollback. This, combined with auto-rollback if a 'jump' is detected should prevent issues that were common to hector_slam.

## Particle Fiters 

I found [this video and the thumbnail from it below](https://www.youtube.com/watch?v=aUkBa1zMKv4) to be extremely helpful in simplifying and understand how a particle filter can be used to help determine the location of our robot given relatively simple measuring tools like lidar and a guess of our speed or travel distance. Combined with a highly parallel processor (GPU) we can simulate Billions of Particle location hypothesis per second. 

[![IMAGE ALT TEXT HERE](https://raw.githubusercontent.com/avirtuos/cudaSLAM/master/docs/img/particple_filter.png)](https://www.youtube.com/watch?v=aUkBa1zMKv4)
