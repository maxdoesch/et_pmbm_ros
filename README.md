# ET-GGIW-PMBM ROS

ROS package for the C++ implementation of the Extended Target Gamma Gaussian Inverse Wishart Poisson Multi-Bernoulli Mixture Filter created for a Bachelor's Thesis at the University of Applied Sciences Nuremberg.

`Design and Implementation of a Lidar-Based Multiple Object Tracking System for Autonomous Shunting Locomotives` 

# Requirements
`OpenCV`
`Boost`
`Eigen`
`ROS`


# Build
```sh
mkdir -p et_pmbm_workspace/src && cd et_pmbm_workspace/src
git clone --recurse-submodules https://github.com/maxdoesch/et_pmbm_ros.git
catkin build
```
