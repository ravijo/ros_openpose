# ros_openpose

ROS wrapper for OpenPose | It supports *(currently but others are planned)*-

:heavy_check_mark: Intel Realsense Camera

</br>

<p align="center">
    <img src="files/ros_openpose.gif", width="800">
    </br>
    <sup>Sample video showing visualization on RViz</sup>
</p>


## Dependencies
1. [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)
1. [realsense-ros](https://github.com/IntelRealSense/realsense-ros)


## Compilation
1. Make sure to download the complete repository. Use `git clone https://github.com/ravijo/ros_openpose.git` or download zip as per your convenience.
1. Invoke catkin tool inside ros workspace i.e., `catkin_make`


### Steps to run
1. Make sure that ROS env is sourced properly by executing the following command `source devel/setup.bash`
1. Invoke the single launch file by executing the following command `roslaunch ros_openpose run.launch`

### Note
This package has been tested on the following environment configuration-

| Name      | Value                                  |
| ----------| -------------------------------------- |
| OS        | Ubuntu 14.04.6 LTS (64-bit)            |
| RAM       | 16 GB                                  |
| Processor | Intel® Core™ i7-7700 CPU @ 3.60GHz × 8 |
| Kernel    | Version 4.4.0-148-generic              |
| ROS       | Indigo                                 |
| GCC       | Version 5.5.0                          |
| OpenCV    | Version 2.4.8                          |
| OpenPose  | Version 1.5.1                          |
| GPU       | GeForce GTX 1080                       |
| CUDA      | Version 8.0.61                         |
| cuDNN     | Version 5.1.10                         |


