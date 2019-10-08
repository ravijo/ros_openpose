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


## Compilation
1. Make sure to download the complete repository. Use `git clone https://github.com/ravijo/ros_openpose.git` or download zip as per your convenience.
1. Invoke catkin tool inside ros workspace i.e., `catkin_make`


### Steps to run
1. Make sure that ROS env is sourced properly by executing the following command `source devel/setup.bash`
1. Invoke the single launch file by executing the following command `roslaunch ros_openpose run.launch`
