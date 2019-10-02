# ros_openpose

ROS wrapper for OpenPose | It supports *(currently but others are planned)*-

:heavy_check_mark: Intel Realsense Camera


<p align="center">
    <img src="files/ros_openpose.gif", width="800">
    <br>
    <sup>Sample video showing visualization on RViz</sup>
</p>


### Steps to run
1. `source devel/setup.bash`
1. `roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=pointcloud`
1. `rosrun ros_openpose rosOpenpose`

Standard OpenPose flags can be used as well. For example, to disable multi threading:
```
rosrun ros_openpose rosOpenpose --disable_multi_thread
```

