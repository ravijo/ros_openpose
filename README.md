# ros_openpose
ROS wrapper for OpenPose

<p align="center">
    <img src="files/ros_openpose.gif", width="800">
    <br>
    <sup>Sample video showing visualization on RViz</sup>
</p>


### Steps to run
1. `source devel/setup.bash`
1. `roslaunch realsense2_camera rs_camera.launch`
1. `rosrun ros_openpose rosOpenpose`

Standard OpenPose flags can be used as well. For example, to disable multi threading:
```
rosrun ros_openpose rosOpenpose --disable_multi_thread
```

*Please note that this is a beta version. At present, it is tested only with realsense camera. However, I am going to make it generic very soon.*
