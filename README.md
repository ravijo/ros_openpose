# ros_openpose
ROS wrapper for OpenPose

### Steps to run
1. `source devel/setup.bash`
1. `roslaunch realsense2_camera rs_camera.launch`
1. `rosrun ros_openpose rosOpenpose`

Standard OpenPose flags can be used as well. For example, to disable multi threading:
```
rosrun ros_openpose rosOpenpose --disable_multi_thread
```
