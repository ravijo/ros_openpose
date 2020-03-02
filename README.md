# ros_openpose

ROS wrapper for OpenPose | It supports *(currently but others are planned)*-

- [x] Intel Realsense Camera :heavy_check_mark:
- [x] Microsoft Kinect v2 Camera :heavy_check_mark:
- [x] Any color camera such as webcam etc :heavy_check_mark:

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


## Troubleshooting
1. While compiling the package, if the following error is reported at the terminal-
    ```
    error: no match for ‘operator=’ (operand types are ‘op::Matrix’ and ‘const cv::Mat’)
    ```
    In this case, please update the OpenPose. Most likely, an old version of OpenPose is installed. So please checkout Openpose from the master branch as [described here](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md#update-openpose). Alternatively, you can checkout OpenPose version 1.5.1 by running the following command at the root directory of OpenPose installation-


       git checkout tags/v1.5.1


    Do not forget to run `sudo make install` to install the OpenPose system-wide.


## Steps to run
1. Make sure that ROS env is sourced properly by executing the following command-
    ```
   source devel/setup.bash
    ```
1. Invoke the single launch file by executing the following command-
    ```
   roslaunch ros_openpose run.launch
    ```

The standard openpose command-line arguments are also supported. To do so, please set the value of `openpose_args` by editing the [run.launch](https://github.com/ravijo/ros_openpose/blob/98e928c883474eace8c71f588b74bf25666ee9ca/launch/run.launch#L30) file as shown below-

```
<arg name="openpose_args" value="--face --hand"/>
```

## Run with Intel Realsense Camera
```
roslaunch ros_openpose run.launch camera:=realsense
```

## Run with Microsoft Kinect v2 Camera
```
roslaunch ros_openpose run.launch camera:=kinect
```

## Run with Any color Camera such as Webcam etc
```
roslaunch ros_openpose run.launch camera:=nodepth
```



## Note
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


## Issues (or Error Reporting)
Please check [here](https://github.com/ravijo/ros_openpose/issues) and create issues accordingly.
