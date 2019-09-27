/**
* CameraReader.hpp: header file for CameraReader
* Author: Ravi Joshi
* Date: 2019/02/01
*/

#pragma once

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// CV brigge header
#include <cv_bridge/cv_bridge.h>

// OpenCV header
#include <opencv2/core/core.hpp>

// std::vector header
#include <vector>

namespace ros_openpose
{
  class CameraReader
  {
  private:
    cv::Mat mImage;
    std::string mTopicName;
    image_transport::ImageTransport mIt;
    image_transport::Subscriber mSubscriber;

    inline void subscribe();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  public:
    // camera calibration parameters
    cv::Mat mCameraExtrinsics;
    cv::Mat mCameraIntrinsics;

    // we don't want to instantiate using deafult constructor
    CameraReader() = delete;

    // copy constructor
    CameraReader(const CameraReader& other);

    // copy assignment operator
    CameraReader& operator=(const CameraReader& other);

    // main constructor
    CameraReader(image_transport::ImageTransport& it, const std::string& topicName);

    // destructor
    ~CameraReader();

    // get the image from camera
    const cv::Mat getFrame()
    {
      return mImage;
    }

    // get the camera extrinsic parameter
    const cv::Mat getCameraExtrinsics()
    {
      return mCameraExtrinsics;
    }

    // get the camera intrinsic parameter
    const cv::Mat getCameraIntrinsics()
    {
      return mCameraIntrinsics;
    }
  };
}
