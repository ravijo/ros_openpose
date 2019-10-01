/**
* CameraReader.hpp: header file for CameraReader
* Author: Ravi Joshi
* Date: 2019/02/01
*/

#pragma once

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// CV brigge header
#include <cv_bridge/cv_bridge.h>

// OpenCV header
#include <opencv2/core/core.hpp>

// c++ headers
#include <vector>
#include <mutex>

namespace ros_openpose
{
  class CameraReader
  {
  private:
    cv::Mat mColorImage;
    cv::Mat mDepthImage;
    std::string mColorTopic;
    std::string mDepthTopic;
    std::string mCamInfoTopic;
    ros::NodeHandle mNh;
    std::mutex mImageMutex;
    ros::Subscriber mCamInfoSubscriber;

    std::shared_ptr<sensor_msgs::CameraInfo> mSPtrCameraInfo;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> mSPtrColorImageSub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> mSPtrDepthImageSub;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>> mSPtrSyncSubscriber;

    inline void subscribe();
    void imageCallback(const sensor_msgs::ImageConstPtr& colorMsg, const sensor_msgs::ImageConstPtr& depthMsg);
    void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& camMsg);

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
    CameraReader(ros::NodeHandle& nh, const std::string& colorTopic, const std::string& depthTopic,
                 const std::string& camInfoTopic);

    // destructor
    ~CameraReader();

    // get the color image from camera
    const cv::Mat getColorFrame()
    {
      return mColorImage;
    }

    // get the depth image from camera
    const cv::Mat getDepthFrame()
    {
      return mDepthImage;
    }

    // compute pixel to 3D without considering distortion
    void compute3DPoint(const float pixel_x, const float pixel_y, const float depth, float (&point)[3])
    {
      /*
      * K.at(0) = intrinsic.fx
      * K.at(4) = intrinsic.fy
      * K.at(2) = intrinsic.ppx
      * K.at(5) = intrinsic.ppy
      */

      float x = (pixel_x - mSPtrCameraInfo->K.at(2)) / mSPtrCameraInfo->K.at(0);
      float y = (pixel_y - mSPtrCameraInfo->K.at(5)) / mSPtrCameraInfo->K.at(4);

      point[0] = depth * x;
      point[1] = depth * y;
      point[2] = depth;
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
