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

// c++ header
#include <vector>

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
    ros::Subscriber mCamInfoSubscriber;

    // camera calibration parameters
    std::shared_ptr<sensor_msgs::CameraInfo> mSPtrCameraInfo;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> mSPtrColorImageSub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> mSPtrDepthImageSub;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>> mSPtrSyncSubscriber;

    inline void subscribe();
    void imageCallback(const sensor_msgs::ImageConstPtr& colorMsg, const sensor_msgs::ImageConstPtr& depthMsg);
    void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& camMsg);

  public:
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
    const cv::Mat& getColorFrame()
    {
      return mColorImage;
    }

    // get the depth image from camera
    const cv::Mat& getDepthFrame()
    {
      return mDepthImage;
    }

    // compute pixel to 3D without considering distortion
    void compute3DPoint(const float pixel_x, const float pixel_y, float (&point)[3])
    {
      /*
      * K.at(0) = intrinsic.fx
      * K.at(4) = intrinsic.fy
      * K.at(2) = intrinsic.ppx
      * K.at(5) = intrinsic.ppy
      */

      // our depth image type is 16UC1 which has unsigned short as an underlying type
      auto depth = mDepthImage.at<unsigned short>(static_cast<int>(pixel_y), static_cast<int>(pixel_x));

      // convert to meter (SI units)
      auto depthSI = depth * 0.001f;

      auto x = (pixel_x - mSPtrCameraInfo->K.at(2)) / mSPtrCameraInfo->K.at(0);
      auto y = (pixel_y - mSPtrCameraInfo->K.at(5)) / mSPtrCameraInfo->K.at(4);

      point[0] = depthSI * x;
      point[1] = depthSI * y;
      point[2] = depthSI;
    }
  };
}
