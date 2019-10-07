/**
* CameraReader.hpp: header file for CameraReader
* Author: Ravi Joshi
* Date: 2019/02/01
*/

#pragma once

// ROS headers
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

// CV brigge header
#include <cv_bridge/cv_bridge.h>

// OpenCV header
#include <opencv2/core/core.hpp>

// c++ headers
#include <mutex>
#include <vector>

namespace ros_openpose
{
  class CameraReader
  {
  private:
    cv::Mat mColorImage, mDepthImage;
    cv::Mat mColorImageUsed, mDepthImageUsed;
    std::string mColorTopic, mDepthTopic, mCamInfoTopic;
    ros::NodeHandle mNh;
    ros::Subscriber mCamInfoSubscriber;
    std::mutex mMutex;

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
    ~CameraReader() = default;

    // lock color frame. remember that we
    // are just passing the pointer instead of copying whole data
    // the function returns a color image from camera
    const cv::Mat& getColorFrame()
    {
      mMutex.lock();
      mColorImageUsed = mColorImage;
      mMutex.unlock();
      return mColorImageUsed;
    }

    // get the depth image from camera
    const cv::Mat& getDepthFrame()
    {
      return mDepthImage;
    }

    // copy the latest depth image from camera. remember that we
    // are just passing the pointer instead of copying whole data
    void copyLatestDepthImage()
    {
      mMutex.lock();
      mDepthImageUsed = mDepthImage;
      mMutex.unlock();
    }

    // compute the point in 3D space for a given pixel without considering distortion
    void compute3DPoint(const float pixel_x, const float pixel_y, float (&point)[3])
    {
      /*
       * K.at(0) = intrinsic.fx
       * K.at(4) = intrinsic.fy
       * K.at(2) = intrinsic.ppx
       * K.at(5) = intrinsic.ppy
       */

      // our depth image type is 16UC1 which has unsigned short as an underlying type
      auto depth = mDepthImageUsed.at<unsigned short>(static_cast<int>(pixel_y), static_cast<int>(pixel_x));
      if (depth <= 0)
        return;

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
