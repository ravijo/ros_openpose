/**
* cameraReader.cpp: class file for CameraReader
* Author: Ravi Joshi
* Date: 2019/02/01
*/

#include <ros_openpose/cameraReader.hpp>

namespace ros_openpose
{
  CameraReader::CameraReader(ros::NodeHandle& nh, const std::string& colorTopic, const std::string& depthTopic,
                             const std::string& camInfoTopic)
    : mNh(nh), mColorTopic(colorTopic), mDepthTopic(depthTopic), mCamInfoTopic(camInfoTopic)
  {
    // std::cout << "[" << this << "] constructor called" << std::endl;
    subscribe();
  }

  CameraReader::CameraReader(const CameraReader& other)
    : mNh(other.mNh), mColorTopic(other.mColorTopic), mDepthTopic(other.mDepthTopic), mCamInfoTopic(other.mCamInfoTopic)
  {
    // std::cout << "[" << this << "] copy constructor called" << std::endl;
    subscribe();
  }

  CameraReader& CameraReader::operator=(const CameraReader& other)
  {
    // std::cout << "[" << this << "] copy assignment called" << std::endl;
    mNh = other.mNh;
    mColorTopic = other.mColorTopic;
    mDepthTopic = other.mDepthTopic;
    mCamInfoTopic = other.mCamInfoTopic;

    subscribe();
    return *this;
  }

  CameraReader::~CameraReader()
  {
    // std::cout << "[" << this << "] destructor called" << std::endl;
  }

  inline void CameraReader::subscribe()
  {
    mSPtrColorImageSub = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(mNh, mColorTopic, 1);
    mSPtrDepthImageSub = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(mNh, mDepthTopic, 1);

    int queueSize = 4;

    // clang-format off
    mSPtrSyncSubscriber = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>>(
        *mSPtrColorImageSub,
        *mSPtrDepthImageSub,
        queueSize);
    // clang-format on

    mSPtrSyncSubscriber->registerCallback(&CameraReader::imageCallback, this);

    mCamInfoSubscriber = mNh.subscribe(mCamInfoTopic, 1, &CameraReader::camInfoCallback, this);
  }

  void CameraReader::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& camMsg)
  {
    mSPtrCameraInfo = std::make_shared<sensor_msgs::CameraInfo>(*camMsg);
    if (mSPtrCameraInfo != nullptr)
    {
      mCamInfoSubscriber.shutdown();
    }
  }

  void CameraReader::imageCallback(const sensor_msgs::ImageConstPtr& colorMsg,
                                   const sensor_msgs::ImageConstPtr& depthMsg)
  {
    try
    {
      // since we donot want to change the data, therefore we need not copy nh, we can just share nh.
      // auto cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      auto colorPtr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);
      auto depthPtr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_16UC1);

      {
        std::lock_guard<std::mutex> lock(mImageMutex);
        mColorImage = colorPtr->image;
        mDepthImage = depthPtr->image;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      // display the error at most once per 10 seconds
      ROS_ERROR_THROTTLE(10, "cv_bridge exception: %s", e.what());
    }
  }
}
