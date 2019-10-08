/**
* cameraReader.cpp: class file for CameraReader. the CameraReader file provides
*                   functionality for retrieving color and depth images from the
*                   depth sensor. it also reads the camera calibration parameters
*                   via ROS subscriber.
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

  // CameraReader::~CameraReader()
  //{
  //  std::cout << "[" << this << "] destructor called" << std::endl;
  //}

  // we define the subscriber here. we are using TimeSynchronizer filter to receive the synchronized data
  inline void CameraReader::subscribe()
  {
    mSPtrColorImageSub = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(mNh, mColorTopic, 1);
    mSPtrDepthImageSub = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(mNh, mDepthTopic, 1);

    int queueSize = 4;

    // clang-format off
    mSPtrSyncSubscriber = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> >(
        *mSPtrColorImageSub,
        *mSPtrDepthImageSub,
        queueSize);
    // clang-format on

    // define which function should be called when the data is available
    mSPtrSyncSubscriber->registerCallback(&CameraReader::imageCallback, this);

    // create a subscriber to read the camera parameters from the ROS
    mCamInfoSubscriber = mNh.subscribe(mCamInfoTopic, 1, &CameraReader::camInfoCallback, this);
  }

  void CameraReader::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& camMsg)
  {
    mSPtrCameraInfo = std::make_shared<sensor_msgs::CameraInfo>(*camMsg);
    // since the calibration parameters are static so we don't need to keep running
    // the subscriber. that is why, we stop the subscriber once we receive
    // the parameters successfully
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
      // since we don't want to change the data, therefore we need not copy the image, we can just share it.
      // auto colorPtr = cv_bridge::toCvShare(colorMsg, sensor_msgs::image_encodings::BGR8);

      auto colorPtr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);
      auto depthPtr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_16UC1);

      // it is very important to lock the below assignment operation.
      // remember that we are using these variables from another thread too.
      std::lock_guard<std::mutex> lock(mMutex);
      mColorImage = colorPtr->image;
      mDepthImage = depthPtr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      // display the error at most once per 10 seconds
      ROS_ERROR_THROTTLE(10, "cv_bridge exception: %s", e.what());
    }
  }
}
