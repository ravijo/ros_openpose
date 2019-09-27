/**
* cameraReader.cpp: class file for CameraReader
* Author: Ravi Joshi
* Date: 2019/02/01
*/

#include <ros_openpose/cameraReader.hpp>

namespace ros_openpose
{
  CameraReader::CameraReader(image_transport::ImageTransport& it, const std::string& topicName)
    : mIt(it), mTopicName(topicName)
  {
    // std::cout << "[" << this << "] constructor called" << std::endl;
    subscribe();
  }

  CameraReader::CameraReader(const CameraReader& other) : mIt(other.mIt), mTopicName(other.mTopicName)
  {
    // std::cout << "[" << this << "] copy constructor called" << std::endl;
    subscribe();
  }

  CameraReader& CameraReader::operator=(const CameraReader& other)
  {
    // std::cout << "[" << this << "] copy assignment called" << std::endl;
    mIt = other.mIt;
    mTopicName = other.mTopicName;
    subscribe();
    return *this;
  }

  CameraReader::~CameraReader()
  {
    // std::cout << "[" << this << "] destructor called" << std::endl;
  }

  inline void CameraReader::subscribe()
  {
    mSubscriber = mIt.subscribe(mTopicName, 1, &CameraReader::imageCallback, this);
  }

  void CameraReader::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      // since we donot want to change the data, therefore we need not copy it, we can just share it.
      // auto cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      auto cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      mImage = cvPtr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      // display the error at most once per 10 seconds
      ROS_ERROR_THROTTLE(10, "cv_bridge exception: %s", e.what());
    }
  }
}
