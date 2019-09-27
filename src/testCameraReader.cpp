/**
* testRosCameraReader.cpp: test file for CameraReader
* Author: Ravi Joshi
* Date: 2019/02/01
*/

#include <iostream>
#include <vector>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <ros_openpose/cameraReader.hpp>

typedef std::shared_ptr<ros_openpose::CameraReader> sPtrCameraReader;

void show(std::vector<sPtrCameraReader>& readers)
{
  ros::Rate loopRate(10);
  while (ros::ok())
  {
    for (int i = 0; i < readers.size(); i++)
    {
      auto image = readers[i]->getFrame();
      if (!image.empty())
      {
        auto windowName = std::to_string(i + 1);
        cv::imshow(windowName, image);
      }
      else
      {
        // display the error at most once per 10 seconds
        ROS_WARN_THROTTLE(10, "Empty image frame detected. Ignoring...");
      }
    }

    int key = cv::waitKey(1) & 255;
    if (key == 27)  // Escape key
      break;

    ros::spinOnce();
    loopRate.sleep();
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_camera_reader");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  std::vector<sPtrCameraReader> readers;

  const int cameras = 1;
  for (size_t i = 0; i < cameras; i++)
  {
    // topicName is following the convention mentioned below:
    // /camera1/color/image_raw
    // /camera2/color/image_raw
    // /camera3/color/image_raw
    auto topicName = std::string("/camera") + std::to_string(i + 1) + std::string("/color/image_raw");
    auto reader = std::make_shared<ros_openpose::CameraReader>(it, topicName);
    readers.emplace_back(reader);
  }

  show(readers);
  return 0;
}
