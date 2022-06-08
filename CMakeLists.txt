cmake_minimum_required(VERSION 2.8.3)
project(ros_openpose)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport
  message_generation
  message_filters
)

## Make sure 'FindGFlags.cmake' and 'FindGlog.cmake' are visible to cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

## Add OpenPose and other dependencies
find_package(GFlags)
find_package(Glog)
find_package(OpenCV REQUIRED)
find_package(OpenPose REQUIRED)
find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Pass in the version of OpenPose
add_definitions( -DOpenPose_VERSION_MAJOR=${OpenPose_VERSION_MAJOR} )
add_definitions( -DOpenPose_VERSION_MINOR=${OpenPose_VERSION_MINOR} )
add_definitions( -DOpenPose_VERSION_PATCH=${OpenPose_VERSION_PATCH} )

## Print OpenPose version in the terminal
MESSAGE(STATUS "OpenPose VERSION: " ${OpenPose_VERSION})

## Declare ROS messages
add_message_files(
  FILES
  Pixel.msg
  BodyPart.msg
  Person.msg
  Frame.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  message_runtime
  sensor_msgs
  message_filters
)

## Build
# Specify additional locations of header files
include_directories(
  include
  ${OpenPose_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

## Path to Caffe lib i.e., libcaffe.so
## find /home/ravi/tools/openpose -name libcaffe.so
#set(CAFFE_LIB_FOLDER /home/ravi/tools/openpose/build/caffe/lib)
#link_directories(${CAFFE_LIB_FOLDER})

# Declare a C++ executable and
# specify libraries to link a executable target against
add_executable(rosOpenposeAsync
  src/rosOpenposeAsync.cpp
  src/cameraReader.cpp)

add_executable(rosOpenposeSync
  src/rosOpenposeSync.cpp)

# Bug fix
# src: https://github.com/ravijo/ros_openpose/issues/14#issuecomment-631164300
add_dependencies(rosOpenposeAsync ros_openpose_generate_messages_cpp)
add_dependencies(rosOpenposeSync ros_openpose_generate_messages_cpp)

target_link_libraries(rosOpenposeAsync
  ${OpenPose_LIBS}
  gflags ${GFLAGS_LIBRARY}
  ${GLOG_LIBRARY}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

target_link_libraries(rosOpenposeSync
        ${OpenPose_LIBS}
        gflags ${GFLAGS_LIBRARY}
        ${GLOG_LIBRARY}
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        )

add_executable(testCameraReader
  src/testCameraReader.cpp
  src/cameraReader.cpp)

target_link_libraries(testCameraReader
  ${OpenPose_LIBS}
  ${GFLAGS_LIBRARY}
  ${GLOG_LIBRARY}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
