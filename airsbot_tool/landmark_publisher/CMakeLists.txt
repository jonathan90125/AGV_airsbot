cmake_minimum_required(VERSION 3.0.2)
project(landmark_publisher)

set(PACKAGE_DEPENDENCIES
        cartographer_ros
)
 
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES}
  geometry_msgs
  message_runtime
  nav_msgs
  pcl_conversions
  rosbag
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
)
 
catkin_package(
        CATKIN_DEPENDS
        ${PACKAGE_DEPENDENCIES}
)
 
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(landmark_publisher src/landmark_publisher.cpp)
target_link_libraries(landmark_publisher ${catkin_LIBRARIES})



 
