cmake_minimum_required(VERSION 3.0.2)
project(cmdvel_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)
 
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
 

add_executable(cmdvel_publisher src/cmdvel_publisher.cpp)
target_link_libraries(cmdvel_publisher ${catkin_LIBRARIES})













