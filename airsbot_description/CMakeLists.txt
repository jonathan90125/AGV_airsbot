cmake_minimum_required(VERSION 2.8.3)
project(airsbot_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)
 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbot_description
#  CATKIN_DEPENDS urdf xacro
#  DEPENDS system_lib
)

 
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
 
