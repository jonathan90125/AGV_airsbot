cmake_minimum_required(VERSION 3.0.2)
project(airsbot_navigation)
 


find_package(catkin REQUIRED COMPONENTS
  move_base
 
)
 
catkin_package(
 
)
 
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 
