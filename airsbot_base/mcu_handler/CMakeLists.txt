cmake_minimum_required(VERSION 2.8.3)
project(mcu_handler)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
 
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  std_msgs
)
 


catkin_package(
  INCLUDE_DIRS include
 
)

###########
## Build ##
###########
 
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
 
)
 


add_executable(publish_node1 src/publish_node1.cpp 
			    src/mbot_linux_serial.cpp
                            )
target_link_libraries(publish_node1 ${catkin_LIBRARIES}   )

