cmake_minimum_required(VERSION 2.8.3)
project(modbustcp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_library(LIBMODBUS_LIBRARIES
        NAMES libmodbus.so)

if (LIBMODBUS_LIBRARIES)
    message( STATUS "Libmodbus found")
else()
    message( FATAL_ERROR "Libmodbus Not found")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  std_msgs
  actionlib
  move_base_msgs
)

catkin_package(
        # INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp std_msgs
#  LIBRARIES topic_example
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # INCLUDE_DIRS include
        ${catkin_INCLUDE_DIRS}
)

add_executable(tcp_node src/tcp_node.cpp)
target_link_libraries(tcp_node
        ${catkin_LIBRARIES}
        ${LIBMODBUS_LIBRARIES}
        )

