cmake_minimum_required(VERSION 3.0.2)
project(set_init_pose)

set(PACKAGE_DEPENDENCIES
        cartographer_ros
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES}
  roscpp
  rospy
  std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        ${PACKAGE_DEPENDENCIES}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(set_init_pose src/set_init_pose.cpp)
target_link_libraries(set_init_pose ${catkin_LIBRARIES})




