#ifndef MBOT_LINUX_SERIAL_H
#define MBOT_LINUX_SERIAL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>

extern void serialInit();
extern void writeSpeed(double Left_v, double Right_v,unsigned char ctrlFlag);
bool readSpeed(double &Left_v,double &Right_v,double &imu_1,double &imu_2,double &imu_3,double &imu_4,double &imu_5,double &imu_6,double &imu_7,double &imu_8,double &imu_9,double &imu_10,unsigned char &ctrlFlag);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif
