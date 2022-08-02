#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h> 
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

double x,y,th;
double vx,vth;
ros::Time current_time;
void vel_callback(const geometry_msgs::Twist& twist);

int main(int argc, char **argv)
{ 

	ros::init(argc, argv, "carto_odom");
	ros::NodeHandle n; 
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/cartographer_odom", 50);
	ros::Subscriber vel_sub = n.subscribe("vel_data", 1000, vel_callback);
	ros::Rate loop_rate(10);
	 
	 while(ros::ok()){ 

	 
	tf::StampedTransform transform;
	tf::TransformListener listener;
	try
	{
	listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
	listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException &ex)
	{
	ROS_ERROR("%s", ex.what());
	}
	//输出位置信息
	x = static_cast<double>(transform.getOrigin().x());
	y = static_cast<double>(transform.getOrigin().y());
	th = tf::getYaw(transform.getRotation());

    
	 current_time = ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

	ros::spinOnce();
	loop_rate.sleep();

	 }
		 
 
	return 0;
}

 void vel_callback(const geometry_msgs::Twist& twist){
vx=twist.linear.x;
vth=twist.angular.z;
}