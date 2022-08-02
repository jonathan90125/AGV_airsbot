#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"

void pose_callback(const std_msgs::Int8& msg);
void sta_callback(const std_msgs::Int8& msg);
int sta=0,pose=0;

int main(int argc, char **argv)
{ 

	ros::init(argc, argv, "station_publisher");
	ros::NodeHandle n; 
	ros::Publisher pub = n.advertise<std_msgs::Int8>("sta_num", 1000);
	ros::Subscriber sta_sub = n.subscribe("/sta_state", 1000,sta_callback);
	ros::Subscriber pose_sub = n.subscribe("/pose_state", 1000,pose_callback);
	ros::Rate loop_rate(0.2);
	 
	 int sequence[5]={1,2,3,4,5};
	int j;
	while (ros::ok())
	{
		std_msgs::Int8 goal;
		if(sta==0){
			goal.data=sequence[j];
			 ROS_INFO("Sending sta_num:   %d",goal.data);
			pub.publish(goal);
			j++;
			if(j==4){
				j=0;
			}
		}
			 
		ros::spinOnce();
		loop_rate.sleep();
		}
		 
 
	return 0;
}


void sta_callback(const std_msgs::Int8& msg){
	sta=msg.data;
	ROS_INFO(" receive station msg:      %d ",sta);
}

void pose_callback(const std_msgs::Int8& msg){
	ROS_INFO(" receive pose msg ");
	pose=msg.data;
	 
}