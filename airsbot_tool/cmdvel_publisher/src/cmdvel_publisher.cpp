#include "ros/ros.h"

#include "std_msgs/String.h"

#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)

{ 

     ros::init(argc, argv, "publisher");

     ros::NodeHandle n; 

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Rate loop_rate(0.1);

    geometry_msgs::Twist msg;
    msg.angular.z = 0.314;

    while (ros::ok())

    {

        msg.linear.x =0;

        msg.linear.y = 0;

        msg.linear.z = 0;

        msg.angular.x = 0;

        msg.angular.y = 0;

        msg.angular.z = -msg.angular.z;

        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}
