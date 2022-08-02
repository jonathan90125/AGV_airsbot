#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)

{ 

     ros::init(argc, argv, "square_publisher");
     ros::NodeHandle n; 
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    geometry_msgs::Twist msg[3];
    msg[0].linear.x =0;
    msg[0].linear.y = 0;
    msg[0].linear.z = 0;
    msg[0].angular.x = 0;
    msg[0].angular.y = 0;
    msg[0].angular.z = -0.715;

    msg[1].linear.x =0.3;
    msg[1].linear.y = 0;
    msg[1].linear.z = 0;
    msg[1].angular.x = 0;
    msg[1].angular.y = 0;
    msg[1].angular.z =0;

   

    while (true)
    {
        pub.publish(msg[0]);
        sleep(2);
        pub.publish(msg[1]);
        sleep(5);

    }

    return 0;
}
