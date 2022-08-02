#include <cstdlib>
#include "ros/ros.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "geometry_msgs/Pose.h"



ros::ServiceClient start_tra,finish_tra;
void init_callback(const geometry_msgs::Pose& msg);
 geometry_msgs::Pose pose;
 int pose_num=1;
 bool pose_flag=true;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "set_initial_pose");
  ros::NodeHandle nh;
  bool paused_measure = false, interactive = false;
  //two types of services
  start_tra = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
  finish_tra = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("/finish_trajectory");
  ros::Subscriber init_pose_sub = nh.subscribe("/init_pose", 1000,init_callback);

  //define two types of msgs
  cartographer_ros_msgs::StartTrajectory start_msg;
  cartographer_ros_msgs::FinishTrajectory finish_msg;
 ros::Rate loop_rate(1);
    while (ros::ok())
    {    
        if(pose_flag){
          //set msgs
            finish_msg.request.trajectory_id=pose_num;
          
            start_msg.request.configuration_directory = "/home/lxz/cat_ws/src/cartographer_ros/cartographer_ros/configuration_files";
            start_msg.request.configuration_basename ="backpack_2d_localization.lua";
            start_msg.request.use_initial_pose = true;
            start_msg.request.initial_pose.position.x=1.0;
            start_msg.request.initial_pose.position.y=0.0;
            start_msg.request.initial_pose.position.z=0.0;
            start_msg.request.initial_pose.orientation.x=0.0;
            start_msg.request.initial_pose.orientation.y=0.0;
            start_msg.request.initial_pose.orientation.z=0.0;
            start_msg.request.initial_pose.orientation.w=1.0;
            start_msg.request.relative_to_trajectory_id = 0;
          
            //send msgs 
            if (!finish_tra.call(finish_msg))
            {
              ROS_WARN("Failed to finish trajectory!!!!!!!!!");
            }

            if (!start_tra.call(start_msg))
            {
              ROS_WARN("Failed to start trajectory!!!!!!!!!");
            }
            pose_flag=false;
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    }
}
 

void init_callback(const geometry_msgs::Pose& msg){

    pose.position.x=msg.position.x;
		pose.position.y=msg.position.y;
		pose.position.z=msg.position.z;	 

    pose.orientation.x=msg.orientation.x;
    pose.orientation.y=msg.orientation.y;
    pose.orientation.z=msg.orientation.z;
    pose.orientation.w=msg.orientation.w;
    pose_num+=1;
    pose_flag=true;
}