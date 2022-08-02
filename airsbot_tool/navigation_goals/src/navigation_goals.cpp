#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
 
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
move_base_msgs::MoveBaseGoal goal;
int goals_index=0;
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  double goals[4][3]={
	 
	// {1.5,-1,0},
	// {1.5,0,0},
	// {1.5,1,0},
	// {0,0,0},
	// {-1.5,0,0},
	{0.5,0.5,0},
	{0.5,-0.5,0},
	{-0.5,-0.5,0},
	{-0.5,0.5,0},
  };
  while(true){ 
	
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = goals[goals_index][0];
	goal.target_pose.pose.position.y = goals[goals_index][1];	 
	goal.target_pose.pose.orientation.z =sin(goals[goals_index][2]/2);
	goal.target_pose.pose.orientation.w =cos(goals[goals_index][2]/2);
        goals_index++;
	if(goals_index==4)
	   goals_index=0;
	ROS_INFO("Sending goal");
	ROS_INFO("x,y,qz,qw: %f,%f,%f,%f \r\n",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
	ac.sendGoal(goal);
	ac.waitForResult();
	 while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
	    ROS_INFO("failed,restart!!!  \r\n");     
	}
	sleep(1);
	 
      }
 
  return 0;
}
 

  




















