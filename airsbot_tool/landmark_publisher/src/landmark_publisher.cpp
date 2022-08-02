#include "ros/ros.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "cartographer_ros_msgs/LandmarkEntry.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
void ar_callback(const ar_track_alvar_msgs::AlvarMarkers& marker);
int mark_num=0;
struct markers{
    int id;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;}mark1;
markers marklist[10];

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pub_landmark");
    ros::NodeHandle n("~");

    ros::Publisher landmark_pub = n.advertise< cartographer_ros_msgs::LandmarkList>("/landmark", 10);
    ros::Subscriber ar_sub = n.subscribe("/ar_pose_marker", 1000,ar_callback);

    cartographer_ros_msgs::LandmarkList landmark_0;
    int i=0,j=0;
    ros::Rate loop_rate(1);
    while (ros::ok())
    {    
        landmark_0.header.stamp = ros::Time::now();
        landmark_0.header.frame_id = "camera_link";
        landmark_0.landmarks.resize(mark_num);
        i=0;
        j=mark_num;
        while(j-i>0){
            landmark_0.landmarks[i].id =marklist[i].id;
            landmark_0.landmarks[i].tracking_from_landmark_transform.position.x =marklist[i].x;
            landmark_0.landmarks[i].tracking_from_landmark_transform.position.y =marklist[i].y;
            landmark_0.landmarks[i].tracking_from_landmark_transform.position.z =marklist[i].z;
            landmark_0.landmarks[i].tracking_from_landmark_transform.orientation.w = marklist[i].qw;
            landmark_0.landmarks[i].tracking_from_landmark_transform.orientation.x =marklist[i].qx;
            landmark_0.landmarks[i].tracking_from_landmark_transform.orientation.y = marklist[i].qy;
            landmark_0.landmarks[i].tracking_from_landmark_transform.orientation.z = marklist[i].qz;
            landmark_0.landmarks[i].translation_weight = 1000000000;
            landmark_0.landmarks[i].rotation_weight = 0;    
          
            // ROS_INFO("x:      %f", landmark_0.landmarks[i].tracking_from_landmark_transform.position.x );
            // ROS_INFO("y:      %f", landmark_0.landmarks[i].tracking_from_landmark_transform.position.y );
            // ROS_INFO("z:      %f", landmark_0.landmarks[i].tracking_from_landmark_transform.position.z );
            i+=1;
            }
        landmark_pub.publish(landmark_0);
	    ros::spinOnce();
        loop_rate.sleep();
         
    }
    
    return 0;
}



void ar_callback(const ar_track_alvar_msgs::AlvarMarkers& marker)
{

    mark_num=marker.markers.size(); 
    if(marker.markers.size() > 0 && (marker.markers[0].id < 10) ){
       for(int i=0;i< marker.markers.size();i++){ 
                marklist[i].id=marker.markers[i].id;
                marklist[i].x=marker.markers[i].pose.pose.position.x;
                marklist[i].y=marker.markers[i].pose.pose.position.y;
                marklist[i].z=marker.markers[i].pose.pose.position.z;
                marklist[i].qx=marker.markers[i].pose.pose.orientation.x;
                marklist[i].qy=marker.markers[i].pose.pose.orientation.y;
                marklist[i].qz=marker.markers[i].pose.pose.orientation.z;
                marklist[i].qw=marker.markers[i].pose.pose.orientation.w;
            }
     }
 
}
