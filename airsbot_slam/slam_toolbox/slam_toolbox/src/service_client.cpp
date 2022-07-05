#include <cstdlib>
#include "ros/ros.h"
#include "slam_toolbox/toolbox_msgs.hpp"

ros::ServiceClient _clearChanges, _saveChanges, _saveMap, _clearQueue, _interactive, _pause_measurements, _load_submap_for_merging, _merge, _serialize, _load_map;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "client");
  ros::NodeHandle nh;
  bool paused_measure = false, interactive = false;
  nh.getParam("/slam_toolbox/paused_new_measurements", paused_measure);
  nh.getParam("/slam_toolbox/interactive_mode", interactive);
  _serialize = nh.serviceClient<slam_toolbox_msgs::SerializePoseGraph>("/slam_toolbox/serialize_map");
  _saveMap = nh.serviceClient<slam_toolbox_msgs::SaveMap>("/slam_toolbox/save_map");

//save map 
  slam_toolbox_msgs::SaveMap msg;
  msg.request.name.data = "name";
  if (!_saveMap.call(msg))
  {
    ROS_WARN("SlamToolbox: Failed to save map as %s, is service running?",
	      msg.request.name.data.c_str());
  }

//serialize map
  slam_toolbox_msgs::SerializePoseGraph msg2;
  msg2.request.filename = "name";
  if (!_serialize.call(msg2))
  {
    ROS_WARN("SlamToolbox: Failed to serialize pose graph to file, is service running?");
  }

}
 
