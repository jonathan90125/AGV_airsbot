cd /dev
sudo chmod 777 ttyUSB0

{
gnome-terminal -t "1" -x bash -c "roslaunch airsbot_navigation cartographer_configuration.launch;exec bash"
}&

sleep 1s
{
gnome-terminal -t "2" -x bash -c "roslaunch airsbot_navigation cartographer_move_base.launch;exec bash"
}&

sleep 1s
{
gnome-terminal -t "3" -x bash -c "roslaunch airsbot_navigation send_goal.launch;exec bash"
}&

sleep 1s
{
gnome-terminal -t "4" -x bash -c "rosrun station_publisher station_publisher;exec bash"
} 
