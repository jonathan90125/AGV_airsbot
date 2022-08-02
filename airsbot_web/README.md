# ROS_WEB

## Goals:

1.Connecting.

2.Show map.

3.Station planning (Click to set Station)

4.Create yaml

## Completed:

1.Connecting

2.Publish TWIST on topics

## To be solved lately:

1.Read codes&commit the topic design.

2.How to show map in web?

3.Click events (design&HTML coding).

![web_msg](/home/sks/airsbot/airsbot_doc/web_msg.png)

---------------------------------------------------------------------------------------------------------

# HOW TO CONFIGURE ROS_WEB

## 1.Install APACHE2

`$ sudo apt-get update`

`$ sudo apt-get install apache2`

## 2.Set the .conf file

`$ sudo gedit /etc/apache2/sites-available/000-default.conf`

then you need to modify the URL behind 'DocumentRoot'

* Modify it as 'DocumentRoot {DIR_AIRSBOT}/ros_web'

`$ sudo gedit /etc/apache2/apache2.conf`

you need to modify the "/var/www" as "{DIR_AIRSBOT}/ros_web" 

* Unnecessary to modify the comments beginning with '#'.

## 3.See more oprations in detail.

 https://blog.csdn.net/mashuai720/article/details/83030647

# HOW TO SHOW THE WEB ON MOBILE DEVICES.

## 1.Install rosweb tools (Needed in future)

`$ sudo apt-get install ros-melodic-rosbridge-suite`

`$ git clone https://github.com/RobotWebTools/roslibjs.git`

`$ git clone https://github.com/RobotWebTools/ros2djs`

`$ git clone https://github.com/RobotWebTools/ros3djs`

## 2.run rosbirdge ws

`$ roslaunch rosbridge_server rosbridge_websocket.launch`

## 3.run apache2 & visit the IP

`$ systemctl restart apache2`

Then use mobile devices and go to the IP of ros

such as :10.60.77.92

* MUST BE IN THE SMAE LAN.

