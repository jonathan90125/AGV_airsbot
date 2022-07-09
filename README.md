# Airsbot

This is the repo of airsbot which is an AGV of two versions, which are the **indoor** version and the **outdoor** version. Two robots could be equipped with various devices to perform different tasks. When I was in charge of developing the robots, the indoor robot is equipped with a sprayer to disinfect to protect us from COVID19 during this specific period, and the outdoor version is used in construction site to carry material for workers.

<div align=center><img src=".\airsbot_doc\1.JPG" alt="1" height="300" />   <img src=".\airsbot_doc\2.JPG" alt="2" height="300" /> </div>          

The major difference of two robots is their firmware. They both use in-wheel motors, but the communication rules for each motor are different according to their manufacturer.  Moreover, one needs IMU data while the other needs ultrasonic data. Their LADAR and controlling joystick are also different. In general, the firmware need two types code for STM32 chips while the host need only one version of code based on ROS. 

## Host Computer

### 0. Architecture
#### 0.1. System Architecture
![](./airsbot_doc/system_architecture.png)

(some details and devices have changed).

#### 0.2. Software Architecture

![](./airsbot_doc/software_architecture.png)

### 1. Installation

**Airsbot** has tested on machines with the following configurations  
* Ubuntu 18.04.5 LTS + ROS melodic

#### 1.1. Ros melodic installation
Either check official installation tutorial [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu) or follow the command lines below

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt install curl # if you haven't already installed curl
    $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    $ sudo apt update
    $ sudo apt install ros-melodic-desktop-full
    $ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    $ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    $ sudo apt install python-rosdep
    $ sudo rosdep init
    $ rosdep update

#### 1.2. Clone airsbot project

Clone **airsbot** into `src` folder. It is noted that `DIR_AIRSBOT` is the target directory of airsbot project. `DIR_AIRSBOT` could be either catkin workspace or other directories. 

    $ mkdir -p {DIR_AIRSBOT}/src 
    $ cd {DIR_AIRSBOT}/src
    $ git clone https://github.com/jonathan90125/AGV_airsbot.git

#### 1.3. Dependencies Installation

    $ cd ..
    $ rosdep install -q -y -r --from-paths src --ignore-src

### 2. Compile 
Build **airsbot** by catkin tool 

    $ cd {DIR_AIRSBOT}
    $ catkin_make

And source the directory 

    $ source devel/setup.bash

### 3. Run Packages (need further verification)  
#### 3.1. Run airsbot_base
    $ source devel/setup.bash
    $ roslaunch move_base airsbot_move_base.launch
#### 3.2. Run Gmapping  
Cartographer has shown better performance. Implemented but stop maintaining.
#### 3.3. Run Google Cartographer  
    $ source devel/setup.bash
    $ roslaunch cartographer_ros airsbot_cartographer.launch
#### 3.4. Run Slam_toolbox  
Cartographer has shown better performance. Implemented but stop maintaining.
#### 3.5. Run Move_base Navigation
    $ source devel/setup.bash
    $ roslaunch move_base airsbot_move_base.launch

### 4. Firmware
The related MCU codes are in airsbot_firmware package.  
MCU model: STM32F105VCT6 
Project IDE: Keil C, version 5

#### 4.1. Communication Protocol
**TX message,** from MCU to host computer  

    Length: 19 bytes, 152 bits
    Formate: start byte1 + start byte2 + size + odom_x + odom_y + odom_yaw + imu1 + imu2 + imu3 + imu4 + imu5 + imu6 + imu7 + imu8 + imu9 + imu10 + checksum + end byte1 + end byte2
        $ start byte1: 0x55
        $ start byte2: 0xaa
        $ size: length of data, 15 byte
        $ odom_x: newly updated odometry x
        $ odom_y: newly updated odometry y
        $ odom_yaw: newly updated odometry yaw
        $ imuu[1-10]: data read from Imu 
        $ checksum: CRC-8
        $ end byte1: 0x0d
        $ end byte2: 0x0a

**RX message,** from host computer to MCU  

    Length: 9 bytes, 72 bits
    Format: start byte1 + start byte2 + size + left_vel_set + right_set_vel + control_flag + checksum + end byte1 + end byte2
        $ start byte1: 0x55
        $ start byte2: 0xaa
        $ size: length of data, 5 byte
        $ left_vel_set: rpm of left wheel
        $ right_vel_set: rpm of right wheel
        $ control_flag: set different state for robot, 1 or 2
        $ checksum:  CRC-8
        $ end byte1: 0x0d
        $ end byte2: 0x0a

## Slave Computer



