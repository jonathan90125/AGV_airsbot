#include "ros/ros.h"
#include "std_msgs/String.h" //use data struct of std_msgs/String 
#include <sensor_msgs/Imu.h> 
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Pose2D.h> 
#include <mbot_linux_serial.h>
#include "std_msgs/Int8.h" 
 
//test send value
double testSend1=-20.0;
double testSend2=-20.0;
unsigned char testSend3=0x01;
int count = 0;
//test receive value
double testRece1=0.0;
double testRece2=0.0;
double testRece3=0.0;

double rece_imu1=0.0;
double rece_imu2=0.0;
double rece_imu3=0.0;
double rece_imu4=0.0;
double rece_imu5=0.0;
double rece_imu6=0.0;
double rece_imu7=0.0;
double rece_imu8=0.0;
double rece_imu9=0.0;
double rece_imu10=0.0;
double v_l=0;
double v_r=0;

double qx;
double qy;
double qz;
double qw;

double vel_x;
double vel_th;

struct odometry{
    float x;
    float y;
    float yaw;}odom;

unsigned char testRece4=0x00;
void callback(const geometry_msgs::Twist& cmd_vel);
void m_callback(const std_msgs::Int8& msg);
void odom_cal_trigger(int x, int y,int z);

int main(int agrc,char **argv)
{
    ros::init(agrc,argv,"public_node");
    ros::NodeHandle n;
    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("IMU_data", 20); 
    ros::Publisher odom_pub = n.advertise<geometry_msgs::Pose>("odom_data", 20); 
    ros::Publisher tf_pub = n.advertise<geometry_msgs::Pose2D>("tf_data", 20);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("vel_data", 20);
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);
    ros::Subscriber mode_sub = n.subscribe("motor_mode", 1000, m_callback);
    ros::Rate loop_rate(50);	//设置发送数据的频率为50Hz

 
    //串口初始化
    serialInit();
    
    while(ros::ok())
    {
		ros::spinOnce();
		//   writeSpeed(testSend1, testSend2,testSend3);	

		readSpeed(testRece1,testRece2,rece_imu1,rece_imu2,rece_imu3,rece_imu4,rece_imu5,rece_imu6,rece_imu7,rece_imu8,rece_imu9,rece_imu10,testRece4);
		ROS_INFO("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",testRece1,testRece2,rece_imu1,rece_imu2,rece_imu3,rece_imu4,rece_imu5,rece_imu6,rece_imu7,rece_imu8,rece_imu9,rece_imu10,testRece4);


		sensor_msgs::Imu imu_data;
		imu_data.header.stamp = ros::Time::now();
		imu_data.header.frame_id = "base_link";
		//四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
		imu_data.orientation.x = rece_imu1;
		imu_data.orientation.y = rece_imu2;
		imu_data.orientation.z = rece_imu3;
		imu_data.orientation.w = rece_imu4;
		//线加速度
		imu_data.linear_acceleration.x = rece_imu5; 
		imu_data.linear_acceleration.y = rece_imu6;
		imu_data.linear_acceleration.z = rece_imu7;
		//角速度
		imu_data.angular_velocity.x = rece_imu8; 
		imu_data.angular_velocity.y = rece_imu9; 
		imu_data.angular_velocity.z = rece_imu10;
		IMU_pub.publish(imu_data);
		//odom_cal_trigger(testRece1,testRece2,rece_imu1);
                
		odom.x=testRece1/100;
		odom.y=testRece2/100;
		odom.yaw=rece_imu1/100;
		if(count<100){  
		count++;  
		if(count==100){
		ROS_INFO("x,y,yaw:[%f,%f,%f]", odom.x,odom.y,odom.yaw);
		count=0;}
		}
		
		 

		geometry_msgs::Pose pose;
                geometry_msgs::Pose2D pose2d;
                geometry_msgs::Twist twist;
		pose.position.x=odom.x;
		pose.position.y=odom.y;
		pose.position.z=0;	 
                qx=0;
		qy=0;
		qz=sin(odom.yaw/2);
		qw=cos(odom.yaw/2);
                pose.orientation.x=qx;
                pose.orientation.y=qy;
                pose.orientation.z=qz;
                pose.orientation.w=qw;
                odom_pub.publish(pose);
 
		pose2d.x=odom.x;
		pose2d.y=odom.y;
		pose2d.theta=odom.yaw;
		tf_pub.publish(pose2d);

                twist.linear.x=vel_x;
                twist.angular.z=vel_th;
                vel_pub.publish(twist);
               // ROS_INFO("linear.x,angular.z:[%f,%f]",vel_x,vel_th);

		writeSpeed(v_l,v_r,testSend3);
		//ROS_INFO("l_rpm,r_rpm:[%f,%f]",v_l,v_r);
			 
		loop_rate.sleep();
    }
   
    return 0;
}





void callback(const geometry_msgs::Twist& cmd_vel)
{
	//ROS_INFO("Received a /cmd_vel message!");
	//ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
	//ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);
	 
	double v = cmd_vel.linear.x; 
	double w = cmd_vel.angular.z;
     
	v_l = 0;
	v_r = 0;
	double D = 0.4 ;
	double d = D/2;

	v_l = (v + w*d)/0.00732;
	v_r = (v - w*d)/0.00732;
}

void m_callback(const std_msgs::Int8& msg){
	testSend3=msg.data;
}
 
void odom_cal_trigger(int x, int y,int z)
{
    
	// float dl = 0.0, dr = 0.0;
	//float sinval = 0.0, cosval = 0.0;
	//dl = left *43.96 *0.0225/60; // cm
	// dr = right *43.96*0.0225/60; // cm
	//float d_yaw = (dl - dr) / 2.0f /18.5; //rad
	//float displacement = (dl + dr) / 2.0f;

	// float dx = cos(d_yaw)*displacement; //mm
	// float dy = sin(d_yaw)*displacement; //mm

	// sinval = sin(odom.yaw), cosval = cos(odom.yaw);
	//odom.x += (cosval * dx - sinval * dy)/100.f; //m
	// odom.y += (sinval * dx + cosval * dy)/100.f; //m
	// odom.yaw += d_yaw;  //rad
	//vel_x=displacement*0.5;
	//vel_th=d_yaw*0.5;
	odom.x=x/100;
	odom.x=y/100;
	odom.yaw=z/100;
	if(count<100){  
	count++;  
	if(count==100){
	//ROS_INFO("x,y,yaw:[%f,%f,%f]", x,y,z);
	count=0;}
	}
}
 

 

