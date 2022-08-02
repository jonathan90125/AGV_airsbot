#include "ros/ros.h"
#include "std_msgs/Int8.h" //use data struct of std_msgs/String 
#include <sensor_msgs/Imu.h> 
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Pose2D.h> 
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <sys/socket.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
modbus_mapping_t *mb_mapping;

int mission_num,mission_type,mission_priority,mission_state;
int goal_pose_x,goal_pose_y,goal_pose_yaw;
void change_bit_to_zero(int bit_index);
void change_bit_to_one(int bit_index);
int pow_1(int num2);
uint8_t get_bit(int index);
std_msgs::Int8 msg;

uint16_t UT_BITS_ADDRESS = 0x100;
uint16_t UT_BITS_NB = 0x28;
uint8_t UT_BITS_TAB[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
//线圈寄存器bit定义
/****************************
1	暂停运动
2	继续运动
3	停止运动
4	停止定位
5	出发急停
6	解除急停
7	手动控制
8	坐标点导航
9	站点导航
10	松轴推动控制
11	向前移动100ms
12	向后移动100ms
13	向左旋转100ms
14	向右旋转100ms
15	更改位姿
	
****************************/


uint16_t UT_INPUT_BITS_ADDRESS = 0x200;
uint16_t UT_INPUT_BITS_NB = 0x28;
uint8_t UT_INPUT_BITS_TAB[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };

uint16_t UT_REGISTERS_ADDRESS = 0x300;
uint16_t UT_REGISTERS_NB = 0xD;
const uint16_t UT_REGISTERS_NB_MAX = 0x400;

/***************************************************************************
uint16_t UT_REGISTERS_TAB[] = { 0x0000,0x0002,0x0000,0x0000,                 //flags
                                      0x0012,0x0000,0x0000,0x0000,0x0000,0x0000, //twist
                                      0x0001,0x0000,0x0000,                     //goal pose 
                                     };

uint16_t UT_INPUT_REGISTERS_ADDRESS = 0x400;
uint16_t UT_INPUT_REGISTERS_NB = 0x12;
uint16_t UT_INPUT_REGISTERS_TAB[] = { 0x0000,0x0000,0x0001,0x0000,0x0000,0x0001, //flags
                                      0x0032,0x0000,0x0000,0x0000,0x0000,0x0000, //twist
                                      0x0010,0x00010,0x0000,                     //current pose
                                      0x0000,0x0000,0x0000,                     //goal pose
                                      };
*********************************************************************************************/

uint16_t UT_REGISTERS_TAB[] = {
				0x0000, //1 通过位姿定位x
				0x0000, //2 通过位姿定位y
				0x0000, //3 通过位姿定位yaw
				0x0000, //4 通过站点定位
				0x0000, //5 运动状态
				0x0000, //6 自动导航移动到位姿x
				0x0000, //7 自动导航移动到位姿y
				0x0000, //8 自动导航移动到位姿yaw
				0x0000, //9 自动导航移动到站点
				0x0000, //10 设置避障策略
				0x0000, //11 执行动作任务id 
				0x0000, //12 执行动作任务参数0
				0x0000, //13 执行动作任务参数1 
				0x0000, //14 手动控制车辆以Vx线速度运动 
				0x0000, //15 手动控制车辆以Vy线速度运动  
				0x0000, //16 手动控制车辆以w角速度运动  
				0x0000, //17 设置速度级别
				0x0000, //18 设置当前站点 
				0x0000, //19 设置当前地图 
				0x0000, //20 通过位姿强制定位x 
				0x0000, //21 通过位姿强制定位y  
				0x0000, //22 通过位姿强制定位yaw  
				0x0000, //23 自主导航移动到位姿x 
				0x0000, //24 自主导航移动到位姿y 
				0x0000, //25 自主导航移动到位姿yaw
				0x0000, //26 自主导航移动到站点编号
				0x0000, //27 自主导航移动到站点 
				0x0000, //28 执行动作任务编号 
				0x0000, //29 执行动作ID 
				0x0000, //30 执行动作任务参数0 
				0x0000, //31 执行动作任务参数1		 
                                
};

uint16_t UT_INPUT_REGISTERS_ADDRESS = 0x400;
uint16_t UT_INPUT_REGISTERS_NB = 0x12;
uint16_t UT_INPUT_REGISTERS_TAB[] = {   
					0x0000, //1 系统状态
					0x0000, //2 定位姿态
					0x0000, //3 位姿x
					0x0000, //4 位姿y
					0x0000, //5 位姿yaw
					0x0000, //6 位姿置信度
					0x0000, //7 运动状态
					0x0000, //8 导航任务状态
					0x0000, //9 动作任务状态
					0x0000, //10 动作任务结果值
					0x0000, //11 当前站点编号
					0x0000, //12 操作状态 
					0x0000, //13 x方向线速度
					0x0000, //14 y方向线速度
					0x0000, //15 角速度                     
					0x0000, //16 DI状态
					0x0000, //17 DO状态
					0x0000, //18 硬件错误码  
					0x0000, //19 系统上一次错误
					0x0000, //20 电池电压
					0x0000, //21 电池电流
					0x0000, //22 电池温度
					0x0000, //23 电池预计使用时间
					0x0000, //24 当前电量百分比 
					0x0000, //25 当前电池状态
					0x0000, //26 电池循环次数
					0x0000, //27 电池表称容量
					0x0000, //28 运动总里程计
					0x0000, //29 开机总时间
					0x0000, //30 开机总次数
					0x0000, //31 系统当前时间
					0x0000, //32 IP1
					0x0000, //33 IP2
					0x0000, //34 IP3
					0x0000, //35 IP4 
					0x0000, //36 系统版本号
					0x0000, //37 二维码ID
					0x0000, //38 二维码x 
					0x0000, //39 二维码y 
					0x0000, //40 二维码yaw
					0x0000, //41 当前地图名的前两个字节编码 
					0x0000, //42 当前移动任务编号 
					0x0000, //43 当前动作人物编号
					0x0000, //44 硬件错误码1
					0x0000, //45 硬件错误码2 
					0x0000, //46 硬件错误码3                    
};
 
  
int s = -1;
modbus_t *ctx;
 
int rc;
int i;
int use_backend;
uint8_t *query;
int header_length;
uint8_t BIT;
double linear_x,angular_z;
double pose_x,pose_z,pose_a;

void init_modbustcp()
{
	ctx = modbus_new_tcp("10.60.77.92", 1502);
	query = (uint8_t*)malloc(MODBUS_TCP_MAX_ADU_LENGTH);
      
    header_length = modbus_get_header_length(ctx);

    modbus_set_debug(ctx, TRUE);

    mb_mapping = modbus_mapping_new_start_address(
        UT_BITS_ADDRESS, UT_BITS_NB,
        UT_INPUT_BITS_ADDRESS, UT_INPUT_BITS_NB,
        UT_REGISTERS_ADDRESS, UT_REGISTERS_NB_MAX,
        UT_INPUT_REGISTERS_ADDRESS, UT_INPUT_REGISTERS_NB);

    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        modbus_free(ctx);
 
    }

   //设置INPUT寄存器
    modbus_set_bits_from_bytes(mb_mapping->tab_input_bits, 0, UT_INPUT_BITS_NB,
                               UT_INPUT_BITS_TAB);  

    modbus_set_bits_from_bytes(mb_mapping->tab_bits, 0, UT_BITS_NB,
                               UT_BITS_TAB);  


    for (i=0; i < UT_INPUT_REGISTERS_NB; i++) {
        mb_mapping->tab_input_registers[i] = UT_INPUT_REGISTERS_TAB[i];;
    }   

 	s = modbus_tcp_listen(ctx, 1);
        modbus_tcp_accept(ctx, &s);
    
}

void run_modbustcp(){
        int k;
	//updating the input register
         for (i=0; i < UT_INPUT_REGISTERS_NB; i++) {
        mb_mapping->tab_input_registers[i] = UT_INPUT_REGISTERS_TAB[i];;
        }   

        if(rc==-1){
        modbus_tcp_accept(ctx, &s);}
        for(k=0;k<1;k++){

		do {
		    rc = modbus_receive(ctx, query);    
		} while (rc == 0);
	 
		if (rc == -1 && errno != EMBBADCRC) {
		    break;
		}
	 
		rc = modbus_reply(ctx, query, rc, mb_mapping);
		 
		if (rc == -1) {
		    break;
		}
	 
	}
      
}

void callback(const geometry_msgs::Pose2D& pose)
{
	//ROS_INFO("Received a pose message!");
 
	double x = pose.x; 
	double y = pose.y;
	double a = pose.theta;
	UT_INPUT_REGISTERS_TAB[12] = x;
        UT_INPUT_REGISTERS_TAB[13] = y;
        UT_INPUT_REGISTERS_TAB[14] = a;
 
	//ROS_INFO("x,y,a:[%f,%f,%f]",x,y,a);
 

}
   
  
void stop_modbustcp(){  
    if (s != -1) {
        close(s);
    }
    modbus_mapping_free(mb_mapping);
    free(query);
    modbus_close(ctx);
    modbus_free(ctx);
}


int main(int agrc,char **argv)
{
    ros::init(agrc,argv,"tcp_node");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 20); 
    ros::Publisher mode_pub = n.advertise<std_msgs::Int8>("motor_mode", 20); 
	ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose2D>("pose_data", 20); 
	ros::Publisher station_pub = n.advertise<std_msgs::Int8>("sta_num", 20); 
    ros::Subscriber odom_sub = n.subscribe("tf_data", 1000, callback);
    ros::Rate loop_rate(10);	//设置发送数据的频率为10Hz
   
    geometry_msgs::Twist cmd;  cmd.linear.x=0;
    cmd.linear.y=0;
    cmd.linear.z=0;
    cmd.angular.x=0;
    cmd.angular.y=0;
    cmd.angular.z=0; 
    init_modbustcp();
    

    while(ros::ok())
    { 
         
        ros::spinOnce();
        run_modbustcp();
		 
        UT_INPUT_REGISTERS_TAB[0] =0;
     
    
        //mode 1
        if(get_bit(7)==1&&get_bit(8)==0&&get_bit(9)==0&&get_bit(10)==0)
	{
	 
		if(get_bit(11)==1){
			cmd.linear.x=0.1;
			vel_pub.publish(cmd); 
		}
		if(get_bit(12)==1){
			cmd.linear.x=-0.1;
			vel_pub.publish(cmd); 
		}
		if(get_bit(13)==1){
			cmd.angular.z=0.1;
			vel_pub.publish(cmd);		
		}
		if(get_bit(14)==1){
			cmd.angular.z=-0.1;
			vel_pub.publish(cmd); 
		}
		if(get_bit(11)==0&&get_bit(12)==0&&get_bit(13)==0&&get_bit(14)==0){
			    cmd.linear.x=0;
				cmd.linear.y=0;
				cmd.linear.z=0;
				cmd.angular.x=0;
				cmd.angular.y=0;
				cmd.angular.z=0;
				vel_pub.publish(cmd); 
		}
		
	}
		else{}
	//mode 2
        if(get_bit(8)==1&&get_bit(7)==0&&get_bit(9)==0&&get_bit(10)==0){
                 geometry_msgs::Pose2D pos_goal;
 
                if(mb_mapping->tab_registers[5]<0x7fff){
		pos_goal.x = mb_mapping->tab_registers[5]/100;
                }
		else{
		pos_goal.x = (mb_mapping->tab_registers[5]-0x10000)/100;
		}

                if(mb_mapping->tab_registers[6]<0x7fff){
		pos_goal.y = mb_mapping->tab_registers[6]/100;
                }
		else{
		pos_goal.y = (mb_mapping->tab_registers[6]-0x10000)/100;
		}
	
		 
                if(mb_mapping->tab_registers[7]<0x7fff){
		pos_goal.theta= mb_mapping->tab_registers[7]/100;
                }
		else{
		pos_goal.theta= (mb_mapping->tab_registers[7]-0x10000)/100;
		}
        pose_pub.publish(pos_goal);
		 
	}

	//mode 3
	if(get_bit(9)==1&&get_bit(7)==0&&get_bit(10)==0&&get_bit(8)==0){
		std_msgs::Int8 station_num;
		station_num.data=mb_mapping->tab_registers[8]-1;
		station_pub.publish(station_num);
	}
	//mode 4
	if(get_bit(10)==1&&get_bit(7)==0&&get_bit(9)==0&&get_bit(8)==0){
		msg.data=2; 
		mode_pub.publish(msg);
	}
	else
	{
		msg.data=1; 
		mode_pub.publish(msg);
	}

	loop_rate.sleep();      
    }
    stop_modbustcp();
    return 0;
}

void change_bit_to_zero(int bit_index) //bit index start from 1
{  
	int m,n,l;
	l=modbus_get_byte_from_bits(mb_mapping->tab_input_bits, bit_index-1, 1);
        if(l==1){	 
	m=(bit_index)/8;
	n=(bit_index)%8;	 
        UT_INPUT_BITS_TAB[m]= UT_INPUT_BITS_TAB[m]-pow_1(n-1);
	modbus_set_bits_from_byte(mb_mapping->tab_input_bits, m*8, UT_INPUT_BITS_TAB[m]);
	}
}

void change_bit_to_one(int bit_index){ 
	int m,n,l;
	l=modbus_get_byte_from_bits(mb_mapping->tab_input_bits, bit_index-1, 1);
        if(l==0){	 
	m=(bit_index)/8;
	n=(bit_index)%8;	 
        UT_INPUT_BITS_TAB[m]= UT_INPUT_BITS_TAB[m]+pow_1(n-1);
	modbus_set_bits_from_byte(mb_mapping->tab_input_bits, m*8, UT_INPUT_BITS_TAB[m]);
	}
}

int pow_1(int num2){
if(num2==0){
return 1;}

int num1=1;
while(num2>0){
num1*=2;
num2--;
}
return num1;}

uint8_t get_bit(int index){
 uint8_t tmp_bit;
 tmp_bit=modbus_get_byte_from_bits(mb_mapping->tab_bits, index-1, 1); //编号减去1才是实际索引
 return tmp_bit;
}
 
 
