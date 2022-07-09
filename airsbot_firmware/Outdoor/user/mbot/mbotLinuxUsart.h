#ifndef __MBOTLINUXUSART__
#define __MBOTLINUXUSART__
#include <sys.h>	

#define START   0X11

//从linux接收并解析数据到参数地址中
extern int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag);   
//封装数据，调用USART1_Send_String将数据发送给linux
void usartSendData(short ox0, short oy0,short oa0,short imu_1,short imu_2,short imu_3,short imu_4,short imu_5,short imu_6,short imu_7,short imu_8,short imu_9,short power_1, unsigned char ult_byte);
//发送指定字符数组的函数
void USART_Send_String(unsigned char *p,unsigned short sendSize);     
//计算八位循环冗余校验，得到校验值，一定程度上验证数据的正确性
unsigned char getCrc8(unsigned char *ptr, unsigned short len); 
 
#endif
