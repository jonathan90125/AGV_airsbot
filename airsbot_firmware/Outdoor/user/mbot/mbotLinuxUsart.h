#ifndef __MBOTLINUXUSART__
#define __MBOTLINUXUSART__
#include <sys.h>	

#define START   0X11

//��linux���ղ��������ݵ�������ַ��
extern int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag);   
//��װ���ݣ�����USART1_Send_String�����ݷ��͸�linux
void usartSendData(short ox0, short oy0,short oa0,short imu_1,short imu_2,short imu_3,short imu_4,short imu_5,short imu_6,short imu_7,short imu_8,short imu_9,short power_1, unsigned char ult_byte);
//����ָ���ַ�����ĺ���
void USART_Send_String(unsigned char *p,unsigned short sendSize);     
//�����λѭ������У�飬�õ�У��ֵ��һ���̶�����֤���ݵ���ȷ��
unsigned char getCrc8(unsigned char *ptr, unsigned short len); 
 
#endif
