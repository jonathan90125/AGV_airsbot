#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_can.h"
#include "stm32f10x_usart.h"
#include "mbotLinuxUsart.h"
#include "usart.h"
#include "delay.h"
#include <stdio.h>
#include <math.h>
#include <CAN.h>
#include "sys.h"
#include "bsp.h"
#include "WS2812B.h"
#include "misc.h"

//��λ����������
short testSend1   =30;
short testSend2   =30;
short imu01   =1;
short imu02   =2;
short imu03   =3;
short imu04   =4;
short imu05   =5;
short imu06   =6;
short imu07   =7;
short imu08   =8;
short imu09   =9;
short imu010   =10;
unsigned char testSend4 = 0x05;

//��λ����������
int testRece1     =20;
int testRece2     =20;
unsigned char testRece3 = 0x10;
float d_yaw;
float displacement;

struct odometry{
	float x;
	float y;
	float yaw;}odom;


//���ڿ��Ƶƴ���˸��rgb����
uint8_t rgb0[][3] = {0,0,0};
uint8_t rgb1[53][3] = {{0,0,0},{10,0,0},{20,0,0},{30,0,0},{40,0,0},{50,0,0},{60,0,0},{70,0,0},{80,0,0},{90,0,0},
											 {100,0,0},{110,0,0},{120,0,0},{130,0,0},{140,0,0},{150,0,0},{160,0,0},{170,0,0},{180,0,0},{190,0,0},
											 {200,0,0},{210,0,0},{220,0,0},{230,0,0},{240,0,0},{250,0,0},{255,0,0},{250,0,0},{240,0,0},{230,0,0},
											 {220,0,0},{210,0,0},{200,0,0},{190,0,0},{180,0,0},{170,0,0},{160,0,0},{150,0,0},{140,0,0},{130,0,0},
											 {120,0,0},{110,0,0},{100,0,0},{90,0,0},{80,0,0},{70,0,0},{60,0,0},{50,0,0},{40,0,0},{30,0,0},
											 {20,0,0},{10,0,0},{0,0,0}};
uint8_t rgb2[53][3] = {{0,0,0},{0,10,0},{0,20,0},{0,30,0},{0,40,0},{0,50,0},{0,60,0},{0,70,0},{0,80,0},{0,90,0},
											 {0,100,0},{0,110,0},{0,120,0},{0,130,0},{0,140,0},{0,150,0},{0,160,0},{0,170,0},{0,180,0},{0,190,0},
											 {0,200,0},{0,210,0},{0,220,0},{0,230,0},{0,240,0},{0,250,0},{0,255,0},{0,250,0},{0,240,0},{0,230,0},
											 {0,220,0},{0,210,0},{0,200,0},{0,190,0},{0,180,0},{0,170,0},{0,160,0},{0,150,0},{0,140,0},{0,130,0},
											 {0,120,0},{0,110,0},{0,100,0},{0,90,0},{0,80,0},{0,70,0},{0,60,0},{0,50,0},{0,40,0},{0,30,0},
											 {0,20,0},{0,10,0},{0,0,0}};
uint8_t rgb3[53][3] = {{0,0,0},{0,0,10},{0,0,20},{0,0,30},{0,0,40},{0,0,50},{0,0,60},{0,0,70},{0,0,80},{0,0,90},
											 {0,0,100},{0,0,110},{0,0,120},{0,0,130},{0,0,140},{0,0,150},{0,0,160},{0,0,170},{0,0,180},{0,0,190},
											 {0,0,200},{0,0,210},{0,0,220},{0,0,230},{0,0,240},{0,0,250},{0,0,255},{0,0,250},{0,0,240},{0,0,230},
											 {0,0,220},{0,0,210},{0,0,200},{0,0,190},{0,0,180},{0,0,170},{0,0,160},{0,0,150},{0,0,140},{0,0,130},
											 {0,0,120},{0,0,110},{0,0,100},{0,0,90},{0,0,80},{0,0,70},{0,0,60},{0,0,50},{0,0,40},{0,0,30},
											 {0,0,20},{0,0,10},{0,0,0}};


u16 i,j;
void RCC_Configuration(void);
void Delay(__IO uint32_t nCount);
void LED_Config(void); 
void LED_Config2(void);
void LED_Config3(void);
void LED_Config4(void);
void putter_init(void);
void pwm_init(void);
void TIM4_Init(u16 ar,u16 rs);
 

extern int flag;
extern int count;
//���ڴ洢������������ݣ���CAN���жϺ�����õ�����
extern double vl;
extern double vr;
int vl0;
int vr0;
float dx;
float dy;
float dl = 0.0, dr = 0.0;
float sinval = 0.0, cosval = 0.0; 
int ox,oy,oa;
int vl1,vr1;

//���ں���챵��ͨ�ŵ����ݣ����վ��Ϊ1
CanTxMsg TxMsg1={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x60,0x60,0,3,0,0,0}};//ѡ�����ģʽ
CanTxMsg TxMsg2={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2B,0x40,0x60,0,0x0F,0,0,0}};//���ʹ�� 
CanTxMsg TxMsg3={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2B,0x40,0x60,0,0x06,0,0,0}};//������ᣬֹͣ
CanTxMsg TxMsg7={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x5A,0x60,0x11,0x01,0,0,0}};//��ͣ
CanTxMsg TxMsg8={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x5A,0x60,0x11,0,0,0,0}};//��ͣ���
//��ѯ�����ֵ
CanTxMsg DataMsg1={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2F,0x00,0x47,0x01,0x01,0,0,0}};//��PDO
CanTxMsg DataMsg2={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2F,0x00,0x47,0x01,0,0,0,0}};//�ر�PDO
CanTxMsg DataMsg3={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2F,0x00,0x18,0x03,0,10,0,0}};//��PDO


//ͬ�ϣ����վ��Ϊ2
CanTxMsg TxMsg1b={0x602,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x60,0x60,0,3,0,0,0}};//ѡ�����ģʽ
CanTxMsg TxMsg2b={0x602,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2B,0x40,0x60,0,0x0F,0,0,0}};//���ʹ�� 
CanTxMsg TxMsg3b={0x602,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2B,0x40,0x60,0,0x06,0,0,0}};//������ᣬֹͣ
CanTxMsg TxMsg7b={0x602,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x5A,0x60,0x11,0x01,0,0,0}};//��ͣ
CanTxMsg TxMsg8b={0x602,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x5A,0x60,0x11,0,0,0,0}};//��ͣ���
//��ѯ�����ֵ
CanTxMsg DataMsg1b={0x602,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2F,0x00,0x47,0x01,0x01,0,0,0}};//��PDO
CanTxMsg DataMsg2b={0x602,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2F,0x00,0x47,0x01,0,0,0,0}};//�ر�PDO
CanTxMsg DataMsg3b={0x602,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2F,0x00,0x18,0x03,0,10,0,0}};//��PDO


CanTxMsg BMsg; 
CanTxMsg BMsg2={0x0001,0,CAN_ID_STD,CAN_RTR_DATA,0 }; 
CanTxMsg ROSMsg1= {0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0xF0,0x2F,0x09,0,0,0,0}};//�ٶ�����Ϊ10r/min
CanTxMsg ROSMsg1b={0x602,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0xF0,0x2F,0x09,0,0,0,0}};//�ٶ�����Ϊ10r/min
CanTxMsg ROSMsgs= {0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0xF0,0x2F,0x09,0,0,0,0}};//�ٶ�����Ϊ10r/min
CanTxMsg ROSMsgsb={0x602,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0xF0,0x2F,0x09,0,0,0,0}};//�ٶ�����Ϊ10r/min

int main(void)
{
	Stm32_Clock_Init(9); //ϵͳʱ������   												
	USART_Configuration();
	USART3_Configuration();
	bsp_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM4_Init(1999,719);
 
	LED_Config();//�����������
	LED_Config3();//����LED�ƴ�
	pwm_init();
	putter_init();
	delay_init(72);	//��ʱ��ʼ��

	
  CAN1_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);  //��ʼ��CAN 	
  CAN2_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);  //��ʼ��CAN 		
  Delay(2000);
 
 
  flag=0; //�����ж��˶�״̬�ı�־
	
	//���ư��������Դ
	GPIO_SetBits(GPIOB,GPIO_Pin_6);
  GPIO_SetBits(GPIOB,GPIO_Pin_8);
	GPIO_SetBits(GPIOB,GPIO_Pin_9);
	GPIO_SetBits(GPIOD,GPIO_Pin_1);
	GPIO_ResetBits(GPIOE,GPIO_Pin_0); 
	GPIO_ResetBits(GPIOE,GPIO_Pin_1);
  //GPIO_ResetBits(GPIOD,GPIO_Pin_0);
	GPIO_SetBits(GPIOD,GPIO_Pin_0);
	//GPIO_SetBits(GPIOC,GPIO_Pin_0);
	//GPIO_SetBits(GPIOC,GPIO_Pin_2);

  //BMsg.StdId=0x0000;
	//BMsg.IDE=0; 
	//BMsg.RTR=0;
	//BMsg.DLC=0;
	//CAN_SendData(CAN2,&BMsg2); 

	//printf("start!!!!!!!!\n");    


	CAN_SendData(CAN1,&TxMsg1);
	delay_ms(1);
	CAN_SendData(CAN1,&TxMsg1b);
	delay_ms(1);
	CAN_SendData(CAN1,&TxMsg2);
	delay_ms(1);
	CAN_SendData(CAN1,&TxMsg2b);
	delay_ms(1);
	CAN_SendData(CAN1,&DataMsg1);
	delay_ms(1);
	CAN_SendData(CAN1,&DataMsg1b);
  delay_ms(1);

	while (1)
	{
   
					/*******************************
					//��������������
					ultrasonic_startMeasure();
					printf("\n");
					printf("UL1 = %.2f\n", distance_ultrasonic[0]);
					printf("UL2 = %.2f\n", distance_ultrasonic[1]);
					printf("UL3 = %.2f\n", distance_ultrasonic[2]);
					delay_ms(500);
					********************************/		

					/*******************************
					//��翪���Ƹ���λ		
					u8 key;  
					if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == 0){
					//printf("haven't reach the limit");		 
					}
					if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 0){
					GPIO_ResetBits(GPIOE,GPIO_Pin_0); 
					GPIO_ResetBits(GPIOE,GPIO_Pin_1);
					//printf("reach the limit");		 
					}
					//�����Ƹ˵��
					if(key==11){
					if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == 0){
					GPIO_ResetBits(GPIOE,GPIO_Pin_0); 
					GPIO_SetBits(GPIOE,GPIO_Pin_1);} }
					if(key==16){
					GPIO_ResetBits(GPIOE,GPIO_Pin_0); 
					GPIO_ResetBits(GPIOE,GPIO_Pin_1); }
					if(key==12){
					GPIO_SetBits(GPIOE,GPIO_Pin_0); 
					GPIO_ResetBits(GPIOE,GPIO_Pin_1); }			 
					if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 0){
					GPIO_ResetBits(GPIOE,GPIO_Pin_0); 
					GPIO_ResetBits(GPIOE,GPIO_Pin_1);
					}
          ********************************/	 
					
					/*******************************
					//���������
					if(key==6){
					printf("1");}
					if(key==8){
					printf("2");} 
					********************************/	 
					
          /*******************************
					//��ȡ�������
					if(key==15){
					//(CAN2,&BMsg); 
					//delay_ms(500);
					//CAN_SendData(CAN2,&BMsg);
					}
					********************************/	 
					
			//�����������ش�ʵ��ת��
			CAN_SendData(CAN1,&TxMsg1);
			delay_ms(1);
			CAN_SendData(CAN1,&TxMsg1b);
			 delay_ms(1);
			CAN_SendData(CAN1,&TxMsg2);
			delay_ms(1);
			CAN_SendData(CAN1,&TxMsg2b);
			delay_ms(1);
			CAN_SendData(CAN1,&DataMsg1);
			delay_ms(1);
			CAN_SendData(CAN1,&DataMsg1b);
			delay_ms(1);	
 
/******************** 
      //�������
		  CAN_SendData(CAN1,&TxMsg1);
			delay_ms(1);
			CAN_SendData(CAN1,&TxMsg1b);
			delay_ms(1);
			CAN_SendData(CAN1,&TxMsg3);
			delay_ms(1);
			CAN_SendData(CAN1,&TxMsg3b);
			delay_ms(1);
 ********************/
	
			if(flag==1){
			CAN_SendData(CAN1,&TxMsg1);
	    delay_ms(1);
			CAN_SendData(CAN1,&TxMsg1b);
			 delay_ms(1);
			CAN_SendData(CAN1,&TxMsg2);
		 delay_ms(1);
			CAN_SendData(CAN1,&TxMsg2b);
			 delay_ms(1);
			CAN_SendData(CAN1,&ROSMsg1);
			 delay_ms(1);
			CAN_SendData(CAN1,&ROSMsg1b);
				delay_ms(1);
			   }
 
			else if(flag==2){
		 //�������
		  CAN_SendData(CAN1,&TxMsg1);
			 delay_ms(1);
			CAN_SendData(CAN1,&TxMsg1b);
	    delay_ms(1);
			CAN_SendData(CAN1,&TxMsg3);
			 delay_ms(1);
			CAN_SendData(CAN1,&TxMsg3b);
			 delay_ms(3);
		}
	 
	 
		//vl0=vl;
		//vr0=vr;
    //usartSendData(vl0,vr0,imu01,imu02,imu03,imu04,imu05,imu06,imu07,imu08,imu09,imu010,testSend4);	
		//USART_STR(USART3,"AT+EOUT=1\r\n"); 
		 
	 
}}


void RCC_Configuration(void)
{   
  SystemInit();
								  			 
}

//��ʱ����
void Delay(__IO uint32_t nCount)
{
    uint8_t x;
    for(; nCount != 0; nCount--)
	    for(x=0;x<100;x++);
}

//����IO���ŵĳ�ʼ����������� 
void LED_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_11;				   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);			
}
void LED_Config2(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);         
}

void LED_Config3(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_8;				   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);			
}

void LED_Config4(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);       
}

void LED_Config5(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_3;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);       
}

void pwm_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); 
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1;				   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);		     
}

//�Ƹ˵�����ų�ʼ������©���
void putter_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1;				   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);		     
}

 
 
void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART2,USART_IT_RXNE);//��������жϱ�־λ
		 //��ROS���յ������ݣ���ŵ���������������
		 usartReceiveOneData(&testRece1,&testRece2,&testRece3);
	   //����־λΪ1ʱ�����ٶȸ�ֵ�����͵�CAN��Data����
		 if(testRece3 ==0x01){
		 flag=1;
			 if(testRece1>0){
		     ROSMsg1.Data[4]=0x00+testRece1;
				 ROSMsg1.Data[5]=0x00;
				 ROSMsg1.Data[6]=0x00;
				 ROSMsg1.Data[7]=0x00;
			 }
			 if(testRece1==0){
		     ROSMsg1.Data[4]=0x00;
				 ROSMsg1.Data[5]=0x00;
				 ROSMsg1.Data[6]=0x00;
				 ROSMsg1.Data[7]=0x00;
			 }
			 if(testRece1<0){
				 ROSMsg1.Data[4]=0xFF+(testRece1+1);
				 ROSMsg1.Data[5]=0xFF;
				 ROSMsg1.Data[6]=0xFF;
				 ROSMsg1.Data[7]=0xFF;
			 }
			 if(testRece2>0){
				 ROSMsg1b.Data[4]=0xFF-(testRece2-1);
				 ROSMsg1b.Data[5]=0xFF;
				 ROSMsg1b.Data[6]=0xFF;
				 ROSMsg1b.Data[7]=0xFF;
			 }
			 if(testRece2<0){
				 ROSMsg1b.Data[4]=0x00-testRece2;
				 ROSMsg1b.Data[5]=0x00;
				 ROSMsg1b.Data[6]=0x00;
				 ROSMsg1b.Data[7]=0x00;
			 }
			 	 if(testRece2==0){
				 ROSMsg1b.Data[4]=0x00;
				 ROSMsg1b.Data[5]=0x00;
				 ROSMsg1b.Data[6]=0x00;
				 ROSMsg1b.Data[7]=0x00;
			 }
	   }
		 //��־λΪ2ʱ���ٶ�Ϊ0��������ֹͣ
		  if(testRece3 ==0x02){
				flag=2;
			}
	 }
		 
		   if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)                    
    { 
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
}
 
 void TIM4_Init(u16 ar,u16 rs)
{
	TIM_TimeBaseInitTypeDef TIM_InitStrue;
	NVIC_InitTypeDef NVIC_InitStrue;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//???????
	
	TIM_InitStrue.TIM_Period=ar;//?????
	TIM_InitStrue.TIM_Prescaler=rs;//????????
	TIM_InitStrue.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_InitStrue.TIM_ClockDivision=TIM_CKD_DIV1;//??????:TDTS = Tck_tim
	TIM_TimeBaseInit(TIM4,&TIM_InitStrue);//??????,????????
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//???????
	
	NVIC_InitStrue.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStrue);//?????,????????

	TIM_Cmd(TIM4,ENABLE);//?????
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)
	{
		//����ش��ٶȳ���1000Ϊʵ��rpm
    vl1=vl/1000;
    vr1=vr/1000;
		//��Ϊ��װ��ʽ��ԭ����ת�����෴ʱ����ʵ��ǰ������ˣ���������һ���ٶ�Ҫȡ��
		vr1=-vr1;
			//����ش��ٶȳ���1000Ϊʵ��rpm
 
    dl = vl1 *43.96 *0.021/60; // cm
    dr = vr1 *43.96*0.021/60; // cm
    d_yaw = (dl - dr) / 2.0f /17.9; //rad
    displacement = (dl + dr) / 2.0f;
    
    dx = cos(d_yaw)*displacement; //mm
    dy = sin(d_yaw)*displacement; //mm
    
    sinval = sin(odom.yaw), cosval = cos(odom.yaw);
    odom.x += (cosval * dx - sinval * dy)/100.f; //m
    odom.y += (sinval * dx + cosval * dy)/100.f; //m
    odom.yaw += d_yaw;  //rad
    ox=odom.x*100;
		oy=odom.y*100;
		oa=odom.yaw*100;
		//printf("x,y,yaw:    %d, %d, %d \n",ox,oy,oa);
		//printf("#####    %f %f \n",vl,vr);
		usartSendData(ox,oy,oa,imu02,imu03,imu04,imu05,imu06,imu07,imu08,imu09,imu010,testSend4);	
 
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);//???????
	}
}	
 
