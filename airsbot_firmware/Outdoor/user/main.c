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
#include "ch_serial.h"

//代表手柄上的不同按键
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2         9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      26



//These are stick values
#define PSS_RX 5                //右摇杆X轴数据
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8


//控制B10,B11,B14,B15四个引脚用于手柄通讯，其中B10下拉输入，其余推挽输出
#define DI   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 1           //PB12  输入

#define DO_H GPIO_SetBits(GPIOB,GPIO_Pin_11)       //命令位高
#define DO_L GPIO_ResetBits(GPIOB,GPIO_Pin_11)

#define CS_H GPIO_SetBits(GPIOB,GPIO_Pin_14)     //CS拉高
#define CS_L GPIO_ResetBits(GPIOB,GPIO_Pin_14)      //CS拉低

#define CLK_H GPIO_SetBits(GPIOB,GPIO_Pin_15)    //时钟拉高
#define CLK_L GPIO_ResetBits(GPIOB,GPIO_Pin_15)      //时钟拉低
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))

static raw_t raw = {0};                                         /* IMU stram read/control struct */
static uint8_t decode_succ;                               /* 0: no new frame arrived, 1: new frame arrived */

typedef struct
{
    uint8_t code;
    char    name[8];
}item_code_name_t;

const item_code_name_t item_code_name[] = 
{
    {0x90, "id"},
    {0xA0, "acc"},
    {0xB0, "gyr"},
    {0xC0, "mag"},
    {0xD0, "eul"},
    {0xD1, "quat"},
    {0xF0, "pressure"},
    {0x91, "IMUSOL"},   /* collection of acc,gyr,mag,eul etc. to replace A0,B0,C0,D0... see user manual*/
    {0x60, "GWSOL"},    /* HI221 node imu data collection  see user manual */
};

static const char *code2name(uint8_t code)
{
    const char *p = NULL;
    int i;
    for(i=0; i<ARRAY_SIZE(item_code_name); i++)
    {
        if(code == item_code_name[i].code)
        {
            p = item_code_name[i].name;
        }
    }
    return p;
}

u32 ID[3];
u16 Handkey;
u8 Comd[2]={0x01,0x42};	//开始命令，请求数据，
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//按键值与按键明 


//上位机发送数据
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
short power01 =0;
unsigned char ultrasonic = 0x00;

uint16_t CH[18];   
int sn;
uint8_t  rc_flag = 0;
uint8_t USART1_RX_BUF[26];

//上位机接收数据
int testRece1     =20;
int testRece2     =20;
unsigned char testRece3 = 0x10;
float d_yaw;
float displacement;

struct odometry{
	float x;
	float y;
	float yaw;}odom;


//用于控制灯带闪烁的rgb数组
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
//引脚配置函数
void LED_Config(void); 
void LED_Config2(void);
void LED_Config3(void);
void LED_Config4(void);
void putter_init(void);
void pwm_init(void);
void TIM4_Init(u16 ar,u16 rs);
int usartReceiveimu(char imu_1[],char imu_2[],char imu_3[],char imu_4[],char imu_5[],char imu_6[]);
void Sbus_Data_Count(uint8_t *buf);
void dump_imu_data(raw_t *raw);
u8 PS2_RedLight(void);//判断是否为红灯模式
void PS2_ReadData(void);
void PS2_Cmd(u8 CMD);		  //
u8 PS2_DataKey(void);		  //键值读取
u8 PS2_AnologData(u8 button); //得到一个摇杆的模拟量
void PS2_ClearData(void);	  //清除数据缓冲区 

extern int flag;
extern int count;
//用于存储电机传来的数据，在CAN的中断函数里得到更新
extern double vl;
extern double vr;
extern double power_val;
extern int charing;
extern int percentage;
extern int protection_1;
extern int protection_2;
extern int ntc[6];
int vl0;
int vr0;
float dx;
float dy;
float dl = 0.0, dr = 0.0;
float sinval = 0.0, cosval = 0.0; 
int ox,oy,oa;
int vl1,vr1;
char ult[] = {0xe8,0x02,0xbc};
int dis_tmp=0;
int dis_hl=0;
int distance[5]={0,0,0,0,0};
int ctr_flag=0;
int fin_flag=0;
int d=0;
int bit=0;
int remote_vl=0;
int remote_vr=0;
int ch0,ch1;
/////////////////////////////////////////////////////////
//用于和轮毂电机通信的数据，电机站号为1
CanTxMsg TxMsg1={0x601,0,CAN_ID_STD,CAN_RTR_DATA,5,{0x2F,0x60,0x60,0,3}};  //选择速度环
CanTxMsg TxMsg2={0x601,0,CAN_ID_STD,CAN_RTR_DATA,6,{0x2B,0x40,0x60,0,0x80,0}};   //复位错误指令
CanTxMsg TxMsg3={0x601,0,CAN_ID_STD,CAN_RTR_DATA,6,{0x2B,0x40,0x60,0,0x0F,0}};   //便捷开是使能
CanTxMsg ROSMsg1= {0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0xFF,0x60,0x00,0,0,0,0}}; 
CanTxMsg ROSMsg1b={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0xFF,0x68,0x00,0,0,0,0}}; 
CanTxMsg ROSMsg2= {0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0xFF,0x60,0x00,0,0,0,0}}; 
CanTxMsg ROSMsg2b={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0xFF,0x68,0x00,0,0,0,0}}; 
 
CanTxMsg maxMsg= {0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x80,0x60,0x00,0xD0,0x07,0,0}}; 
 


CanTxMsg ROSMsgs= {0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0xFF,0x60,0x00,0,0,0,0}};//速度设置为0r/min
CanTxMsg ROSMsgsb={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0xFF,0x68,0x00,0,0,0,0}};//速度设置为0r/min

CanTxMsg PDOMsg_hb={0x000,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}; 

CanTxMsg PDOMsg0={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x01,0x18,0x01,0x81,0x02,0x00,0x80}}; 
CanTxMsg PDOMsg1={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x01,0x1A,0x01,0x20,0x00,0x64,0x60}}; 
CanTxMsg PDOMsg3={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x01,0x1A,0x02,0x20,0x00,0x69,0x60}}; 
CanTxMsg PDOMsg4={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2F,0x01,0x1A,0x00,0x02,0x00,0x00,0x00}}; 
CanTxMsg PDOMsg5={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2F,0x01,0x18,0x02,0xFF,0x00,0x00,0x00}};
CanTxMsg PDOMsg6={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2B,0x01,0x18,0x03,0x64,0x00,0x00,0x00}};
CanTxMsg PDOMsg7={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x2B,0x01,0x18,0x05,0x0A,0x00,0x00,0x00}};
CanTxMsg PDOMsg8={0x601,0,CAN_ID_STD,CAN_RTR_DATA,8,{0x23,0x01,0x18,0x01,0x81,0x02,0x00,0x00}};

///////////////////////////////////////////////////////
CanTxMsg powerMsg={0x100,0,CAN_ID_STD,CAN_RTR_DATA,8,{0,0,0,0,0,0,0,0}};
CanTxMsg powerMsg2={0x101,0,CAN_ID_STD,CAN_RTR_DATA,8,{0,0,0,0,0,0,0,0}};
CanTxMsg powerMsg3={0x102,0,CAN_ID_STD,CAN_RTR_DATA,8,{0,0,0,0,0,0,0,0}};
CanTxMsg powerMsg4={0x105,0,CAN_ID_STD,CAN_RTR_DATA,8,{0,0,0,0,0,0,0,0}};
//下面全部为手柄通信相关函数
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //输出以为控制位
		}
		else DO_L;

		CLK_H;                        //时钟拉高
		delay_us(50);
		CLK_L;
		delay_us(50);
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
}

u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//读取手柄数据
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;

	CS_L;

	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据

	for(byte=2;byte<9;byte++)          //开始接受数据
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			CLK_L;
			delay_us(50);
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(50);
	}
	CS_H;	
}

//对读出来的PS2的数据进行处理      只处理了按键部分         默认数据是红灯模式  只有一个按键按下时
//按下为0， 未按下为1
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //没有任何按键按下
}

//得到一个摇杆的模拟量	 范围0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//清除数据缓冲区
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}

/////////////////////////////////

int main(void)
{
	Stm32_Clock_Init(9); //系统时钟设置   												
	 
	USART1_Configuration();
	USART2_Configuration();
 // USART3_Configuration();
	UART4_Configuration();
	bsp_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM4_Init(1999,719);
  Timer2_init();
	LED_Config();//设置推挽输出
	LED_Config2();//设置推挽输出
	LED_Config3();//控制LED灯带
	pwm_init(); //imu
	putter_init();
	delay_init(72);	//延时初始化
	
 
  CAN1_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);  //初始化CAN 	
  CAN2_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);  //初始化CAN 		
  Delay(2000);
 
 
  flag=2; //用于判断运动状态的标志
	
	//控制板载输出电源
	GPIO_SetBits(GPIOB,GPIO_Pin_6);
  GPIO_SetBits(GPIOB,GPIO_Pin_8);
	GPIO_SetBits(GPIOB,GPIO_Pin_9);
	GPIO_ResetBits(GPIOE,GPIO_Pin_0); 
	GPIO_ResetBits(GPIOE,GPIO_Pin_1);
  GPIO_SetBits(GPIOD,GPIO_Pin_0);
	GPIO_SetBits(GPIOD,GPIO_Pin_1);
 
 
  //printf("start!!!");
  CAN_SendData(CAN1,&PDOMsg_hb);
	delay_ms(1);
	
	CAN_SendData(CAN1,&TxMsg1);
	delay_ms(1);
	CAN_SendData(CAN1,&TxMsg2);
	delay_ms(1);
	CAN_SendData(CAN1,&TxMsg3);
	delay_ms(1);

 
 
	CAN_SendData(CAN1,&PDOMsg0);
	delay_ms(1);
	CAN_SendData(CAN1,&PDOMsg1);
	delay_ms(1);
	CAN_SendData(CAN1,&PDOMsg3);
	delay_ms(1);
	CAN_SendData(CAN1,&PDOMsg4);
	delay_ms(1);
	CAN_SendData(CAN1,&PDOMsg5);
	delay_ms(1);
	CAN_SendData(CAN1,&PDOMsg6);
	delay_ms(1);
	CAN_SendData(CAN1,&PDOMsg7);
	delay_ms(1);
	CAN_SendData(CAN1,&PDOMsg8);
	delay_ms(1);
 
  CAN_SendData(CAN1,&maxMsg);
	delay_ms(1);
	
	while (1)
	{   
		  //printf("heart beat!!\r\n");		
      //读取电池电量
		  CAN_SendData(CAN2,&powerMsg);
			delay_ms(1);
			CAN_SendData(CAN2,&powerMsg2);
			delay_ms(1);
			CAN_SendData(CAN2,&powerMsg3);
			delay_ms(1);
			CAN_SendData(CAN2,&powerMsg4);
      delay_ms(1);
	  
		  
		  //printf("charing: %d\r\n",charing); 是否充电
		  //printf("percentage: %d\r\n",percentage); 剩余电量百分比
		  //printf("protection_1: %d\r\n",protection_1); 保护标志位1
			//printf("protection_2: %d\r\n",protection_2); 保护标志位2
	   	//printf("ntc: %d ",ntc[0]); 电池温度
 
			//读取超声波数据 
		  if(dis_tmp==0){
				//printf("d0!!\r\n");	
				dis_hl=0;
				USART_SendData(USART2, 0xd0);
				delay_us(50);
				USART_SendData(USART2, 0x02);
				delay_us(50);
				USART_SendData(USART2, 0xbc);
				delay_ms(50);
			}
			if(dis_tmp==1){
				//printf("d2!!\r\n");	
				dis_hl=0;
				USART_SendData(USART2, 0xd2);
				delay_us(50);
				USART_SendData(USART2, 0x02);
				delay_us(50);
				USART_SendData(USART2, 0xbc);
				delay_ms(50);
			}
			if(dis_tmp==2){
				//printf("d4!!\r\n");	
				dis_hl=0;
				USART_SendData(USART2, 0xd4);
				delay_us(50);
				USART_SendData(USART2, 0x02);
				delay_us(50);
				USART_SendData(USART2, 0xbc);
				delay_ms(50);
			}
			if(dis_tmp==3){
				//printf("d6!!\r\n");	
				dis_hl=0;
				USART_SendData(USART2, 0xd6);
				delay_us(50);
				USART_SendData(USART2, 0x02);
				delay_us(50);
				USART_SendData(USART2, 0xbc);
				delay_ms(50);
			}
			if(dis_tmp==4){
				//printf("d8!!\r\n");	
				dis_hl=0;
				USART_SendData(USART2, 0xd8);
				delay_us(50);
				USART_SendData(USART2, 0x02);
				delay_us(50);
				USART_SendData(USART2, 0xbc);
				delay_ms(50);
			}
			if(dis_tmp<4){
				dis_tmp++;
			}
			else{
				dis_tmp=0;
			}
			//printf("dis_tmp: %d\r\n",dis_tmp);
     // printf("ulttrasonic data:");
		  for(d =0;d<5;d++){
				//printf(" %d",distance[d]);
				bit=1<<d;
			if(distance[d]<3000){
				ultrasonic=ultrasonic|bit;
			}
			else{
				ultrasonic=ultrasonic&(~bit);
			}
			}
			 
			//imu数据读取
			if(decode_succ)
				{
					 decode_succ = 0;
				   //dump_imu_data(&raw);
				}
         
			//控制灯带闪烁
			//实际使用过程中调用这个函数设置颜色数组跟灯带长度即可： WS2812_led2_send(&rgb1[i],8);
			//更改一次之后灯带会一直保持设定颜色
			WS2812_led2_send(rgb0,8);
			for(i=0;i<53;i++)
			{
				WS2812_led2_send(&rgb1[i],8);
				delay_ms(1);
			}
			for(i=0;i<53;i++)
			{
				WS2812_led2_send(&rgb2[i],8);
				delay_ms(1);
			}
			for(i=0;i<53;i++)
			{
				WS2812_led2_send(&rgb3[i],8);
				delay_ms(1);
			}
			
			if(flag==2){
			//sbus飞控数据解析
			 Sbus_Data_Count(USART1_RX_BUF);
		 
				
			 // if(CH[4]>650&&CH[4]<1350){
			 //
			 //}
      //sbus遥控
				if(CH[4]<650&&CH[4]>100){
			 if (((ch0-CH[3])<500&&(ch0-CH[3])>-500)&&((ch1-CH[1])<500&&(ch1-CH[1])>-500)){
					if((CH[3]<1100&&CH[3]>900  &&  CH[1]<1100&&CH[1]>900) ||(CH[3]==0&&CH[1]==0))
					{
							remote_vl=0;
							remote_vr=0;
					}
					else{
						remote_vl=-(CH[1]-1000)/1;
						remote_vr=-(CH[1]-1000)/1;
						remote_vl=remote_vl+(CH[3]-1000)/2;
						remote_vr=remote_vr-(CH[3]-1000)/2;	
					}
			//printf("vl: %d , vr: %d \r\n",remote_vl,remote_vr);
				}
			 
				if (  
						((distance[1]>300&&remote_vl>=remote_vr)&&((distance[2]>300&&distance[3]>300&&remote_vl<0&&remote_vr<0)||(distance[4]>300&&remote_vl>0&&remote_vr>0)))
						||((distance[0]>300&&remote_vl<remote_vr)&&((distance[2]>300&&distance[3]>300&&remote_vl<0&&remote_vr<0)||(distance[4]>300&&remote_vl>0&&remote_vr>0)))
				    ||(distance[1]>300&&remote_vl>0&&remote_vr<0)
				    ||(distance[0]>300&&remote_vl<0&&remote_vr>0)
				   ){
							
								 if(remote_vl>0){
									 ROSMsg1.Data[4]=0x00+remote_vl%256;
									 ROSMsg1.Data[5]=0x00+remote_vl/256;
									 ROSMsg1.Data[6]=0x00;
									 ROSMsg1.Data[7]=0x00;
								 }
								 if(remote_vl==0){
									 ROSMsg1.Data[4]=0x00;
									 ROSMsg1.Data[5]=0x00;
									 ROSMsg1.Data[6]=0x00;
									 ROSMsg1.Data[7]=0x00;
								 }
								 if(remote_vl<0){
									 ROSMsg1.Data[4]=0xFF+((remote_vl+1)%256);
									 ROSMsg1.Data[5]=0xFF+((remote_vl+1)/256);
									 ROSMsg1.Data[6]=0xFF;
									 ROSMsg1.Data[7]=0xFF;
								 }
								 if(remote_vr>0){
									 ROSMsg1b.Data[4]=0xFF-((remote_vr-1)%256);
									 ROSMsg1b.Data[5]=0xFF-((remote_vr-1)/256);
									 ROSMsg1b.Data[6]=0xFF;
									 ROSMsg1b.Data[7]=0xFF;
								 }
								 if(remote_vr==0){
									 ROSMsg1b.Data[4]=0x00;
									 ROSMsg1b.Data[5]=0x00;
									 ROSMsg1b.Data[6]=0x00;
									 ROSMsg1b.Data[7]=0x00;
								 }
								 if(remote_vr<0){
									 ROSMsg1b.Data[4]=0x00-remote_vr%256;
									 ROSMsg1b.Data[5]=0x00-remote_vr/256;
									 ROSMsg1b.Data[6]=0x00;
									 ROSMsg1b.Data[7]=0x00;
								 }
								 
								CAN_SendData(CAN1,&ROSMsg1);
								delay_ms(1);
								CAN_SendData(CAN1,&ROSMsg1b);
								delay_ms(1);
							 }
							 
						else{
							CAN_SendData(CAN1,&ROSMsgs);
						delay_ms(1);
						CAN_SendData(CAN1,&ROSMsgsb);
						delay_ms(1);
						 }
					 }
				if(CH[4]>1350){
						CAN_SendData(CAN1,&ROSMsgs);
						delay_ms(1);
						CAN_SendData(CAN1,&ROSMsgsb);
						delay_ms(1);
				}
				 ch0=CH[3];
		     ch1=CH[1];
		 }
	  
		else if(flag==1){
			CAN_SendData(CAN1,&ROSMsg2);
			delay_ms(1);
			CAN_SendData(CAN1,&ROSMsg2b);
			delay_ms(1);
		}
		
		
}}


void RCC_Configuration(void)
{   
  SystemInit();
								  			 
}

//延时函数
void Delay(__IO uint32_t nCount)
{
    uint8_t x;
    for(; nCount != 0; nCount--)
	    for(x=0;x<100;x++);
}

//各个IO引脚的初始化，推挽输出 
void LED_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_14|GPIO_Pin_15;				   
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
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_8;				   
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

//推杆电机引脚初始化，开漏输出
void putter_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1;				   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);		     
}


void USART1_IRQHandler(void)
{
	 
	uint8_t res;
	uint8_t clear = 0;
	static uint8_t Rx_Sta = 1;
	 //printf("usart1");
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		res = USART_ReceiveData(USART1);	
		//printf(": %x",res);
		USART1_RX_BUF[Rx_Sta++] = res;
	}
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		clear = USART1->SR;
		clear = USART1->DR;
		USART_ClearITPendingBit(USART1,USART_IT_IDLE);
		USART1_RX_BUF[0] = Rx_Sta - 1;
		Rx_Sta = 1;
				
	}
	 if(USART_GetFlagStatus(USART1,USART_FLAG_ORE) != RESET)                                             	 //
    {                                                                                                    //
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1,USART_FLAG_ORE);                                                                    	 //
    }   
 
    
} 


void USART2_IRQHandler()
{ 
		uint8_t ch;
	 if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
			  if(dis_hl==0){
					ch = USART_ReceiveData(USART2);	
					distance[dis_tmp] = ch*256;
					dis_hl+=1;
				}
				else{
					ch = USART_ReceiveData(USART2);	
					distance[dis_tmp]+=ch;
				}
			 
		}
		if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)                    
    { 
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
		if(USART_GetFlagStatus(USART2,USART_FLAG_ORE) != RESET)                                             	 //
    {                                                                                                    //
		 USART_ClearFlag(USART2,USART_FLAG_ORE);                                                                    	 //
    }   
}

void USART3_IRQHandler(void)
{
  uint8_t ch;
	uint8_t clear = 0;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(USART3);	
    }
		//if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)                    
    //{ 
    //    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
   // }
    
    decode_succ = ch_serial_input(&raw, ch);
} 
 

void UART4_IRQHandler(void)
{
	if(USART_GetFlagStatus(UART4,USART_FLAG_ORE) != RESET)                                             	 //
    {                                                                                                    //
		USART_ReceiveData(UART4);
		USART_ClearFlag(UART4,USART_FLAG_ORE);                                                                    	 //
    } 	
		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
 	 {
       	 
		 USART_ClearITPendingBit(UART4,USART_IT_RXNE);//首先清除中断标志位
		 //从ROS接收到的数据，存放到下面三个变量中
		 usartReceiveOneData(&testRece1,&testRece2,&testRece3);
	   //当标志位为1时，把速度赋值给发送到CAN的Data数据
		 if(testRece3 ==0x01){
		 flag=1;
			 if(testRece2>0){
		     ROSMsg2.Data[4]=0x00+testRece2%256;
				 ROSMsg2.Data[5]=0x00+testRece2/256;
				 ROSMsg2.Data[6]=0x00;
				 ROSMsg2.Data[7]=0x00;
			 }
			 if(testRece2==0){
		     ROSMsg2.Data[4]=0x00;
				 ROSMsg2.Data[5]=0x00;
				 ROSMsg2.Data[6]=0x00;
				 ROSMsg2.Data[7]=0x00;
			 }
			 if(testRece2<0){
				 ROSMsg2.Data[4]=0xFF+((testRece2+1)%256);
				 ROSMsg2.Data[5]=0xFF+((testRece2+1)/256);
				 ROSMsg2.Data[6]=0xFF;
				 ROSMsg2.Data[7]=0xFF;
			 }
			 if(testRece1>0){
				 ROSMsg2b.Data[4]=0xFF-((testRece1-1)%256);
				 ROSMsg2b.Data[5]=0xFF-((testRece1-1)/256);
				 ROSMsg2b.Data[6]=0xFF;
				 ROSMsg2b.Data[7]=0xFF;
			 }
			 if(testRece1<0){
				 ROSMsg2b.Data[4]=0x00-testRece1%256;
				 ROSMsg2b.Data[5]=0x00-testRece1/256;
				 ROSMsg2b.Data[6]=0x00;
				 ROSMsg2b.Data[7]=0x00;
			 }
			 	 if(testRece1==0){
				 ROSMsg2b.Data[4]=0x00;
				 ROSMsg2b.Data[5]=0x00;
				 ROSMsg2b.Data[6]=0x00;
				 ROSMsg2b.Data[7]=0x00;
			 }
	   }
		 //标志位为2时，速度为0，机器人停止
		  if(testRece3 ==0x02){
				flag=2;
			}
	 }
		 
		   if(USART_GetITStatus(UART4, USART_IT_TXE) != RESET)                    
    { 
        USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
    }
  
} 
 
 
 
 //tim4初始化
 void TIM4_Init(u16 ar,u16 rs)
{
	TIM_TimeBaseInitTypeDef TIM_InitStrue;
	NVIC_InitTypeDef NVIC_InitStrue;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); 
	
	TIM_InitStrue.TIM_Period=ar; 
	TIM_InitStrue.TIM_Prescaler=rs; 
	TIM_InitStrue.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_InitStrue.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_InitStrue); 
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); 
	
	NVIC_InitStrue.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStrue); 

	TIM_Cmd(TIM4,ENABLE); 
}
//里程计计算
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)
	{
		 
		
		//电机回传速度除以1000为实际rpm
    vl1=vl;
    vr1=vr;
		//因为安装方式的原因，旋转方向相反时才能实现前进或后退，所以其中一个速度要取反
		vl1=-vl1;
			//电机回传速度除以1000为实际rpm
 
    dl = vl1 *98.91 *0.021/2400; // cm ，减速比40
    dr = vr1 *98.91 *0.021/2400; // cm ，减速比40
    d_yaw = (dl - dr) / 2.0f /26.2; //rad
    displacement = (dl + dr) / 2.0f;
    
    dx = cos(d_yaw)*displacement; //mm
    dy = sin(d_yaw)*displacement; //mm
    
    sinval = sin(odom.yaw), cosval = cos(odom.yaw);
    odom.x += (cosval * dx - sinval * dy)/100.f; //m
    odom.y += (sinval * dx + cosval * dy)/100.f; //m
    odom.yaw += d_yaw;  //rad
		//printf("odom: %d, %d, %d\r\n",ox,oy,oa);
    ox=odom.x*100;
		oy=odom.y*100;
		oa=odom.yaw*100;
		
		imu01=raw.imu[0].acc[0]*1000;
		imu02=raw.imu[0].acc[1]*1000;
		imu03=raw.imu[0].acc[2]*1000;
		imu04=raw.imu[0].gyr[0]*1000;
		imu05=raw.imu[0].gyr[1]*1000;
		imu06=raw.imu[0].gyr[2]*1000;
		imu07=raw.imu[0].eul[0]*1000;
		imu08=raw.imu[0].eul[1]*1000;
		imu09=raw.imu[0].eul[2]*1000;
		power01=power_val;
		//printf("num: %f" ,raw.imu[0].acc[0]*1000);
		 
	  //usartSendData(ox,oy,oa,imu01,imu02,imu03,imu04,imu05,imu06,imu07,imu08,imu09,power01,ultrasonic);	
 
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
	}
}	

//imu数据显示函数
static void dump_imu_data(raw_t *raw)
{
    int i;
    if(raw->item_code[0] != KItemGWSOL) /* HI226 HI229 CH100 CH110 */  
    {
        printf("%-16s%d\r\n",       "id:",  raw->imu[0].id);
        printf("%-16s%.3f %.3f %.3f\r\n",       "acc(G):",        raw->imu[0].acc[0], raw->imu[0].acc[1],  raw->imu[0].acc[2]);
        printf("%-16s%.3f %.3f %.3f\r\n",       "gyr(deg/s):",    raw->imu[0].gyr[0], raw->imu[0].gyr[1],  raw->imu[0].gyr[2]);
        printf("%-16s%.3f %.3f %.3f\r\n",       "mag(uT):",       raw->imu[0].mag[0], raw->imu[0].mag[1],  raw->imu[0].mag[2]);
        printf("%-16s%.3f %.3f %.3f\r\n",       "eul(deg):",      raw->imu[0].eul[0], raw->imu[0].eul[1],  raw->imu[0].eul[2]);
        printf("%-16s%.3f %.3f %.3f %.3f\r\n",  "quat:",          raw->imu[0].quat[0], raw->imu[0].quat[1],  raw->imu[0].quat[2], raw->imu[0].quat[3]);
        printf("%-16s%.3f\r\n",       "presure(pa):",  raw->imu[0].pressure);
        printf("%-16s%d\r\n",       "timestamp(ms):",  raw->imu[0].timestamp);
        printf("item: ");
        for(i=0; i<raw->nitem_code; i++)
        {
            printf("0x%02X(%s)", raw->item_code[i], code2name(raw->item_code[i]));
        }
        printf("\r\n");
    }
    else /* HI222(HI221GW) */
    {
        putchar(10);
        printf("gateway: %s%d, %s%d\r\n",       "gwid:",      raw->gwid, "node cnt:", raw->nimu);
        for(i=0; i<raw->nimu; i++)
        {
            putchar(10);
            printf("%-16s%d\r\n",       "id:",  raw->imu[i].id);
            printf("%-16s%.3f %.3f %.3f\r\n",       "acc(G):",        raw->imu[i].acc[0], raw->imu[i].acc[1],  raw->imu[i].acc[2]);
            printf("%-16s%.3f %.3f %.3f\r\n",       "gyr(deg/s):",    raw->imu[i].gyr[0], raw->imu[i].gyr[1],  raw->imu[i].gyr[2]);
            printf("%-16s%.3f %.3f %.3f\r\n",       "mag(uT):",       raw->imu[i].mag[0], raw->imu[i].mag[1],  raw->imu[i].mag[2]);
            printf("%-16s%.3f %.3f %.3f\r\n",       "eul(deg):",      raw->imu[i].eul[0], raw->imu[i].eul[1],  raw->imu[i].eul[2]);
            printf("%-16s%.3f %.3f %.3f %.3f\r\n",  "quat:",          raw->imu[i].quat[0], raw->imu[i].quat[1],  raw->imu[i].quat[2], raw->imu[i].quat[3]);
            printf("%-16s%.3f\r\n",       "presure(pa):",  raw->imu[i].pressure);
            printf("%-16s%d\r\n",       "timestamp(ms):",  raw->imu[i].timestamp);
        }
    }
}

//sbus解析函数
void Sbus_Data_Count(uint8_t *buf)
{
	CH[ 0] = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;
	CH[ 1] = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;
	CH[ 2] = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 )  | (int16_t)buf[ 6] << 10 ) & 0x07FF;
	CH[ 3] = ((int16_t)buf[ 6] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;
	CH[ 4] = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;
	CH[ 5] = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 )  | (int16_t)buf[10] <<  9 ) & 0x07FF;
	CH[ 6] = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;
	CH[ 7] = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;
	
	CH[ 8] = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8 )) & 0x07FF;
	CH[ 9] = ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5 )) & 0x07FF;
	CH[10] = ((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2 )  | (int16_t)buf[17] << 10 ) & 0x07FF;
	CH[11] = ((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7 )) & 0x07FF;
	CH[12] = ((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4 )) & 0x07FF;
	CH[13] = ((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1 )  | (int16_t)buf[21] <<  9 ) & 0x07FF;
	CH[14] = ((int16_t)buf[21] >> 2 | ((int16_t)buf[22] << 6 )) & 0x07FF;
	CH[15] = ((int16_t)buf[22] >> 5 | ((int16_t)buf[23] << 3 )) & 0x07FF;
}
 