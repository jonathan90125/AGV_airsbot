#include "stm32f10x_can.h"
#include "CAN.h"
#include "usart.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>

u8 RxBuf[8];
u8 Rx_flag=0;
double vl=0;
double vr=0;
double power_val;
int charing;
int percentage;
int protection_1;
int protection_2;
int ntc[6];
/*****************************************
  CAN1 Config  CAN1 remap
  FIFO_0	  
  返回：
*****************************************/
void CAN1_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;

    /* 打开GPIO时钟、AFIO时钟，CAN时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


		/* CAN1 RX PA11 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* CAN1 TX PA12 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

	 //GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);  // CAN1 remap

    /* CAN1 Enabling interrupt */									  
    NVIC_InitStructure.NVIC_IRQChannel=CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);									
							  	
    /* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
	CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);   

    CAN_InitStructure.CAN_TTCM=DISABLE;
    CAN_InitStructure.CAN_ABOM=DISABLE;
    CAN_InitStructure.CAN_AWUM=DISABLE;
    CAN_InitStructure.CAN_NART=DISABLE;
    CAN_InitStructure.CAN_RFLM=DISABLE;
    CAN_InitStructure.CAN_TXFP=DISABLE;
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
	//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_SJW=sjw;
    CAN_InitStructure.CAN_BS1=bs1;  
    CAN_InitStructure.CAN_BS2=bs2;	
    CAN_InitStructure.CAN_Prescaler=pres;
    

    CAN_Init(CAN1,&CAN_InitStructure);	// CAN1											

    CAN_FilterInitStructure.CAN_FilterNumber=0;	 
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	 // 标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位过滤器
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			// 过滤器标识符
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;				
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;		// 过滤器屏蔽标识符
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	 // FIFO0指向过滤器
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);  // CAN1

}
 
void CAN2_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* 打开GPIO时钟、AFIO时钟，CAN时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	/* CAN2 RX PB12 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* CAN2 TX PB13 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);								

	/* CAN2 Enabling interrupt */								 	  
    NVIC_InitStructure.NVIC_IRQChannel=CAN2_RX1_IRQn;	// FIFO_1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);								  	

    /* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
	CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);   

    CAN_InitStructure.CAN_TTCM=DISABLE;
    CAN_InitStructure.CAN_ABOM=DISABLE;
    CAN_InitStructure.CAN_AWUM=DISABLE;
    CAN_InitStructure.CAN_NART=DISABLE;
    CAN_InitStructure.CAN_RFLM=DISABLE;
    CAN_InitStructure.CAN_TXFP=DISABLE;
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
	//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_SJW=sjw;
    CAN_InitStructure.CAN_BS1=bs1;  
    CAN_InitStructure.CAN_BS2=bs2;	
    CAN_InitStructure.CAN_Prescaler=pres;

    CAN_Init(CAN2,&CAN_InitStructure);   // CAN2													

    CAN_FilterInitStructure.CAN_FilterNumber=14;	// 
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	 // 标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位过滤器
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			// 过滤器标识符
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;				
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;		// 过滤器屏蔽标识符
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO1;	 // FIFO1指向过滤器
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);  // CAN2
}


/*****************************************
  CAN发送数据
*****************************************/
void CAN_SendData(CAN_TypeDef* CANx,CanTxMsg* CanData)
{
	 // int j;
    uint8_t retrys=0;
    uint8_t mailbox=0;

  //  do
	//{
	    mailbox=CAN_Transmit(CANx,CanData);
		retrys++;
	//}
	//while((mailbox==CAN_TxStatus_NoMailBox)&&(retrys<0xFE));
	retrys=0;
	 //for(j=0;j<8;j++){
	//		printf("%x  ",CanData->Data[j]);
	// }
}

 
static void Char2Str(char *Datout,char *Datin,unsigned char len)
{
    unsigned char j;
    
    for(j=0;j<len;j++)
    {
        sprintf(Datout,",%02X",Datin[j]);
        Datout+=3;	
    }	
}


/*****************************
  向串口发送一条CAN数据 字符串
  port    CAN端口 1：CAN1  2：CAN2
  RxMsg	  CAN数据
*****************************/
void Comm_Send_CANmsg_str(uint8_t port,CanRxMsg *RxMsg)
{
    char Buf[60];

	memset(Buf,0x00,60);  // 清空

	sprintf(Buf,"%d",port);    
	USART_STR(USART2,Buf);

	if(RxMsg->IDE==CAN_ID_STD) // 标准帧 
	{
	    sprintf(Buf,",S0x%08X",RxMsg->StdId);
		USART_STR(USART2,Buf);

		sprintf(Buf,",%d",RxMsg->DLC);
	    USART_STR(USART2,Buf);	
	    Char2Str(Buf,(char*)RxMsg->Data,8);
	    USART_STR(USART2,Buf);
	}
	else // 扩展帧
	{
	    sprintf(Buf,",E0x%08X",RxMsg->ExtId);
		USART_STR(USART2,Buf);

		sprintf(Buf,",%d",RxMsg->DLC);
	    USART_STR(USART2,Buf);	
	    Char2Str(Buf,(char*)RxMsg->Data,8);
	    USART_STR(USART2,Buf);
	}
	    
	USART_STR(USART2,"\r\n");				
}

 
void CAN1_RX0_IRQHandler(void)
{//int j=0;
  CanRxMsg RxMessage;

  RxMessage.StdId=0x00;
  RxMessage.ExtId=0x00;
  RxMessage.IDE=0;
  RxMessage.DLC=0;
  RxMessage.FMI=0;  
  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);  
 /*
	printf("id: %x\r\n", RxMessage.StdId);
	printf("data length: %d ,data:  ",sizeof(RxMessage.Data));
 
	   for(j=0;j<	 sizeof(RxMessage.Data) ;j++){
			printf("%x  ",RxMessage.Data[j]);}
 	printf(" \r\n" );
	printf(" \r\n" );
	*/
	  if(RxMessage.StdId==0x181 ){
			if(RxMessage.Data[3]<0x7F){
		 	vl=RxMessage.Data[0]+256*RxMessage.Data[1]+256*256*RxMessage.Data[2];}
			else{
					vl=(RxMessage.Data[0]-0xFF)+256*(RxMessage.Data[1]-0xFF)+256*256*(RxMessage.Data[2]-0xFF);
			}
			
			if(RxMessage.Data[7]<0x7F){
		 	vr=RxMessage.Data[4]+256*RxMessage.Data[5]+256*256*RxMessage.Data[6];}
			else{
					vr=(RxMessage.Data[4]-0xFF)+256*(RxMessage.Data[5]-0xFF)+256*256*(RxMessage.Data[6]-0xFF);
			} 
		   } 
}

void CAN2_RX1_IRQHandler(void)
{
	//int j=0;

  CanRxMsg RxMessage;
  
  RxMessage.StdId=0x00;
  RxMessage.ExtId=0x00;
  RxMessage.IDE=0;
  RxMessage.DLC=0;
  RxMessage.FMI=0;   
	
	 
  CAN_Receive(CAN2,CAN_FIFO1, &RxMessage);  
  //printf("powerid: %x  ",RxMessage.StdId);	
	if(RxMessage.StdId==0x100){
		power_val=RxMessage.Data[4]*256+RxMessage.Data[5];
		charing=RxMessage.Data[6];
		printf("data6: %x  ",RxMessage.Data[6]);	
	}
	
 	if(RxMessage.StdId==0x101){
		percentage=RxMessage.Data[4]*256+RxMessage.Data[5];}
	
	if(RxMessage.StdId==0x102){
		protection_1=RxMessage.Data[4]; 
		protection_2=RxMessage.Data[5]; 
	 }
	
	 	if(RxMessage.StdId==0x105){ 
		ntc[0]=RxMessage.Data[0]*256+RxMessage.Data[1];
		ntc[1]=RxMessage.Data[2]*256+RxMessage.Data[3]; 
		ntc[2]=RxMessage.Data[4]*256+RxMessage.Data[5];
	 }
		
	 
} 
 
