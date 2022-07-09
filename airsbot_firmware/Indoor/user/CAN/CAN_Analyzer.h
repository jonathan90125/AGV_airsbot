

#ifndef __CAN_ANALYZER_H__
#define __CAN_ANALYZER_H__

#include "stm32f10x_can.h"
#include "CAN.h"
#include "usart.h"
#include<string.h>

#define  CAN_TBUFFLEN     100    // CAN����ʱ���¼�����С
#define  CAN_CBUFFLEN     100    // CACת����¼�����С
#define  CAN_USARTBUFLEN  100	 // CAN����ת�����ڷ��͵Ļ����С



void CAN_RX_Data(uint8_t port,CanRxMsg *RxMsg,uint16_t Tim);
void CAN_AnalyInit(void);
void Comm_Send_CANmsg(uint8_t port,CanRxMsg *RxMsg,uint16_t Tim);
void Comm_Send_CANmsg_str(uint8_t port,CanRxMsg *RxMsg,uint16_t Tim);
void Comm_Send_ID_str(uint8_t port,CanRxMsg *RxMsg);
void Comm_Send_All_ID_str(void);
void Comm_Send_N_CrossTransmit_str(void);
void Set_N_CrossTransmit(uint32_t ID);
void CAN_CrossTransmit(uint8_t port,CanRxMsg *Msg);
uint8_t CheckNotCross(uint32_t ID);
void show_change_ID(void);
void show_change_ticks(void);
void show_not_change(void);
void Debug_tick(void);
void Msg_Ctrl_Show(uint8_t port,CanRxMsg *RxMsg,uint16_t Tim);
void time_tick_1ms(void);


#endif






