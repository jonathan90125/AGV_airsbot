  
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "usart.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>

#pragma import(__use_no_semihosting) 

_sys_exit(int x) 
{ 
  x = x; 
}
 
struct __FILE 
{ 
  int handle; 
}; 

FILE __stdout;

int fputc(int ch, FILE *f)
   {
      USART_SendData(UART4, (unsigned char) ch); 
      while (!(UART4->SR & USART_FLAG_TXE));
      return (ch);
   }
/*
  USART配置GPIO配置
*/

void USART2_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// 打开GPIOA时钟、AFIO时钟，USART2时钟  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	// USART2 TX PA2  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// USART2 RX PA3  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
											
	// USART 配置  
	USART_DeInit(USART2);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	// 使能USART2收发中断 
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);		   

	// 使能USART2 
	USART_Cmd(USART2, ENABLE);
	// 清除发送完成标志  
	USART_ClearFlag(USART2, USART_FLAG_TC);
  //USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);		   
  
    // Enable the USART2 Interrupt  
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			 //串口中断设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	 
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
  
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
											
	 
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 100000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;

	USART_Init(USART1, &USART_InitStructure);
 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);   
  USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

	 
	USART_Cmd(USART1, ENABLE);
	 
	//USART_ClearFlag(USART1, USART_FLAG_TC);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		   
  
   
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			 //串口中断设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
 
 
void USART3_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE);  
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 
 
 	//GPIO_FullRemap_USART3     D8   D9
 	GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);  
	//USART3_TX   GPIOD8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	 
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
   
  //USART3_RX	  GPIOD9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);		   

	// Enable the USART3 Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			 //串口中断设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  							
	// USART 配置 
	USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	// 使能USART3收发中断 
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART3, USART_IT_TXE, ENABLE);		   

	// 使能USART3 
	USART_Cmd(USART3, ENABLE);
	// 清除发送完成标志 
 
}
 

void UART4_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 打开GPIOA时钟、AFIO时钟，USART2时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* USART2 TX PC10 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* USART2 RX PC11 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
											
	/* USART 配置 */
	USART_DeInit(UART4);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	/* 使能USART4收发中断 */
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_ITConfig(UART4, USART_IT_TXE, ENABLE);		   

	/* 使能USART4 */
	USART_Cmd(UART4, ENABLE);
	/* 清除发送完成标志 */
	USART_ClearFlag(UART4, USART_FLAG_TC);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		   
  
    /* Enable the USART4 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;			 //串口中断设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
  串口发送函数
*/
void USART_Send(USART_TypeDef* USARTx, uint8_t *Dat,uint16_t len)
{
  uint16_t i;
  for(i=0;i<len;i++)
  {
  	USART_SendData(USARTx, Dat[i]);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);     
  }
}

/*
  串口发送字符串
*/
void USART_STR(USART_TypeDef* USARTx,char *str)
{
    uint8_t len,i;
	len=strlen(str);
	for(i=0;i<len;i++)
	{
	  	USART_SendData(USARTx, str[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET); 	
	}
}









