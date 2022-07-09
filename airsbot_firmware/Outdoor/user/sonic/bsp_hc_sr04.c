#include "bsp_hc_sr04.h"
#include "bsp_systick.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "usart.h"
#include "delay.h"

float distance_ultrasonic[UL_NUM];

 
static void ultrasonic_timConfig(uint16_t arr, uint16_t psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  // NVIC_InitTypeDef NVIC_InitStruct;
  
  UL_TIM_CLK_FUNCTION(UL_TIM_CLK, ENABLE);
 
  TIM_TimeBaseInitStruct.TIM_Period = arr;
  TIM_TimeBaseInitStruct.TIM_Prescaler = psc;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(UL_TIM, &TIM_TimeBaseInitStruct);
  
  //TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(UL_TIM, DISABLE); //Ĭ�Ϲر� 
}

static void ultrasonic_gpioConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;          
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_ResetBits(GPIOC, GPIO_Pin_8);  
  GPIO_ResetBits(GPIOC, GPIO_Pin_7); 
  GPIO_ResetBits(GPIOC, GPIO_Pin_9);	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); 
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;          
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); 
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;          
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;          
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void ultrasonic_extiConfig(void)
{
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //EXTI��Ҫʹ��AFIOʱ��
  
  GPIO_EXTILineConfig(UL_GPIO_PORT_SOURCE, GPIO_PinSource10 ); //���ж������ӵ�GPIO�˿�
  GPIO_EXTILineConfig(UL_GPIO_PORT_SOURCE, GPIO_PinSource11 );
	
  EXTI_InitStruct.EXTI_Line = EXTI_Line10|EXTI_Line11;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel = UL_EXTI_IRQ;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}

static void ultrasonic_extiConfig2(void)
{
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //EXTI��Ҫʹ��AFIOʱ��
 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);
  EXTI_InitStruct.EXTI_Line = EXTI_Line6;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel =  EXTI9_5_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}

void ultrasonic_init(void)
{
  ultrasonic_gpioConfig();
  ultrasonic_extiConfig();
	ultrasonic_extiConfig2();
  ultrasonic_timConfig(UL_TIM_ARR, UL_TIM_PSC);   //��ʼ��TIM2��ʱ��������һ��Ϊ1/100000S��10us����1/(72M/(719+1))s
}
	
//���η��ʹ����źţ���ѯ��������ֵ�����ݲ������ƣ��ú���ִ����ϣ��������������Ѿ�ִ����ɡ�
void ultrasonic_startMeasure(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_8);
	delay_us(20);
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
  delay_ms(10); //��ʱ��֤UL1�Ѿ������ж�\
	
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
	delay_us(20);
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
  delay_ms(10); //��ʱ��֤UL1�Ѿ������ж�
 
	GPIO_SetBits(GPIOC, GPIO_Pin_9);
	delay_us(20);
	GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  delay_ms(10); //��ʱ��֤UL1�Ѿ������ж�
}

 

void EXTI15_10_IRQHandler(void)
{		
  /*Ultrasonic 1*/	
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line10);  
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10))  //�����ش����жϺ�ȴ���Ϊ�͵�ƽ
    {
      if(TIM_GetCounter(TIM3) >= UL_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;  //�����趨�Ĳ��������룬�����ȴ�
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* ���㳬�����������ľ��룬�糬�������밴��������ʾ */
		distance_ultrasonic[0] = TIM_GetCounter(TIM3) * 340 / 2000.0;  //cnt * 1/100000 * 340 / 2 *100(��λ��cm)
	}
  
 	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line11);  
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11))  //�����ش����жϺ�ȴ���Ϊ�͵�ƽ
    {
      if(TIM_GetCounter(TIM3) >= UL_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;  //�����趨�Ĳ��������룬�����ȴ�
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* ���㳬�����������ľ��룬�糬�������밴��������ʾ */
		distance_ultrasonic[1] = TIM_GetCounter(TIM3) * 340 / 2000.0;  //cnt * 1/100000 * 340 / 2 *100(��λ��cm)
	}
  
}

void EXTI9_5_IRQHandler(void)
{		
  /*Ultrasonic 1*/	
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line6);  
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6))  //�����ش����жϺ�ȴ���Ϊ�͵�ƽ
    {
      if(TIM_GetCounter(TIM3) >= UL_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;  //�����趨�Ĳ��������룬�����ȴ�
      }
    }
		TIM_Cmd(TIM3,DISABLE);
    /* ���㳬�����������ľ��룬�糬�������밴��������ʾ */
		distance_ultrasonic[2] = TIM_GetCounter(TIM3) * 340 / 2000.0;  //cnt * 1/100000 * 340 / 2 *100(��λ��cm)
	}
 
  
}
