#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"

//�˿ں궨��
#define UL_PORT        GPIOB
#define UL_GPIO_CLK    RCC_APB2Periph_GPIOB
#define UL_GPIO_CLK_FUNCTION  RCC_APB2PeriphClockCmd
#define UL_GPIO_PORT_SOURCE  GPIO_PortSourceGPIOB

#define UL_TIM     TIM3
#define UL_TIM_CLK RCC_APB1Periph_TIM3
#define UL_TIM_CLK_FUNCTION RCC_APB1PeriphClockCmd

#define UL1_TRIG_PIN   GPIO_Pin_8
#define UL1_ECHO_PIN   GPIO_Pin_15
#define UL1_EXTI_LINE  EXTI_Line15
#define UL1_GPIO_PIN_SOURCE  GPIO_PinSource15 

#define UL2_TRIG_PIN   GPIO_Pin_6
#define UL2_ECHO_PIN   GPIO_Pin_7
#define UL2_EXTI_LINE  EXTI_Line7
#define UL2_GPIO_PIN_SOURCE  GPIO_PinSource14 

#define UL_EXTI_IRQ    EXTI15_10_IRQn

//
#define UL_NUM 3
#define UL_TIM_ARR 0xffff     //��������Ϊ���ֵ
#define UL_TIM_PSC 719        //����Ƶ�ʣ�72M/(719+1) = 100000������һ������ʱ��Ϊ��1S/100000=10us
#define MAX_MEASURE_DISTANCE 255      //��λ��cm
#define UL_TIM_MAX_COUNT 1500 //�����������Ӧ�Ķ�ʱ������  //cnt = 255cm * 2 /(340 * 100) * 100000
/* ��ʱ�����÷ǳ��ؼ�����ǰ�趨ʱ��Ҫȷ���ϸ��������Ѿ��������жϣ���Ϊ�������жϾͻ��Զ��ȵ�������ɣ���
   ���ϸ�������ɺ���ܿ����¸��������Ĳ������������������źŶ����ͳ�ȥ���ⲿ�жϻ����δ�����ִ�����յ�
   �����źŵ��жϷ���������һ���жϻ�ȵ���ǰ�ж�ִ�����֮��Ż���룬��ʱ�����źŵĸߵ�ƽʱ����ܺ�
   С��û���ˣ�����յ������ݻ���ʾ0���С����ֵ�� */
#define MEASURE_INTERVAL 5000  //��λ��us  //��ʵ��������ڵ���500us��Ϊ���ʣ�

extern float distance_ultrasonic[];


void ultrasonic_init(void);
void ultrasonic_startMeasure(void);



#endif /* __ULTRASONIC_H*/




