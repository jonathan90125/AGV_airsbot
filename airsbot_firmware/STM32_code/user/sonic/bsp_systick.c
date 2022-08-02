 
#include "bsp_systick.h"
#include "stm32f10x.h" 
 
volatile uint32_t _us_tick = 0;
volatile uint32_t _ms_tick = 0;

/**
  * @brief  initialize systick
  * @param  None
  * @retval None
  */
void systick_init(void) 
{
 
  _us_tick = 0;
  _ms_tick = 0;      
  /* SystemCoreClock / 1000000  1us�ж�һ�� */
  /* SystemCoreClock / 1000     1ms�ж�һ�� */
	if(SysTick_Config(72000000  / 1000000))
  {
    /*capture error*/
    while(1);
  }
  
  /* Configure the SysTick handler priority */
  //NVIC_SetPriority(SysTick_IRQn, 0x0);
}

 
uint32_t millis(void) 
{
	return _ms_tick;
}

/**
  * @brief  ��ȡϵͳ��ǰ��us����ֵ
  * @param  delay time
  * @retval ϵͳ��ǰʱ��us
  */
uint32_t micros(void)
{
  return _us_tick;
}

/**
  * @brief  ��λϵͳ����
  * @param  None
  * @retval None
  */
void systick_reset(void) 
{
	_us_tick = 0;
  _ms_tick = 0;
}
 

