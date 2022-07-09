#include "bsp.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
void bsp_init(void)
{ 
  /* 配置中断优先级分组 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  systick_init();
  ultrasonic_init();  //初始化超声波模块
}


