#include "WS2812B.h"
#include "mbotLinuxUsart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
/* Buffer that holds one complete DMA transmission
 * 
 * Ensure that this buffer is big enough to hold
 * all data bytes that need to be sent
 * 
 * The buffer size can be calculated as follows:
 * number of LEDs * 24 bytes + 42 bytes
 * 
 * This leaves us with a maximum string length of
 * (2^16 bytes per DMA stream - 42 bytes)/24 bytes per LED = 2728 LEDs
 */
//#define TIM2_CCR1_Address 0x40000034 	// physical memory address of Timer 2 CCR1 register
#define TIM3_CCR3_Address 0x4000043C    // physical memory address of Timer 3 CCR1 register
#define TIM2_CCR1_Address 0x40000034
#define TIM2_CCR2_Address 0x40000038
#define TIM2_CCR3_Address 0x4000003C
#define TIM2_CCR4_Address 0x40000040
	
#define TIMING_ONE  50
#define TIMING_ZERO 25
uint16_t LED1_BYTE_Buffer[300];
uint16_t LED2_BYTE_Buffer[300];
uint16_t LED3_BYTE_Buffer[300];
uint16_t LED4_BYTE_Buffer[300];
 

void Timer2_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
	TIM_TimeBaseStructure.TIM_Period = 90-1; // 800kHz 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		
	/* PWM2 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	
	/* PWM3 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* DMA1 Channel2 Config for PWM1 by TIM2_CH1*/
 
	DMA_DeInit(DMA1_Channel2);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)TIM3_CCR3_Address;	// physical address of Timer 3 CCR1
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)LED2_BYTE_Buffer;		// this is the buffer memory 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						// data shifted from memory to peripheral
	DMA_InitStructure.DMA_BufferSize = 42;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// automatically increase buffer index
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// stop DMA feed after buffer size is reached
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	
 
//	TIM_DMACmd(TIM2, TIM_DMA_CC1, ENABLE);
	TIM_DMACmd(TIM3, TIM_DMA_CC3, ENABLE);
//	TIM_DMACmd(TIM2, TIM_DMA_CC3, ENABLE);
//	TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);
}
 


void WS2812_led2_send(uint8_t (*color)[3], uint16_t len)
{
	uint8_t i;
	uint16_t memaddr;
	uint16_t buffersize;
	buffersize = (len*24)+43;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	memaddr = 0;				// reset buffer memory index

	while (len)
	{	
		for(i=0; i<8; i++) // GREEN data
		{
			LED2_BYTE_Buffer[memaddr] = ((color[0][1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr++;
		}
		for(i=0; i<8; i++) // RED
		{
			LED2_BYTE_Buffer[memaddr] = ((color[0][0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr++;
		}
		for(i=0; i<8; i++) // BLUE
		{
			LED2_BYTE_Buffer[memaddr] = ((color[0][2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr++;
		}
		len--;
	}
 
  	LED2_BYTE_Buffer[memaddr] = ((color[0][2]<<8) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
 
	  memaddr++;	
		while(memaddr < buffersize)
		{
			LED2_BYTE_Buffer[memaddr] = 0;
			memaddr++;
		}

		DMA_SetCurrDataCounter(DMA1_Channel2, buffersize); 	// load number of bytes to be transferred
		DMA_Cmd(DMA1_Channel2, ENABLE); 			// enable DMA channel 7
		TIM_Cmd(TIM3, ENABLE); 						// enable Timer 2
		while(!DMA_GetFlagStatus(DMA1_FLAG_TC2)) ; 	// wait until transfer complete
		TIM_Cmd(TIM3, DISABLE); 	// disable Timer 2
		DMA_Cmd(DMA1_Channel2, DISABLE); 			// disable DMA channel 7
		DMA_ClearFlag(DMA1_FLAG_TC2); 				// clear DMA1 Channel 7 transfer complete flag
}

 
 




