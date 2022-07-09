#ifndef __WS2812B_H
#define	__WS2812B_H

#include "stm32f10x.h"
#include "delay.h"	

//#define WS2812_IN_PIN	PA0

void Timer2_init(void);
void WS2812_led1_send(uint8_t (*color)[3], uint16_t len);
void WS2812_led2_send(uint8_t (*color)[3], uint16_t len);
void WS2812_led3_send(uint8_t (*color)[3], uint16_t len);
void WS2812_led4_send(uint8_t (*color)[3], uint16_t len);

#endif /* __LED_H */
