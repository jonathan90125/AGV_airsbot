#ifndef __USART_H__
#define __USART_H__

 
void USART_Send(USART_TypeDef* USARTx, uint8_t *Dat,uint16_t len);
void USART_STR(USART_TypeDef* USARTx,char *str);
void USART1_Configuration(void);
void USART2_Configuration(void);
 void USART3_Configuration(void);
void UART4_Configuration(void);

#endif
extern int flag;
extern int count;
