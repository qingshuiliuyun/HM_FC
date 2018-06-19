



#ifndef _USART_H_
#define _USART_H_
#include "stm32f10x.h"

#define USART_EN


#ifdef USART_EN

void USART1_Config(uint32_t Baud) ;
void Usart1_IRQ(void);
void Usart1_Send(unsigned char *DataToSend ,u8 data_num);


void USART2_Config(uint32_t Baud) ;
void Usart2_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);


void USART3_Config(uint32_t Baud) ;
void Usart3_IRQ(void);
void Usart3_Send(unsigned char *DataToSend ,u8 data_num);

#endif








#endif


























