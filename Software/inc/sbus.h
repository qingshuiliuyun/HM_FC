

#ifndef _SBUS_H_
#define _SBUS_H_


#include "stm32f10x.h"



extern uint16_t Chanel[16];



void SBUS_Init(void);
void SBUS_Prepare(u8 data);
void SBUS_Decode(u8 *data);











#endif










