




#ifndef  _ADC_H_
#define  _ADC_H_

#include "stm32f10x_conf.h"

#define ADC_EN

#ifdef ADC_EN

void ADC_Configuration(void);
void ADC_DMA_Configuration(void);
unsigned int Get_ADC_Value(ADC_TypeDef* ADCx);
void ADC_Filter(void);
#endif


#endif






