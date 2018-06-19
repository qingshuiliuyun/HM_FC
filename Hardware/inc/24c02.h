



#ifndef _24C02_H_
#define _24C02_H_
#include "stm32f10x.h"

#define Addr_24CXX 0X50

unsigned char EEPROM_WriteData(uint16_t address,uint8_t *data,uint32_t number);
unsigned char EEPROM_ReadData(uint16_t address,uint8_t *buffer,uint32_t number);

unsigned char EEPROM_nWriteData(uint16_t address,uint8_t *data,uint32_t number);
unsigned char EEPROM_nReadData(uint16_t address,uint8_t *buffer,uint32_t number);


#endif










