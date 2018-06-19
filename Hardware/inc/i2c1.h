#ifndef _I2C1_H
#define _I2C1_H

#include "stm32f10x.h"

//IIC���в�������
void I2C1_Init(void);                // ��ʼ��IIC��IO��	

u8 I2C1_Write_1Byte(u8 SlaveAddress,unsigned short REG_Address,u8 REG_data);
u8 I2C1_Read_1Byte(u8 SlaveAddress,unsigned short REG_Address,u8 *REG_data);
u8 I2C1_Write_nByte(u8 SlaveAddress,unsigned short address,unsigned char number,unsigned char *data);
u8 I2C1_Read_nByte(u8 SlaveAddress,unsigned short address,unsigned char number,unsigned char *buffer);
#endif

//------------------End of File----------------------------
