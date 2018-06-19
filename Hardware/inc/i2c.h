#ifndef __IOI2C_H
#define __IOI2C_H

#include "stm32f10x.h"
//IO��������	 
#define IIC_SCL    PBout(6)         // SCL     pb14   pb10
#define IIC_SDA    PBout(7)         // SDA	    pb15   pb11
#define READ_SDA   PBin(7)          // ����SDA 

//IIC���в�������
void IIC_Init(void);                // ��ʼ��IIC��IO��				 
void IIC_Start(void);				        // ����IIC��ʼ�ź�
void IIC_Stop(void);	  			      // ����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			    // IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);// IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				      // IIC�ȴ�ACK�ź�
void IIC_Ack(void);					        // IIC����ACK�ź�
void IIC_NAck(void);				        // IIC������ACK�ź�

u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
#endif

//------------------End of File----------------------------
