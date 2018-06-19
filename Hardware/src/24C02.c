


#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "24c02.h"
#include "sys.h"
 #include "i2c.h"

//ע�⣬���eeprom��atmelH416 256K bit = 32768bytes,�����Ҫ�����ֽڴ洢��ַ



// EEPROMдһ���ֽ�����
u8 EEPROM_Write_1Byte(u8 SlaveAddress,uint16_t REG_Address,u8 REG_data)
{
  IIC_Start();
	IIC_Send_Byte(SlaveAddress<<1);   
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte((uint8_t)(REG_Address>>8));     
	IIC_Wait_Ack();
	IIC_Send_Byte(REG_Address);     
	IIC_Wait_Ack();
	IIC_Send_Byte(REG_data);
	IIC_Wait_Ack();   
	IIC_Stop(); 
	return 0;
}

// EEPROM��1�ֽ�����
u8 EEPROM_Read_1Byte(u8 SlaveAddress,uint16_t REG_Address,u8 *REG_data)
{      		
	IIC_Start();
	IIC_Send_Byte(SlaveAddress<<1); 
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte((unsigned char)(REG_Address>>8));     
	IIC_Wait_Ack();
	IIC_Send_Byte(REG_Address);     
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(SlaveAddress<<1 | 0x01);
	IIC_Wait_Ack();
	*REG_data= IIC_Read_Byte(0);
	IIC_Stop();
	return 0;
}	









uint8_t EEPROM_WriteData(uint16_t address,uint8_t *data,uint32_t number)
{
	uint32_t i=0;
	for(i=0;i<number;i++)
	{
		EEPROM_Write_1Byte(Addr_24CXX,address,data[i]);
// 		Delay_ms(5);
		address += 1;	
  }
	return 0;
}


uint8_t EEPROM_ReadData(uint16_t address,uint8_t *buffer,uint32_t number)
{
	uint32_t i = 0;
	for(i=0;i<number;i++)
	{
		  EEPROM_Read_1Byte(Addr_24CXX,address,&buffer[i]);
// 		  Delay_ms(5);
		  address += 1;
  }
	return 0;
}



//ʹ������������������д�ٶȿ�һЩ
uint8_t EEPROM_nWriteData(uint16_t REG_Address,uint8_t *buf,uint32_t len)
{
	  int32_t Integral = 0,Decimal = 0;
	  int32_t LenCount = 0;
	  Integral = len/64;//�õ��������֣�˵������ô���64�ֽ�
	  Decimal  = len%64;//�õ�С�����֣�˵������ô���������ֽ�
	  
	  if(len == 0) return 1;//���ش����־
	
	  while(Integral>-1)
		{
			if(Integral != 0)
			{
				  LenCount = 64;
			}
			else
			{
				  if(Decimal == 0) return 0;//����պ���������ô������Ӧ���Ѿ��������
				  LenCount = Decimal;
			}

			IIC_Start();
			IIC_Send_Byte(Addr_24CXX<<1); 
			if(IIC_Wait_Ack())
			{
				IIC_Stop();
				return 1;
			}
			IIC_Send_Byte(REG_Address>>8);     
			IIC_Wait_Ack();
			IIC_Send_Byte(REG_Address);     
			IIC_Wait_Ack();
			while(LenCount--) 
			{
				IIC_Send_Byte(*buf++); 
				IIC_Wait_Ack();
			}
			IIC_Stop();
			
			
			Integral--;//�������ֿ�ʼ��1
		}
	return 0;
}


uint8_t EEPROM_nReadData(uint16_t REG_Address,uint8_t *buf,uint32_t len)
{
		int32_t Integral = 0,Decimal = 0;
	  int32_t LenCount = 0;
	  Integral = len/64;//�õ��������֣�˵������ô���64�ֽ�
	  Decimal  = len%64;//�õ�С�����֣�˵������ô���������ֽ�
	
	  if(len == 0) return 1;//���ش����־
	
	  while(Integral>-1)
		{
			if(Integral != 0)
			{
				  LenCount = 64;
			}
			else
			{
				  if(Decimal == 0) return 0;//����պ���������ô������Ӧ���Ѿ��������
				  LenCount = Decimal;
			}
	
			IIC_Start();
			IIC_Send_Byte(Addr_24CXX<<1); 
			if(IIC_Wait_Ack())
			{
				IIC_Stop();
				return 1;
			}
			IIC_Send_Byte(REG_Address>>8);     
			IIC_Wait_Ack();
			IIC_Send_Byte(REG_Address);     
			IIC_Wait_Ack();
			
			IIC_Start();
			IIC_Send_Byte(Addr_24CXX<<1 | 0x01); 
			IIC_Wait_Ack();
			while(LenCount) 
			{
				if(LenCount == 1)
				{
					*buf = IIC_Read_Byte(0);
				}
				else
				{
					*buf = IIC_Read_Byte(1);
				}
				buf++;
				LenCount--;
			}
			IIC_Stop();
	
			Integral--;//�������ֿ�ʼ��1
		}
	
	return 0;
}








