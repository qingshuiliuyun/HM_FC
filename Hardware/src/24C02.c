


#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "24c02.h"
#include "sys.h"
 #include "i2c.h"

//注意，这个eeprom是atmelH416 256K bit = 32768bytes,因此需要两个字节存储地址



// EEPROM写一个字节数据
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

// EEPROM读1字节数据
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



//使用下面这两个函数读写速度快一些
uint8_t EEPROM_nWriteData(uint16_t REG_Address,uint8_t *buf,uint32_t len)
{
	  int32_t Integral = 0,Decimal = 0;
	  int32_t LenCount = 0;
	  Integral = len/64;//得到整数部分，说明有那么多个64字节
	  Decimal  = len%64;//得到小数部分，说明有那么多个多余的字节
	  
	  if(len == 0) return 1;//返回错误标志
	
	  while(Integral>-1)
		{
			if(Integral != 0)
			{
				  LenCount = 64;
			}
			else
			{
				  if(Decimal == 0) return 0;//如果刚好整除，那么到这里应该已经传输结束
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
			
			
			Integral--;//整数部分开始减1
		}
	return 0;
}


uint8_t EEPROM_nReadData(uint16_t REG_Address,uint8_t *buf,uint32_t len)
{
		int32_t Integral = 0,Decimal = 0;
	  int32_t LenCount = 0;
	  Integral = len/64;//得到整数部分，说明有那么多个64字节
	  Decimal  = len%64;//得到小数部分，说明有那么多个多余的字节
	
	  if(len == 0) return 1;//返回错误标志
	
	  while(Integral>-1)
		{
			if(Integral != 0)
			{
				  LenCount = 64;
			}
			else
			{
				  if(Decimal == 0) return 0;//如果刚好整除，那么到这里应该已经传输结束
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
	
			Integral--;//整数部分开始减1
		}
	
	return 0;
}








