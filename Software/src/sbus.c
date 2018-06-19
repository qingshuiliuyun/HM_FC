



#include "sbus.h"
#include "usart.h"


void SBUS_Init(void)
{
	   USART1_Config(100000);//sbus固定的波特率
}

//[startbyte] [data1] [data2] .... [data22] [flags][endbyte]
u8 SBusRxBuffer[25];
u8 SBusState = 0;
void SBUS_Prepare(u8 data)
{
	  static u8 _data_len = 0,_data_cnt = 0;
		if(SBusState==0&&data== 0x0f)//第一个字
		{
					 SBusState=1;
					 SBusRxBuffer[0]=data;
			     _data_len = 24;
			     _data_cnt = 1;	
		}
		else if((SBusState==1)&&(_data_len>0))//第二个字
		{   
			    _data_len--;
					SBusRxBuffer[_data_cnt++]=data;
			    if(_data_len == 0)
					{
						    SBUS_Decode(SBusRxBuffer);
						    SBusState = 0;
						    _data_len = 0;
								_data_cnt = 0;	
          }
		}
		else
		{
	         SBusState = 0;
			     _data_len = 0;
			     _data_cnt = 0;	
		}
}


/*
bit7 = ch17 = digital channel (0x80)
bit6 = ch18 = digital channel (0x40)
bit5 = Frame lost, equivalent red LED on receiver (0x20)
bit4 = failsafe activated (0x10)
bit3 = n/a
bit2 = n/a
bit1 = n/a
bit0 = n/a
*/


uint16_t Chanel[16];
unsigned char SBUS_flag,SBUS_end;
void SBUS_Decode(u8 *data)
{
	   Chanel[0]  = (( data[2]&0x07)<<8)  |  data[1];
	   Chanel[1]  = (((data[3]&0x3F)<<8)  | (data[2]&0xF8)) >>3;
	   Chanel[2]  = (( data[5]&0x01)<<10) | (data[4]<<2)         | ((data[3]&0XC0)>>6);
	   Chanel[3]  = (((data[6]&0x0F)<<8)  | (data[5]&0XFE))>>1;
	   Chanel[4]  = (((data[7]&0x7F)<<8)  | (data[6]&0XF0))>>4;
	   Chanel[5]  = (( data[9]&0x03)<<9)  | (((data[8]<<8)       | (data[7]&0X80))>>7);
	   Chanel[6]  = (((data[10]&0x1F)<<8) | (data[9]&0xFC)) >> 2;
	   Chanel[7]  = (( data[11]<<8)       | (data[10]&0XE0)) >> 5;
	   Chanel[8]  = (( data[13]&0x07)<<8) |  data[12];
	   Chanel[9]  = (((data[14]&0x3F)<<8) | (data[13]&0xF8)) >>3;
	   Chanel[10] = (( data[16]&0x01)<<10)| (data[15]<<2)        | ((data[14]&0XC0)>>6);
	   Chanel[11] = (((data[17]&0x0F)<<8) | (data[16]&0XFE))>>1;
	   Chanel[12] = (((data[18]&0x7F)<<8) | (data[17]&0XF0))>>4;
	   Chanel[13] = (( data[20]&0x03)<<9) | (((data[19]<<8)      | (data[18]&0X80))>>7);
	   Chanel[14] = (((data[21]&0x1F)<<8) | (data[20]&0xFC)) >> 2;
	   Chanel[15] = (( data[22]<<8)       | (data[21]&0XE0)) >> 5;
	
	   SBUS_flag = data[23];
	   SBUS_end  = data[24];  
}




