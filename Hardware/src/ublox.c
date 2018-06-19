/*************************************************
Copyright (C)  FACCON Tech. Co., Ltd.
File name: ubolx.c
Author:ZH
Version:V1.O
Date:2018/01/09
Description: ��ʼ��GPS,��������Э��,GPS������,���ݸ�������,NAV����ʹ��
Others: 
Function List: 
1.ubloxInitGps();GPS��ʼ��
2.ubloxSet_PRT();�޸�GPS������,Э��
3.ubloxSet_Rate();�޸�GPS���ݸ�������
4.ubloxmsg_Enable();ʹ��������Ϣ
5.ublox_Prepare();����,����GPS��ʼ���������
6.ublox_NAV_POSLLH();����NAV_POSLL����
7.ublox_NAV_STATUS();����NAV_STATUS����
8.ublox_NAV_VELNED();����NAV_VELNED����
History: 
*************************************************/

#include "ublox.h"
#include "usart.h"
#include "include.h"
#include "Senser.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "portable.h"
#include "FreeRTOSConfig.h"


void usart_SetBaudRate(uint32_t Baud)
{
     USART3_Config(Baud);
}


void ublox_Send(uint8_t *data,uint16_t len)
{
	   Usart3_Send(data,len);
}

/***********************************************************************************************
*������ ��ubloxInitGps
*������������ ����ʼ��GPS
*�������� ����
*��������ֵ ����
*���� ��
*������������ �� 
*�����޸����� �� 
*�޸��� ��
*�޸�ԭ�� �� 
*�汾 �� 
*��ʷ�汾 �� 
***********************************************************************************************/
void ublox_InitGps(void)
{
	usart_SetBaudRate(9600);//��ʼ������
	vTaskDelay(pdMS_TO_TICKS(100));
	ubloxSet_PRT(115200,UBLOX_PROTOCOL_UBX,UBLOX_PROTOCOL_UBX);	
	vTaskDelay(pdMS_TO_TICKS(100));
	usart_SetBaudRate(115200);//���³�ʼ������
	vTaskDelay(pdMS_TO_TICKS(100));
	ubloxSet_Rate(80);
	vTaskDelay(pdMS_TO_TICKS(100));
	ubloxmsg_Enable(UBLOX_NAV_CLASS,UBLOX_NAV_PVT);
	vTaskDelay(pdMS_TO_TICKS(100));
}


/***********************************************************************************************
*������ ��ubloxSet_PRT
*������������ ������ublox����Э��;������;
*�������� ��baud;inProtoMask;outProtoMask
*��������ֵ ����
*���� ��
*������������ �� 
*�����޸����� �� 
*�޸��� ��
*�޸�ԭ�� �� 
*�汾 �� 
*��ʷ�汾 �� 
***********************************************************************************************/
void ubloxSet_PRT(uint32_t baud,uint16_t inProtoMask,uint16_t outProtoMask){
//B5 62 06 00 14 00 01 00 00 00 00 08 CO 00 00 C2 01 00 01 00 01 00 00 00 00 00 (CK_A CK_B)
	 unsigned char data_to_send[100];
	 unsigned char data_cnt = 0;  
	    
   union {unsigned char B[4];uint16_t D;uint32_t T;}src;
	 uint8_t CK_A = 0,CK_B = 0;
	 uint8_t i;
	 
	//==========֡ͷ=========
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR1;	
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR2;	 
	//==========CLASS===============
	data_to_send[data_cnt++] = UBLOX_CFG_CLASS;
	//==========ID================
	data_to_send[data_cnt++] = UBLOX_CFG_PRT;
	//==========LEN================
	data_to_send[data_cnt++] = 0x14;
	data_to_send[data_cnt++] = 0x00;// LEN
	//==========Data===============	 
	data_to_send[data_cnt++] = UBLOX_PortNum_UART1;//portID
	data_to_send[data_cnt++] = 0x00;//reserved1
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;//txready
	data_to_send[data_cnt++] = 0xC0;
	data_to_send[data_cnt++] = 0x08;
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;//mode
	 
	src.T = baud;
	data_to_send[data_cnt++] = src.B[0];
	data_to_send[data_cnt++] = src.B[1];
	data_to_send[data_cnt++] = src.B[2];
	data_to_send[data_cnt++] = src.B[3];//baudrate

	src.D = inProtoMask;
	data_to_send[data_cnt++] = src.B[0];
	data_to_send[data_cnt++] = src.B[1];//inProtoMask
		
	src.D = outProtoMask;
	data_to_send[data_cnt++] = src.B[0];
	data_to_send[data_cnt++] = src.B[1];//outProtoMask
		
	data_to_send[data_cnt++]=0x00;
	data_to_send[data_cnt++]=0x00;//flags
	
	data_to_send[data_cnt++]=0x00;
	data_to_send[data_cnt++]=0x00;//reserved2

	//==========LEN================
	src.D = data_cnt - 6;
	data_to_send[4] = src.B[0];
	data_to_send[5] = src.B[1];
	//==========SUM================
	for(i=2;i<data_cnt;i++)
	{
		CK_A = CK_A + data_to_send[i];
		CK_B = CK_B + CK_A;
	}
	data_to_send[data_cnt++]=CK_A;
	data_to_send[data_cnt++]=CK_B;
	//=========Send=================
	ublox_Send(data_to_send,data_cnt);			
}

/***********************************************************************************************
*������ �� ubloxSet_Rate
*������������ ���������ݸ���Ƶ��eg:100ms=10hz
*�������� �� ms 
*��������ֵ �� 
*���� ��
*������������ �� 
*�����޸����� �� 
*�޸��� ��
*�޸�ԭ�� �� 
*�汾 �� 
*��ʷ�汾 �� 
***********************************************************************************************/
void ubloxSet_Rate(unsigned short int ms) {
//B5 62 06 08 06 00    64 00 01 00 01 00     7A 12
	unsigned char data_to_send[100];
	unsigned char data_cnt = 0;      
    union {unsigned char B[4];uint16_t D;}src;
	 uint8_t CK_A = 0,CK_B = 0;
	 uint8_t i;
	//==========֡ͷ=========
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR1;
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR2;
	//==========CLASS===============
	data_to_send[data_cnt++] = UBLOX_CFG_CLASS;
	//==========ID================
	data_to_send[data_cnt++] = UBLOX_CFG_RATE;
	//==========LEN================
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;
	//==========Data===============	 
	src.D = ms;
		data_to_send[data_cnt++] = src.B[0];
		data_to_send[data_cnt++] = src.B[1];//100ms --- 10hz
	 
		data_to_send[data_cnt++] = 0x01;
		data_to_send[data_cnt++] = 0x00;//navRate
	 
		data_to_send[data_cnt++] = 0x01;
		data_to_send[data_cnt++] = 0x00;//GPS-Time		
	//==========LEN================
	src.D = data_cnt - 6;
	data_to_send[4] = src.B[0];
	data_to_send[5] = src.B[1];
	//==========SUM================
	for(i=2;i<data_cnt;i++)
	{
		CK_A = CK_A + data_to_send[i];
		CK_B = CK_B + CK_A;
	}
	data_to_send[data_cnt++]=CK_A;
	data_to_send[data_cnt++]=CK_B;
	//=========Send=================
	ublox_Send(data_to_send,data_cnt);			
}

/***********************************************************************************************
*������ �� ubloxmsg_Enable
*������������ ��ʹ��NAV����
*�������� �� msgClass,msgID
*��������ֵ �� 
*���� ��
*������������ �� 
*�����޸����� �� 
*�޸��� ��
*�޸�ԭ�� �� 
*�汾 �� 
*��ʷ�汾 �� 
***********************************************************************************************/
void ubloxmsg_Enable(uint8_t msgClass,uint8_t msgID)
{
//NAV-STATUS: B5 62 06 01 08 00  01 03 00 01 00 00 00 00  14 C5  	
//NAV-POSLLH: B5 62 06 01 08 00  01 02 00 01 00 00 00 00  13 BE
//NAV-VELNED: B5 62 06 01 08 00  01 12 00 01 00 00 00 00  23 2E	
	unsigned char data_to_send[100];
	unsigned char data_cnt = 0;      
    union {unsigned char B[2];uint32_t D;}src;
	uint8_t CK_A = 0,CK_B = 0;
	uint8_t i;	
	//==========֡ͷ=========
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR1;
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR2;
	//==========CLASS===============
	data_to_send[data_cnt++] = UBLOX_CFG_CLASS;
	//==========ID================
	data_to_send[data_cnt++] = UBLOX_CFG_MSG;
	//==========LEN================
	data_to_send[data_cnt++] = 0x06;
	data_to_send[data_cnt++] = 0x00;
	//==========Data===============	 
	data_to_send[data_cnt++] = msgClass;
	data_to_send[data_cnt++] = msgID;
	
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x01;
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;
	//==========LEN================
	src.D = data_cnt - 6;
	data_to_send[4] = src.B[0];
	data_to_send[5] = src.B[1];
	//==========SUM================
	for(i=2;i<data_cnt;i++)
	{
		CK_A = CK_A + data_to_send[i];
		CK_B = CK_B + CK_A;
	}
	data_to_send[data_cnt++]=CK_A;
	data_to_send[data_cnt++]=CK_B;
	//=========Send=================
	ublox_Send(data_to_send,data_cnt);	
}

/***********************************************************************************************
*������ ��ublox_Protocol_Prepare(); 
*������������ �����ա������յ���GPS����
*�������� �� data
*��������ֵ �� 
*���� ��
*������������ �� 
*�����޸����� �� 
*�޸��� ��
*�޸�ԭ�� �� 
*�汾 �� 
*��ʷ�汾 �� 
***********************************************************************************************/
uint8_t ublox_RxBuffer[256];
uint8_t ublox_XXBuffer[256];
uint8_t ublox_State = 0;

void ublox_Prepare(uint8_t data)
{	
	static uint8_t _data_cnt = 0;
	static uint16_t _data_len;
	static uint8_t i = 0;
	uint16_t Sum = 0,Check = 1;
	union {uint8_t B[2];uint16_t D;}src;
	
	ublox_XXBuffer[i++] = data;
	
	if(ublox_State==0&&data==UBLOX_SYNC_CHAR1)//0xb5
	{
		ublox_State=1;
		ublox_RxBuffer[0]=data;
	}
	else if(ublox_State==1&&data==UBLOX_SYNC_CHAR2)//0x62
	{
		ublox_State=2;   
		ublox_RxBuffer[1]=data;
	}
	else if(ublox_State==2&&data==UBLOX_NAV_CLASS)//class-0x01
	{
		ublox_State=3;
		ublox_RxBuffer[2]=data;		
	}
	else if(ublox_State==3)//id
	{
		ublox_State=4;
		ublox_RxBuffer[3]=data;		
	}
	else if(ublox_State==4)//len1
	{
		_data_cnt = 4;
		ublox_State=5;
		ublox_RxBuffer[_data_cnt++]=data;
	}
	else if(ublox_State==5)//len2
	{
		ublox_State=6;
		ublox_RxBuffer[_data_cnt++]=data;       
		src.B[0] = ublox_RxBuffer[4];
		src.B[1] = ublox_RxBuffer[5];
		_data_len = src.D + 2;
	}
	else if(ublox_State==6)
	{
		_data_len--;//PAYLODE���ݳ���
		ublox_RxBuffer[_data_cnt++]=data;
		if(_data_len==0)
		{
			Sum = ublox_CheckSum(ublox_RxBuffer,_data_cnt);			
			src.B[0] = ublox_RxBuffer[_data_cnt-2];//CK_A
			src.B[1] = ublox_RxBuffer[_data_cnt-1];//CK_B			
			Check = src.D;			
			if(Sum == Check)
			{			      
				switch(ublox_RxBuffer[3])//�ж�ID 
				{   
					case UBLOX_NAV_VELNED:
					{
						ublox_NAV_VELNED(ublox_RxBuffer);//����VELNED
					}break;
					case UBLOX_NAV_STATUS:
					{
						ublox_NAV_STATUS(ublox_RxBuffer);//����STATUS
					}break;
					case UBLOX_NAV_POSLLH: 
					{
						ublox_NAV_POSLLH(ublox_RxBuffer);//����POSLLH
					}break;	
          case UBLOX_NAV_PVT:
					{
						ublox_NAV_PVT(ublox_RxBuffer);//����PVT
					}break;						
					default  : ublox_State=0;
						break;
				}
        ublox_State=0;				
			}
			else ublox_State = 0;
		}	
	}
	else ublox_State = 0;
}

/***********************************************************************************************
*������ �� CheckSum();
*������������ :У���
*�������� �� buff,len
*��������ֵ �� 
*���� ��
*������������ �� 
*�����޸����� �� 
*�޸��� ��
*�޸�ԭ�� �� 
*�汾 �� 
*��ʷ�汾 �� 
***********************************************************************************************/
uint16_t ublox_CheckSum(uint8_t *buff,uint16_t len)
{   
	uint8_t i,CK_A = 0, CK_B = 0;
	
	union {uint8_t B[2];uint16_t D;}src;
	
	for(i=2;i<(len - 2);i++)
	{
		CK_A = CK_A + buff[i];
		CK_B = CK_B + CK_A;
	}
	
	src.B[0] = CK_A;
	src.B[1] = CK_B;
	return src.D;
}

/***********************************************************************************************
*������ �� ublox_NAV_VELNED();ublox_NAV_STATUS();ublox_NAV_POSLLH();
*������������ �� ������ӦNAV����
*�������� �� data
*��������ֵ �� 
*���� ��
*������������ �� 
*�����޸����� �� 
*�޸��� ��
*�޸�ԭ�� �� 
*�汾 �� 
*��ʷ�汾 �� 
***********************************************************************************************/

#define DeepOfFilter 5
//_xyz_f_ned V_NED[DeepOfFilter],A_NED,V_SUM,V_S;


uint16_t FilterCount = 0,FilterSum = 0;

void ublox_NAV_PVT(uint8_t *data)
{
	union {uint8_t B[4];uint16_t I2[2];uint16_t U2[2];int32_t I4;uint32_t U4;}src;	
	uint8_t src_count = 0,data_count = 6;
	uint32_t iTOW,tAcc,nano,flags,flags2,vAcc,sAcc,headAcc;
	
	float hMSL,hAcc;
	
	static float PeriodTime = 0,LastTime = 0;
	static float LastSpeed_N = 0,LastSpeed_E = 0,LastSpeed_D = 0;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	iTOW = src.U4;
	
	PeriodTime = (iTOW - LastTime) * 1e-3;
	LastTime = iTOW;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	Senser.GPS.Date.Year = src.U2[0];

	Senser.GPS.Date.Month = data[data_count++];
	Senser.GPS.Date.Day   = data[data_count++];
	Senser.GPS.Date.Hour  = data[data_count++];
	Senser.GPS.Date.Minute= data[data_count++];
	Senser.GPS.Date.Second= data[data_count++];
	
	Senser.GPS.isValid = data[data_count++];
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	tAcc = src.U4;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	nano = src.I4;

	Senser.GPS.gpsFix = data[data_count++];
	flags  = data[data_count++];
	flags2 = data[data_count++];
	Senser.GPS.NumberSV = data[data_count++];
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	Senser.GPS.Longitude = src.I4 * 1e-7;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	Senser.GPS.Latitude = src.I4 * 1e-7;


	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	Senser.GPS.Altitude = src.I4 * 1e-3;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	hMSL = src.I4 * 1e-3;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	hAcc = src.U4 * 1e-3;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	vAcc = src.U4 * 1e-3;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	Senser.GPS.NorthSpeed = src.I4 * 1e-3;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	Senser.GPS.EastSpeed = src.I4 * 1e-3;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	Senser.GPS.DownSpeed = src.I4 * 1e-3;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	Senser.GPS.Speed = src.I4 * 1e-3;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	Senser.GPS.Course = src.I4 * 1e-5;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	sAcc = src.I4 * 1e-3;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	headAcc = src.I4 * 1e-5;
	
	//...
	
	
//	//V_NED[0].n = 
//	FilterSum ++;
//	if(FilterSum >= DeepOfFilter) FilterSum = DeepOfFilter;
//	
//	//����ǰ�����ݴ��������������
//	V_SUM.n -= V_NED[FilterCount].n;
//	V_SUM.e -= V_NED[FilterCount].e;
//	V_SUM.d -= V_NED[FilterCount].d;
//	//�������������������������
//	V_SUM.n += Senser.alternate.gps1.NorthSpeed;
//	V_SUM.e += Senser.alternate.gps1.EastSpeed;
//	V_SUM.d += Senser.alternate.gps1.DownSpeed;
//	//��������������
//	V_NED[FilterCount].n = Senser.alternate.gps1.NorthSpeed;
//	V_NED[FilterCount].e = Senser.alternate.gps1.EastSpeed;
//	V_NED[FilterCount].d = Senser.alternate.gps1.DownSpeed;
// 
//  FilterCount++;
//	if(FilterCount >= DeepOfFilter) FilterCount = 0;
// 
//  V_S.n = V_SUM.n/FilterSum;
//  V_S.e = V_SUM.e/FilterSum;
//	V_S.d = V_SUM.d/FilterSum;
	
//	if((PeriodTime != 0)&&(Senser.alternate.gps1.valid & 0x0f))
//	{
//	    Senser.alternate.gps1.NorthAccelerate = (V_S.n - LastSpeed_N)/PeriodTime;
//		  Senser.alternate.gps1.EastAccelerate  = (V_S.e - LastSpeed_E)/PeriodTime;
//		  Senser.alternate.gps1.DownAccelerate  = (V_S.d - LastSpeed_D)/PeriodTime;
//		
//		  LastSpeed_N = V_S.n;
//		  LastSpeed_E = V_S.e;
//		  LastSpeed_D = V_S.d; 
//	}
//	else
//	{
//		  Senser.alternate.gps1.NorthAccelerate = 0;
//		  Senser.alternate.gps1.EastAccelerate  = 0;
//		  Senser.alternate.gps1.DownAccelerate  = 0;
//		  
//		  LastSpeed_N = Senser.alternate.gps1.NorthSpeed;
//		  LastSpeed_E = Senser.alternate.gps1.EastSpeed;
//		  LastSpeed_D = Senser.alternate.gps1.DownSpeed; 
//	}
		
	
}









int32_t NAV_VELNED[9];
int32_t NAV_STATUS[7];
int32_t NAV_POSLLH[7];
float UpdateTime;
void ublox_NAV_VELNED(uint8_t *data)
{
	union {uint8_t B[4];int32_t W;}src;	
	uint8_t src_count = 0,data_count = 6;
	uint32_t iTOW,speed,sAcc,cAcc;
	
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	iTOW=NAV_VELNED[0];
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	Senser.GPS.NorthSpeed = NAV_VELNED[1] * 1e-2;

	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	Senser.GPS.EastSpeed = NAV_VELNED[2] * 1e-2;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	Senser.GPS.DownSpeed = NAV_VELNED[3] * 1e-2;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	speed=NAV_VELNED[4] * 1e-2;

	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	Senser.GPS.Speed = NAV_VELNED[5] * 1e-2;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	Senser.GPS.Course = NAV_VELNED[6] * 1e-5;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	sAcc=NAV_VELNED[7];

	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	cAcc=NAV_VELNED[8];
	
	
	
	
	
	
}

void ublox_NAV_STATUS(uint8_t *data)
{
	union {uint8_t B[4];uint32_t D;uint8_t T;}src;	
	uint8_t src_count = 0,data_count = 6;
	uint32_t iTOW,ttff,msss;
	uint8_t gpsFix,flags,fixStat,flags2;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_STATUS[src_count++] = src.D;
	iTOW = NAV_STATUS[0];
	
	src.T = data[data_count++];
	NAV_STATUS[src_count++] = src.T;
	gpsFix = NAV_STATUS[1];
	
	src.T  = data[data_count++];
	NAV_STATUS[src_count++] = src.T;
	flags = NAV_STATUS[2];

	src.T  = data[data_count++];
	NAV_STATUS[src_count++] = src.T;
	fixStat = NAV_STATUS[3];
	
	src.T  = data[data_count++];
	NAV_STATUS[src_count++] = src.T;
	flags2 = NAV_STATUS[4];
		
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.D;
	ttff = NAV_STATUS[5];
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.D;
	msss = NAV_STATUS[6];	
}

void ublox_NAV_POSLLH(uint8_t *data)
{
	union {uint8_t B[4];uint32_t D;}src;	
	uint8_t src_count = 0,data_count = 6;
	uint32_t iTOW,hAcc,vAcc;
	float hMSL;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	iTOW = NAV_POSLLH[0];
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	Senser.GPS.Longitude = NAV_POSLLH[1] * 1e-7;

	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	Senser.GPS.Latitude = NAV_POSLLH[2] * 1e-7;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	Senser.GPS.Altitude = NAV_POSLLH[3] * 1e-3 ;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	hMSL = NAV_POSLLH[4];

	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	hAcc = NAV_POSLLH[5];
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	vAcc = NAV_POSLLH[6];	
}

