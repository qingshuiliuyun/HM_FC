/*beijing-faccon-flightControlProtocl-Use-LittleEnding*/  

#include "include.h"
#include "protocol.h"
#include "usart.h"
#include "mpu6050.h"
#include "adc.h"
#include "senser.h"
#include "thread.h"


//echo
uint8_t Echo_CMD1 = 0;
uint8_t Echo_CMD2 = 0;
uint8_t Echo_CMD3 = 0;
uint8_t Echo_CMD4 = 0;
uint8_t Echo_GetRoute = 0;//has been recieved a point (0x07) 
uint8_t Echo_Error2 = 0;
uint8_t Echo_Error3 = 0;
uint8_t Echo_Error4 = 0;

//Route
double Protocol_RouteDot[5][10][7] = {0};//7.168kb大小
unsigned char Protocol_Send_cnt=1;

//=====================外部需要传输到地面站的数据定义============================
//姿态
int16_t Protocol_ACC_X,Protocol_ACC_Y,Protocol_ACC_Z;
int16_t Protocol_GYRO_X,Protocol_GYRO_Y,Protocol_GYRO_Z;
int16_t Protocol_MAG_X,Protocol_MAG_Y,Protocol_MAG_Z;
float Protocol_ROL,Protocol_PIT,Protocol_YAW;
//信号
int16_t   Protocol_RC_CH[10];//origin
int16_t   Protocol_PWM_motor[8];//0~100%
int16_t   Protocol_PWM_servo[4];//-30~+30 deg
//GPS
uint8_t Protocol_GPSStatus,Protocol_StarNum;
float Protocol_Latitude,Protocol_Longitude;
float Protocol_Altitude ,Protocol_Speed;
float Protocol_Course;
//状态
float Protocol_ALT_U,Protocol_ALT_P,Protocol_Votage1,Protocol_Votage2,Protocol_Current1,Protocol_Current2;
//航线状态
uint8_t  Protocol_CtrlMode,Protocol_AirplaneMode,Protocol_CurrentStatus,Protocol_LockStatus,Protocol_TurningStatus = 2;
uint16_t Protocol_CurrentTarget;
float    Protocol_AirSpeed,Protocol_WaitToFlight,Protocol_DiffrentDistance,Protocol_DiffrentAngle,Protocol_DiffrentHight,Protocol_DiffrentSpeed;
uint16_t Protocol_TargetRoll,Protocol_TargetPitch,Protocol_TargetYaw,Protocol_TargetHight;
float    Protocol_UseSpeed,Protocol_wzSpeed,Protocol_BoardHight;
//===============================================================================



void Protocol_Send(u8 *data,u8 Len)//发送函数封装，使用函数式
{
	  Usart2_Send(data,Len);//发送出去
}


//Transmission
unsigned char Protocol_Send_Echo = 0,
              Protocol_Send_Version = 0,
							Protocol_Send_Attiude = 0,
							Protocol_Send_Signal = 0,
							Protocol_Send_Status = 0,
							Protocol_Send_GPS1 = 0,
							Protocol_Send_PID1 = 0,
							Protocol_Send_PID2 = 0,
							Protocol_Send_PID3 = 0,
							Protocol_Send_PID4 = 0,
              Protocol_Send_TaskSet1 = 0,
              Protocol_Send_TaskSet2 = 0,
              Protocol_Send_TaskSet3 = 0,
              Protocol_Send_TaskSet4 = 0,
              Protocol_Send_TaskSet5 = 0,
              Protocol_Send_TaskSet6 = 0,
              Protocol_Send_TaskSet7 = 0,
							Protocol_Send_Route,
							Protocol_Send_RouteInfo;
							
void Protocol_Transmission(float T)//Send the Info to Station one by one;
{ 
	  
		static u16 cnt = 0;
		static u16 Echo_cnt 	    = 200;
		static u16 Attiude_cnt    = 100;
		static u16 Signal_cnt 	  = 150;
		static u16 Status_cnt 	  = 1000;
		static u16 GPS1_cnt 	    = 200;
	  static u16 SendRoute_cnt  = 100;//100ms发一次航点
		static u16 RouteInfo_cnt	= 250;

		
		if((cnt % Echo_cnt) == 0)
		Protocol_Send_Echo = 1;
		
		if((cnt % Attiude_cnt) == 0)
		Protocol_Send_Attiude = 1;
		
		if((cnt % Signal_cnt) == 0)
		Protocol_Send_Signal = 1;
		
		if((cnt % Status_cnt) == 0)
		Protocol_Send_Status = 1;
		
		if((cnt % GPS1_cnt) == 0)
		Protocol_Send_GPS1 = 1;
		
		if((cnt % SendRoute_cnt) == 0)
		Protocol_Send_Route = 1;
		
		if((cnt % RouteInfo_cnt) == 0)
		Protocol_Send_RouteInfo = 1;

		cnt += T *1000;
	  if(cnt>=1200) cnt = 0;
		
	  if(Protocol_Send_Echo)
		{
			 Protocol_Send_Echo = 0;
			 Protocol_T_Echo(Echo_CMD1,Echo_CMD2,Echo_CMD3,Echo_CMD4,Echo_GetRoute,Echo_Error2,Echo_Error3,Echo_Error4); 
			 Echo_CMD1 = 0;
			 Echo_CMD2 = 0;
			 Echo_CMD3 = 0;
			 Echo_CMD4 = 0;
			 Echo_GetRoute = 0;
			 Echo_Error2 = 0;
			 Echo_Error3 = 0;
			 Echo_Error4 = 0;
    }
		if(Protocol_Send_Version)
		{
			 Protocol_Send_Version = 0;
			 Protocol_T_Version(HardwareVER,SoftwareVER,ProtocolVER,AirplaneVER);
    }
		if(Protocol_Send_Attiude)//使用固定封装方式
		{
			 Protocol_Send_Attiude = 0;
// 			 Protocol_T_Attiude(Senser.acc.radian.x,Senser.acc.radian.y,Senser.acc.radian.z,
// 													Senser.gyro.radian.x,Senser.gyro.radian.y,Senser.gyro.radian.z,
// 													Senser.mag.radian.x,Senser.mag.radian.y,Senser.mag.radian.z,
// 													0,0,0);
    }
		if(Protocol_Send_Signal)//使用固定封装方式
		{
			Protocol_Send_Signal = 0;
			 Protocol_T_Signal(Protocol_RC_CH[0],Protocol_RC_CH[1],Protocol_RC_CH[2],Protocol_RC_CH[3],Protocol_RC_CH[4],
			                   Protocol_RC_CH[5],Protocol_RC_CH[6],Protocol_RC_CH[7],Protocol_RC_CH[8],Protocol_RC_CH[9],
			                   Protocol_PWM_motor[0],Protocol_PWM_motor[1],Protocol_PWM_motor[2],Protocol_PWM_motor[3],
			                   Protocol_PWM_motor[4],Protocol_PWM_motor[5],Protocol_PWM_motor[6],Protocol_PWM_motor[7],
			                   Protocol_PWM_servo[0],Protocol_PWM_servo[1],Protocol_PWM_servo[2],Protocol_PWM_servo[3]);
    }
		if(Protocol_Send_Status)
		{
			 Protocol_Send_Status = 0;
			 Protocol_T_Status(Protocol_ALT_U,   Protocol_ALT_P,
			                   Protocol_Votage1, Protocol_Votage2,
			                   Protocol_Current1,Protocol_Current2);
    }
		if(Protocol_Send_GPS1)
		{
			 Protocol_Send_GPS1 = 0;
			 Protocol_T_GPS1(Protocol_GPSStatus,Protocol_StarNum,
			                 Protocol_Latitude, Protocol_Longitude,
			                 Protocol_Altitude, Protocol_Speed,Protocol_Course);	
    }	
		if(Protocol_Send_PID1)
		{
			 Protocol_Send_PID1 = 0;
			 Protocol_T_PID1(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
    }
		if(Protocol_Send_PID2)
		{
			 Protocol_Send_PID2= 0;
			 Protocol_T_PID2(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
    }
		if(Protocol_Send_PID3)
		{
			 Protocol_Send_PID3 = 0;
			 Protocol_T_PID3(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
    }
		if(Protocol_Send_PID4)
		{
			 Protocol_Send_PID4= 0;
			 Protocol_T_PID4(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
    }
		if(Protocol_Send_Route)//航点传输还存在BUG，当没有航点时会传失败，导致无法传输航点，地面站死机
		{
					if((Protocol_GetRoute&0x10) == 0x10)//发送航点
					{
						  uint8_t Groups = (Protocol_GetRoute&0x0f);

						  if(Protocol_RouteDot[Groups][1][0] == 0)//如果第一个航点的第0位的值为0，那么说明该航点没有航线信息
							{
								Protocol_T_Route(Groups,
																	0,//总航点
																	0,//当前航点
																	0,//纬度
																	0,//经度
																	0,//高度
																	0,//速度
																	0,//半径
																	0);//动作
								Protocol_Send_cnt= 1;
								Protocol_GetRoute = 0;
							}
							else
							{
										Protocol_T_Route(Groups,
																		Protocol_RouteDot[Groups][Protocol_Send_cnt][0],//总航点
																		Protocol_Send_cnt,                              //当前航点
																		Protocol_RouteDot[Groups][Protocol_Send_cnt][2],//纬度
																		Protocol_RouteDot[Groups][Protocol_Send_cnt][1],//经度
																		Protocol_RouteDot[Groups][Protocol_Send_cnt][3],//高度
																		Protocol_RouteDot[Groups][Protocol_Send_cnt][5],//速度
																		Protocol_RouteDot[Groups][Protocol_Send_cnt][4],//半径
																		Protocol_RouteDot[Groups][Protocol_Send_cnt][6]);//动作
										
											
										if((Protocol_GetRoute&0x70) == 0x70)
										{
											Protocol_GetRoute = 0;
											Protocol_Send_cnt ++;//如果上位机已经接受，那么这个值增加
											
											if(Protocol_RouteDot[Groups][Protocol_Send_cnt-1][0] < Protocol_Send_cnt)//总航点等于当前航点时
											 {
												Protocol_Send_cnt= 1;
												Protocol_GetRoute = 0;
											 }
											
										}
							}
					}
			 Protocol_Send_Route = 0;
    }
		if(Protocol_Send_RouteInfo)
		{
			 Protocol_Send_RouteInfo= 0;
			 Protocol_T_RouteInfo(Protocol_CtrlMode,Protocol_AirplaneMode,Protocol_CurrentStatus,Protocol_LockStatus,Protocol_TurningStatus,Protocol_CurrentTarget,
			                      Protocol_AirSpeed,Protocol_WaitToFlight,Protocol_DiffrentDistance,Protocol_DiffrentAngle,Protocol_DiffrentHight,Protocol_DiffrentSpeed,
			                      Protocol_TargetRoll,Protocol_TargetPitch,Protocol_TargetYaw,Protocol_TargetHight,Protocol_UseSpeed,Protocol_wzSpeed,Protocol_BoardHight);	
    }
		
		if(Protocol_Send_TaskSet1)//起飞
		{
			 Protocol_Send_TaskSet1 = 0;
			 Protocol_T_TaskSet(1,0,0,0,0,0,0,0,0);
    }
		if(Protocol_Send_TaskSet2)//过渡
		{
			 Protocol_Send_TaskSet2 = 0;
			 Protocol_T_TaskSet(2,0,0,0,0,0,0,0,0);
    }
		if(Protocol_Send_TaskSet3)//盘旋
		{
			 Protocol_Send_TaskSet3 = 0;
			 Protocol_T_TaskSet(3,0,0,0,0,0,0,0,0);
    }
		if(Protocol_Send_TaskSet4)//巡航
		{
			 Protocol_Send_TaskSet4 = 0;
			 Protocol_T_TaskSet(4,0,0,0,0,0,0,0,0);
    }
		if(Protocol_Send_TaskSet5)//过渡
		{
			 Protocol_Send_TaskSet5 = 0;
			 Protocol_T_TaskSet(5,0,0,0,0,0,0,0,0);
    }
		if(Protocol_Send_TaskSet6)//降落
		{
			 Protocol_Send_TaskSet6 = 0;
			 Protocol_T_TaskSet(6,0,0,0,0,0,0,0,0);
    }
		if(Protocol_Send_TaskSet7)//关车
		{
			 Protocol_Send_TaskSet7 = 0;
			 Protocol_T_TaskSet(7,0,0,0,0,0,0,0,0);
    }
				
		
}
//===============Echo=======================
void Protocol_T_Echo(unsigned char Protocol_Echo_CMD1,
										 unsigned char Protocol_Echo_CMD2,
										 unsigned char Protocol_Echo_CMD3,
										 unsigned char Protocol_Echo_CMD4,
										 unsigned char Protocol_Echo_Error1,
										 unsigned char Protocol_Echo_Error2,
										 unsigned char Protocol_Echo_Error3,
										 unsigned char Protocol_Echo_Error4)
{
	   unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;  
	   u8 sum = 0,i = 0;
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x00;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============
	   data_to_send[data_cnt++] = Protocol_Echo_CMD1;
	   data_to_send[data_cnt++] = Protocol_Echo_CMD2;
	   data_to_send[data_cnt++] = Protocol_Echo_CMD3;
	   data_to_send[data_cnt++] = Protocol_Echo_CMD4;
	   data_to_send[data_cnt++] = Protocol_Echo_Error1;
	   data_to_send[data_cnt++] = Protocol_Echo_Error2;
		 data_to_send[data_cnt++] = Protocol_Echo_Error3;
	   data_to_send[data_cnt++] = Protocol_Echo_Error4;
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);
}
//==============version=====================
void Protocol_T_Version(float Protocol_HardwareVersion,
                        float Protocol_SoftwareVersion,
                        float Protocol_ProtocolVersion,
                        float Protocol_AirplaneVersion)
{
	   unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;   
	   u8 sum = 0,i = 0;
     union {unsigned char B[2]; unsigned short D;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x01;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============
     src.D = Protocol_HardwareVersion * 100;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];

     src.D = Protocol_SoftwareVersion * 100;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];

     src.D = Protocol_ProtocolVersion * 100;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_AirplaneVersion * 100;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);
}
//==============Attiude==================
void Protocol_T_Attiude(float Protocol_ACC_X, float Protocol_ACC_Y,float Protocol_ACC_Z,
                        float Protocol_GYRO_X, float Protocol_GYRO_Y,float Protocol_GYRO_Z,
                        float Protocol_MAG_X, float Protocol_MAG_Y,float Protocol_MAG_Z,
                        float Protocol_ROL, float Protocol_PIT,float Protocol_YAW)
{
	   unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;   
	   u8 sum = 0,i = 0;
     union {unsigned char B[2]; unsigned short D;short T;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;//0
	   data_to_send[data_cnt++] = 0xAA;//1
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x02;//2
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;//3
	   //==========data===============
     src.D = Protocol_ACC_X*100;
	   data_to_send[data_cnt++] = src.B[0];//4
	   data_to_send[data_cnt++] = src.B[1];//5

     src.D = Protocol_ACC_Y*100;
	   data_to_send[data_cnt++] = src.B[0];//6
	   data_to_send[data_cnt++] = src.B[1];//7

     src.D = Protocol_ACC_Z*100;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_GYRO_X*100;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_GYRO_Y*100;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_GYRO_Z*100;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_MAG_X*100;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_MAG_Y*100;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_MAG_Z*100;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];//21
		 
		 src.T = Protocol_ROL * 100;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.T = Protocol_PIT * 100;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.T = Protocol_YAW * 10;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];//27
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;//28
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);
}
//=======================Signal============================
void Protocol_T_Signal(int16_t Protocol_ROL, int16_t Protocol_PIT, int16_t Protocol_THR, int16_t Protocol_YAW,
                       int16_t Protocol_AUX1,int16_t Protocol_AUX2,int16_t Protocol_AUX3,int16_t Protocol_AUX4,
                       int16_t Protocol_AUX5,int16_t Protocol_AUX6,int16_t Protocol_PWM1,int16_t Protocol_PWM2,
                       int16_t Protocol_PWM3,int16_t Protocol_PWM4,int16_t Protocol_PWM5,int16_t Protocol_PWM6,
                       int16_t Protocol_PWM7,int16_t Protocol_PWM8,int16_t Protocol_PWM9,int16_t Protocol_PWM10,
                       int16_t Protocol_PWM11,int16_t Protocol_PWM12)
{
     unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;   
	   u8 sum = 0,i = 0;
     union {unsigned char B[2]; signed short D;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x03;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============
     src.D = Protocol_ROL;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];

     src.D = Protocol_PIT;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];

     src.D = Protocol_THR;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_YAW;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_AUX1;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_AUX2;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_AUX3;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_AUX4;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_AUX5;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_AUX6;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM1;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM2;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM3;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM4;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM5;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM6;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM7;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM8;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM9;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM10;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM11;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D = Protocol_PWM12;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);
}
//=======================Status=============================
void Protocol_T_Status(float Protocol_ALT_U,   float Protocol_ALT_P,
                       float Protocol_Votage1, float Protocol_Votage2,
                       float Protocol_Current1,float Protocol_Current2)
{
	   unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;   
	   u8 sum = 0,i = 0;
     union {unsigned char B[4]; float F;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x04;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============
     src.F = Protocol_ALT_U;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];

     src.F = Protocol_ALT_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];

     src.F = Protocol_Votage1;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
		 src.F = Protocol_Votage2;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
		 src.F = Protocol_Current1;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
		 src.F = Protocol_Current2;
		 data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);
}
//======================gps============================
void Protocol_T_GPS1(uint8_t Protocol_GPSStatus,uint8_t Protocol_StarNum,
                     double Protocol_Latitude,double Protocol_Longitude,
                     float Protocol_Altitude ,float Protocol_Speed,
                     float Protocol_Course)
{
	   unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;   
	   u8 sum = 0,i = 0;
     union {unsigned char B[8]; signed short D[4];float F[2];double DD;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x10;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============
	   data_to_send[data_cnt++] = Protocol_GPSStatus;
	   data_to_send[data_cnt++] = Protocol_StarNum;

     src.DD = Protocol_Latitude;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     data_to_send[data_cnt++] = src.B[4];
	   data_to_send[data_cnt++] = src.B[5];
     data_to_send[data_cnt++] = src.B[6];
	   data_to_send[data_cnt++] = src.B[7];

     src.DD = Protocol_Longitude;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 data_to_send[data_cnt++] = src.B[4];
	   data_to_send[data_cnt++] = src.B[5];
     data_to_send[data_cnt++] = src.B[6];
	   data_to_send[data_cnt++] = src.B[7];
		 
     src.F[0] = Protocol_Altitude;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F[0] = Protocol_Speed;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F[0] = Protocol_Course;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);	   
}


void Protocol_T_PID1(float Protocol_PID1_P,float Protocol_PID1_I,float Protocol_PID1_D,
                     float Protocol_PID2_P,float Protocol_PID2_I,float Protocol_PID2_D,
                     float Protocol_PID3_P,float Protocol_PID3_I,float Protocol_PID3_D,
                     float Protocol_PID4_P,float Protocol_PID4_I,float Protocol_PID4_D,
                     float Protocol_PID5_P,float Protocol_PID5_I,float Protocol_PID5_D,
                     float Protocol_PID6_P,float Protocol_PID6_I,float Protocol_PID6_D)
{
	   unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;   
	   u8 sum = 0,i = 0;
     union {unsigned char B[4]; int16_t D[2];float F;}src;
// 	   union {unsigned char B[2]; int16_t D;}src;
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x21;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============

     src.F = Protocol_PID1_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID1_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID1_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_PID2_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID2_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID2_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_PID3_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID3_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID3_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID4_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID4_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID4_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID5_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID5_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID5_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID6_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID6_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID6_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);	
}
void Protocol_T_PID2(float Protocol_PID1_P,float Protocol_PID1_I,float Protocol_PID1_D,
                     float Protocol_PID2_P,float Protocol_PID2_I,float Protocol_PID2_D,
                     float Protocol_PID3_P,float Protocol_PID3_I,float Protocol_PID3_D,
                     float Protocol_PID4_P,float Protocol_PID4_I,float Protocol_PID4_D,
                     float Protocol_PID5_P,float Protocol_PID5_I,float Protocol_PID5_D,
                     float Protocol_PID6_P,float Protocol_PID6_I,float Protocol_PID6_D)
{
		 unsigned char data_to_send[150];
	   unsigned char data_cnt = 0; 
     u8 sum = 0,i = 0;  
	   union {unsigned char B[4]; int16_t D[2];float F;}src;
//      union {unsigned char B[2]; int16_t D;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x22;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============

     src.F = Protocol_PID1_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID1_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID1_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_PID2_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID2_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID2_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_PID3_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID3_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID3_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID4_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID4_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID4_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID5_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID5_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID5_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID6_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID6_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID6_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);	
}
void Protocol_T_PID3(float Protocol_PID1_P,float Protocol_PID1_I,float Protocol_PID1_D,
                     float Protocol_PID2_P,float Protocol_PID2_I,float Protocol_PID2_D,
                     float Protocol_PID3_P,float Protocol_PID3_I,float Protocol_PID3_D,
                     float Protocol_PID4_P,float Protocol_PID4_I,float Protocol_PID4_D,
                     float Protocol_PID5_P,float Protocol_PID5_I,float Protocol_PID5_D,
                     float Protocol_PID6_P,float Protocol_PID6_I,float Protocol_PID6_D)
{
		 unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;   
	   u8 sum = 0,i = 0;
	   union {unsigned char B[4]; int16_t D[2];float F;}src;
//      union {unsigned char B[2]; int16_t D;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x23;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============

     src.F = Protocol_PID1_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID1_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID1_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_PID2_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID2_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID2_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_PID3_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID3_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID3_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID4_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID4_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID4_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID5_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID5_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID5_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID6_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID6_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID6_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);	
}
void Protocol_T_PID4(float Protocol_PID1_P,float Protocol_PID1_I,float Protocol_PID1_D,
                     float Protocol_PID2_P,float Protocol_PID2_I,float Protocol_PID2_D,
                     float Protocol_PID3_P,float Protocol_PID3_I,float Protocol_PID3_D,
                     float Protocol_PID4_P,float Protocol_PID4_I,float Protocol_PID4_D,
                     float Protocol_PID5_P,float Protocol_PID5_I,float Protocol_PID5_D,
                     float Protocol_PID6_P,float Protocol_PID6_I,float Protocol_PID6_D)
{
		 unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;  
     u8 sum = 0,i = 0;	
	   union {unsigned char B[4]; int16_t D[2];float F;}src;
//      union {unsigned char B[2]; int16_t D;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x24;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============

     src.F = Protocol_PID1_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID1_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID1_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_PID2_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID2_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID2_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_PID3_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID3_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID3_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID4_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID4_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID4_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID5_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID5_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID5_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 		 
     src.F = Protocol_PID6_P;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID6_I;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     src.F = Protocol_PID6_D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);	
}
void Protocol_T_Route(uint8_t Groups,uint16_t Protocol_TotalPoint,uint16_t Protocol_CurrentPoint,
                      float Protocol_Latitude,float Protocol_Longitude,
                      float Protocol_Altitude,float Protocol_Radius,
                      float Protocol_Velocity,uint8_t Protocol_Action)
{
	   unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;   
     union {unsigned char B[8]; signed short D[2];float F[2]; double DD;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x41;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============

     data_to_send[data_cnt++] = Groups;
 
     src.D[0] = Protocol_TotalPoint;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];

     src.D[0] = Protocol_CurrentPoint;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];

     src.DD = Protocol_Latitude;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 data_to_send[data_cnt++] = src.B[4];
	   data_to_send[data_cnt++] = src.B[5];
     data_to_send[data_cnt++] = src.B[6];
	   data_to_send[data_cnt++] = src.B[7];

     src.DD = Protocol_Longitude;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 data_to_send[data_cnt++] = src.B[4];
	   data_to_send[data_cnt++] = src.B[5];
     data_to_send[data_cnt++] = src.B[6];
	   data_to_send[data_cnt++] = src.B[7];
		 
		 
     src.D[0] = Protocol_Altitude;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
     src.D[0] = Protocol_Radius;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
     src.D[0] = Protocol_Velocity;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.B[0] = Protocol_Action;
	   data_to_send[data_cnt++] = src.B[0];
		 
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 +1 ;
		 //==========SUM================
		 src.D[0] = SendCheckCRC(data_to_send);
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);	
}
void Protocol_T_RouteInfo(uint8_t Protocol_CtrlMode,uint8_t Protocol_AirplaneMode,uint8_t Protocol_CurrentStatus,
                          uint8_t Protocol_LockStatus,uint8_t Protocol_TurningStatus,uint16_t Protocol_CurrentTarget,
                          float Protocol_AirSpeed,float Protocol_WaitToFlight,float Protocol_DiffrentDistance,
                          float Protocol_DiffrentAngle,float Protocol_DiffrentHight,float Protocol_DiffrentSpeed,
                          uint16_t Protocol_TargetRoll,uint16_t Protocol_TargetPitch,uint16_t Protocol_TargetYaw,
                          uint16_t Protocol_TargetHight,float Protocol_UseSpeed,float Protocol_wzSpeed,float Protocol_BoardHight)
{
	   unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;   
	   u8 sum = 0,i = 0;
     union {unsigned char B[4]; signed short D[2];float F;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x42;
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============
	   data_to_send[data_cnt++] = Protocol_CtrlMode;
	   data_to_send[data_cnt++] = Protocol_AirplaneMode;
	   data_to_send[data_cnt++] = Protocol_CurrentStatus;
	   data_to_send[data_cnt++] = Protocol_LockStatus;
	   data_to_send[data_cnt++] = Protocol_TurningStatus;

     src.D[0] = Protocol_CurrentTarget;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];

     src.F = Protocol_AirSpeed;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];

     src.F = Protocol_WaitToFlight;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_DiffrentDistance;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_DiffrentAngle;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.F = Protocol_DiffrentHight;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
		 src.F = Protocol_DiffrentSpeed;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
     src.D[0] = Protocol_TargetRoll;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D[0] = Protocol_TargetPitch;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D[0] = Protocol_TargetYaw;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.D[0] = Protocol_TargetHight;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 
		 src.F = Protocol_UseSpeed;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
		 src.F = Protocol_wzSpeed;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
		 src.F = Protocol_BoardHight;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
		 data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];
		 
		 
     //==========LEN================
	   data_to_send[3] =data_cnt - 4 ;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);	
}


void Protocol_T_TaskSet(u8 ID,float A,float B,float C,float D,float E,float F,float G,float H)
{
	   unsigned char data_to_send[150];
	   unsigned char data_cnt = 0;   
	   u8 sum = 0,i = 0;
     union {unsigned char B[4];float F;}src;
	
	   //==========帧头=========
	   data_to_send[data_cnt++] = 0xAA;
	   data_to_send[data_cnt++] = 0xAA;
	   //==========ID===============
	   data_to_send[data_cnt++] = 0x7F + ID;//id = 1~7
	   //==========LEN================
	   data_to_send[data_cnt++] = 0x00;
	   //==========data===============

     src.F = A;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];

     src.F = B;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];

     src.F = C;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];

     src.F = D;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];	 
     src.F = E;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];

     src.F = F;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];

     src.F = G;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];

     src.F = H;
	   data_to_send[data_cnt++] = src.B[0];
	   data_to_send[data_cnt++] = src.B[1];
     data_to_send[data_cnt++] = src.B[2];
	   data_to_send[data_cnt++] = src.B[3];

		 
     //==========LEN================
	   data_to_send[3] =data_cnt - 4;
		 //==========SUM================
	   for(i=0;i<data_cnt;i++)
		 sum += data_to_send[i];
	   data_to_send[data_cnt++]=sum;
		 //=========Send=================
		 Protocol_Send(data_to_send,data_cnt);	
}





//=====================Receive================================


uint16_t CRC16_TABLE[] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};


unsigned char RPCheckCRC(unsigned char *data)
{
    uint16_t crc = 0;
    uint16_t Dcrc = 0;
	  uint8_t LEN =  data[3]+4-1;
		int i = 0;
		unsigned char temp;

	
	  crc = 0;
	  Dcrc = 0;
	
		while (LEN-- != 0)
		{
				temp = (unsigned char)(crc >> 8); 
				crc <<= 8;
				crc ^= CRC16_TABLE[temp ^ data[i]]; 
				i++;
		}
		
		LEN =  data[3]+4;
		Dcrc = (uint16_t)((data[LEN]<<8) | data[LEN-1]); 
		if(Dcrc == crc )
		{
			 return 1;
    }
		return 0;
}

uint16_t SendCheckCRC(unsigned char *data)
{
     uint16_t crc = 0;
     int32_t i = 0;
     uint8_t temp = 0;
     uint8_t len = (uint8_t)(data[3] + 4 - 1);
    while (len-- != 0)
    {
            temp = (uint8_t)(crc >> 8);
            crc <<= 8;
            crc ^= CRC16_TABLE[temp ^ (uint8_t)data[i]];
            i++;
    }
    return crc;
}




u8 ProRxBuffer[100];
u8 ProState = 0;
void Protocol_Prepare(u8 data)
{	
	static u8 _data_len = 0,_data_cnt = 0;
	if(ProState==0&&data==0xEB)//第一个字
	{
			   ProState=1;
			   ProRxBuffer[0]=data;
	}
	else if(ProState==1&&data==0x90)//第二个字
	{
			  ProState=2;   
			  ProRxBuffer[1]=data;
	}
	else if(ProState==2)//第三个字id号
	{
		    ProState=3;
		    ProRxBuffer[2]=data;		
	}
	else if(ProState==3)//第四个字,长度小于64
	{
		  _data_len = data + 1;
		  _data_cnt = 4;
		  ProState=4;
		  ProRxBuffer[3]=data;//LEN
	}
	else if(ProState==4&&_data_len>0)
	{
		_data_len--;
		ProRxBuffer[_data_cnt++]=data;
		if(ProRxBuffer[2] != 0x41)
		{
				if((_data_len==0)&&(CheckSum(ProRxBuffer) == ProRxBuffer[_data_cnt - 1]))//
				{
					 //解码
						switch(ProRxBuffer[2])//ID 
							 {   
										case 0x00:{ Protocol_R_Beat(ProRxBuffer);     }break;//Beat
										case 0x01:{ Protocol_R_CMD1(ProRxBuffer);     }break;//CMD1
										case 0x02:{ Protocol_R_CMD2(ProRxBuffer);     }break;//CMD2
										case 0x03:{ Protocol_R_CMD3(ProRxBuffer);     }break;//CMD3
										case 0x04:{ Protocol_R_CMD4(ProRxBuffer);     }break;//CMD4
										case 0x05:{ Protocol_R_CMD5(ProRxBuffer);     }break;//CMD5
										case 0x06:{ Protocol_R_CMD6(ProRxBuffer);     }break;//CMD6
										case 0x11:{ Protocol_R_Adjust(ProRxBuffer);   }break;//Adjust
										case 0x21:{ Protocol_R_PID1(ProRxBuffer);     }break;//PID1
										case 0x22:{ Protocol_R_PID2(ProRxBuffer);     }break;//PID2
										case 0x23:{ Protocol_R_PID3(ProRxBuffer);     }break;//PID3
										case 0x24:{ Protocol_R_PID4(ProRxBuffer);     }break;//PID4
										case 0x80:{ Protocol_R_TaskTakeOffSetting(ProRxBuffer);          }break;//Task
										case 0x81:{ Protocol_R_TaskTakeOffToTravelSetting(ProRxBuffer);  }break;
										case 0x82:{ Protocol_R_TaskCircleSetting(ProRxBuffer);           }break;
										case 0x83:{ Protocol_R_TaskTravelSetting(ProRxBuffer);           }break;
										case 0x84:{ Protocol_R_TaskTravelToLandingSetting(ProRxBuffer);  }break;
										case 0x85:{ Protocol_R_TaskLandingSetting(ProRxBuffer);          }break;
										case 0x86:{ Protocol_R_TaskCutOffSetting(ProRxBuffer);           }break;
									
										//case 0x41:{ Protocol_R_GetRoute(ProRxBuffer); }break;//GetRoute 这个是CRC校验
										default  :{ ProState=0;}break;//unfine
							 }		 
					ProState = 0;
				}
			}
			else
			{
				 if((_data_len==0)&&RPCheckCRC(ProRxBuffer))//
				{
					 //解码
						switch(ProRxBuffer[2])//ID 
							 {   
										case 0x41:{ Protocol_R_GetRoute(ProRxBuffer); }break;//GetRoute 这个是CRC校验
										default  :{ ProState=0;}break;//unfine
							 }		 
					ProState = 0;
				}
			}	
	}
	else
		ProState = 0;

}

unsigned char CheckSum(u8 *buff)
{   
	  unsigned char i,ReturnVelue;
	  uint16_t sum = 0;
	  for(i = 0;i<(buff[3]+4);i++)
	  {
			  sum += buff[i]; 
    }
		ReturnVelue = (sum&0xff);
		
		return ReturnVelue;
}

//======================Beat================================
unsigned char HeartBeat;
void Protocol_R_Beat(u8 *data)//00
{
	
	 //if do not recieve beat in 20sec ,it will be out of the distance of Station
	 HeartBeat  = data[8];
}

//======================CMD1=====================================
unsigned char Protocol_GetVerion; 
void Protocol_R_CMD1(u8 *data)//01
{
	  Protocol_GetVerion = data[4];
	  if(Protocol_GetVerion == 0x01) Protocol_Send_Version = 1;
}
//======================CMD2=====================================
unsigned char Protocol_GetPID1,
              Protocol_GetPID2,
              Protocol_GetPID3,
              Protocol_GetPID4,
              Protocol_GetPolarity,
							Protocol_SenserCalibrate,
							Protocol_Get1,
							Protocol_Get2;
void Protocol_R_CMD2(u8 *data)//02
{
	  Protocol_GetPID1     = data[4];
	  Protocol_GetPID2     = data[5];
	  Protocol_GetPID3     = data[6];
	  Protocol_GetPID4     = data[7];
	
	  if(Protocol_GetPID1 == 0x01)  Protocol_Send_PID1 = 1;
	  if(Protocol_GetPID2 == 0x01)  Protocol_Send_PID2 = 1;
	  if(Protocol_GetPID3 == 0x01)  Protocol_Send_PID3 = 1;
	  if(Protocol_GetPID4 == 0x01)  Protocol_Send_PID4 = 1;
	
	  Protocol_GetPolarity     = data[8];
	  Protocol_SenserCalibrate = data[9];
	  Protocol_Get1            = data[10];
	  Protocol_Get2            = data[11];
	          
	  
	
	  if(Protocol_SenserCalibrate == 8)
		{
// 			  mpu6050.Gyro_CALIBRATE = 1;
    }
		else if(Protocol_SenserCalibrate == 9)
		{
// 			  mpu6050.Acc_CALIBRATE = 1;
    }
		else if(Protocol_SenserCalibrate == 10)
		{
// 			  Mag_CALIBRATED = 1;
    }
		else if(Protocol_SenserCalibrate == 11)
		{
// 			  MS5611_Reset();
    }
		else if(Protocol_SenserCalibrate == 12)
		{
			  
    }
			
	
	
}
//======================CMD3=====================================
unsigned char  
              Protocol_GetRoute,
              Protocol_ChangePoint;
void Protocol_R_CMD3(u8 *data)//03
{
	  Protocol_ChangePoint     = data[4];
	
	  Protocol_GetRoute        = data[9];
	
	
	
	
}
//======================CMD4=====================================
unsigned char Protocol_TakeOff,
              Protocol_StartTravel,
              Protocol_StartRoute,
              Protocol_Landing,
              Protocol_BackHome,
              Protocol_Lock,
              Protocol_Action;//0~255
void Protocol_R_CMD4(u8 *data)//04
{
	  Protocol_TakeOff     = data[4];
	  Protocol_StartTravel = data[5];
	  Protocol_StartRoute  = data[6];
	  Protocol_Landing     = data[7];
	  Protocol_BackHome    = data[8];
	  Protocol_Lock        = data[10];
	  Protocol_Action      = data[11];
	

		
}
//======================CMD5====================
float Protocol_R_TakeoffHight,Protocol_R_StartRouteHight;
void Protocol_R_CMD5(u8 *data)//05
{
	  union {unsigned char B[4]; float F;}src;

    src.B[0] = data[4];
	  src.B[1] = data[5];
	  src.B[2] = data[6];
	  src.B[3] = data[7];
    Protocol_R_TakeoffHight = src.F;   

	  src.B[0] = data[8];
	  src.B[1] = data[9];
	  src.B[2] = data[10];
	  src.B[3] = data[11];
    Protocol_R_StartRouteHight = src.F;
}


u8 Protocol_R_ReadTakeOff,
   Protocol_R_ReadTakeOffToTravel,
   Protocol_R_ReadCircle,
   Protocol_R_ReadTravel,
   Protocol_R_ReadTravelTolanding,
   Protocol_R_ReadBackHome,
   Protocol_R_ReadLanding,
   Protocol_R_ReadCutOff;
   
void Protocol_R_CMD6(u8 *data)
{
	 Protocol_R_ReadTakeOff           = data[4];
   Protocol_R_ReadTakeOffToTravel   = data[5];
   Protocol_R_ReadCircle            = data[6];
   Protocol_R_ReadTravel            = data[7];
   Protocol_R_ReadTravelTolanding   = data[8];
   Protocol_R_ReadBackHome          = data[9];
   Protocol_R_ReadLanding           = data[10];
   Protocol_R_ReadCutOff            = data[11];
	
	
		Protocol_Send_TaskSet1 = Protocol_R_ReadTakeOff;
		Protocol_Send_TaskSet2 = Protocol_R_ReadTakeOffToTravel;
		Protocol_Send_TaskSet3 = Protocol_R_ReadCircle;
		Protocol_Send_TaskSet4 = Protocol_R_ReadTravel;
		Protocol_Send_TaskSet5 = Protocol_R_ReadTravelTolanding;
	
		Protocol_Send_TaskSet6 = Protocol_R_ReadLanding;
		Protocol_Send_TaskSet7 = Protocol_R_ReadCutOff;
}



//======================Adjust==============================
int16_t Protocol_AdjustROL,
        Protocol_AdjustPIT,
        Protocol_AdjustTHR,
        Protocol_AdjustYAW,
        Protocol_AdjustAUX1,
        Protocol_AdjustAUX2,
        Protocol_AdjustAUX3,
        Protocol_AdjustAUX4,
        Protocol_AdjustAUX5,
        Protocol_AdjustAUX6,
        Protocol_AdjustPWM1,
        Protocol_AdjustPWM2,
        Protocol_AdjustPWM3,
        Protocol_AdjustPWM4,
        Protocol_AdjustPWM5,
        Protocol_AdjustPWM6,
        Protocol_AdjustPWM7,
        Protocol_AdjustPWM8,
        Protocol_AdjustPWM9,
				Protocol_AdjustPWM10,
				Protocol_AdjustPWM11,
				Protocol_AdjustPWM12;
void Protocol_R_Adjust(u8 *data)//11
{
	  union {unsigned char B[2]; signed short D;}src;

    src.B[0] = data[4];   src.B[1] = data[5];      Protocol_AdjustROL = src.D;
    src.B[0] = data[6];   src.B[1] = data[7];      Protocol_AdjustPIT = src.D;
    src.B[0] = data[8];   src.B[1] = data[9];      Protocol_AdjustTHR = src.D;
    src.B[0] = data[10];  src.B[1] = data[11];     Protocol_AdjustYAW = src.D;
    src.B[0] = data[12];  src.B[1] = data[13];     Protocol_AdjustAUX1 = src.D;
    src.B[0] = data[14];  src.B[1] = data[15];     Protocol_AdjustAUX2 = src.D;
    src.B[0] = data[16];  src.B[1] = data[17];     Protocol_AdjustAUX3 = src.D;
    src.B[0] = data[18];  src.B[1] = data[19];     Protocol_AdjustAUX4 = src.D;
    src.B[0] = data[20];  src.B[1] = data[21];     Protocol_AdjustAUX5 = src.D;
    src.B[0] = data[22];  src.B[1] = data[23];     Protocol_AdjustAUX6 = src.D;
    src.B[0] = data[24];  src.B[1] = data[25];     Protocol_AdjustPWM1 = src.D;
    src.B[0] = data[26];  src.B[1] = data[27];     Protocol_AdjustPWM2 = src.D;
    src.B[0] = data[28];  src.B[1] = data[29];     Protocol_AdjustPWM3 = src.D;
    src.B[0] = data[30];  src.B[1] = data[31];     Protocol_AdjustPWM4 = src.D;
    src.B[0] = data[32];  src.B[1] = data[33];     Protocol_AdjustPWM5 = src.D;
    src.B[0] = data[34];  src.B[1] = data[35];     Protocol_AdjustPWM6 = src.D;
    src.B[0] = data[36];  src.B[1] = data[37];     Protocol_AdjustPWM7 = src.D;
    src.B[0] = data[38];  src.B[1] = data[39];     Protocol_AdjustPWM8 = src.D;
    src.B[0] = data[40];  src.B[1] = data[41];     Protocol_AdjustPWM9 = src.D;
    src.B[0] = data[42];  src.B[1] = data[43];     Protocol_AdjustPWM10 = src.D;
		src.B[0] = data[44];  src.B[1] = data[45];     Protocol_AdjustPWM11 = src.D;
    src.B[0] = data[46];  src.B[1] = data[47];     Protocol_AdjustPWM12 = src.D;
}


//======================PID1==================================  
void Protocol_R_PID1(u8 *data)//21
{
	  union {unsigned char B[4]; float F;}src;

//     src.B[0] = data[4];   src.B[1] = data[5];   src.B[2] = data[6];   src.B[3] = data[7];    ctrl_1.PID[PIDROLL].kp  = src.F;
//     src.B[0] = data[8];   src.B[1] = data[9];   src.B[2] = data[10];  src.B[3] = data[11];   ctrl_1.PID[PIDROLL].ki  = src.F;
//     src.B[0] = data[12];  src.B[1] = data[13];  src.B[2] = data[14];  src.B[3] = data[15];   ctrl_1.PID[PIDROLL].kd  = src.F;
//     src.B[0] = data[16];  src.B[1] = data[17];  src.B[2] = data[18];  src.B[3] = data[19];   ctrl_1.PID[PIDPITCH].kp = src.F;
//     src.B[0] = data[20];  src.B[1] = data[21];  src.B[2] = data[22];  src.B[3] = data[23];   ctrl_1.PID[PIDPITCH].ki = src.F;
//     src.B[0] = data[24];  src.B[1] = data[25];  src.B[2] = data[26];  src.B[3] = data[27];   ctrl_1.PID[PIDPITCH].kd = src.F;
//     src.B[0] = data[28];  src.B[1] = data[29];  src.B[2] = data[30];  src.B[3] = data[31];   ctrl_1.PID[PIDYAW].kp   = src.F;
//     src.B[0] = data[32];  src.B[1] = data[33];  src.B[2] = data[34];  src.B[3] = data[35];   ctrl_1.PID[PIDYAW].ki   = src.F;
//     src.B[0] = data[36];  src.B[1] = data[37];  src.B[2] = data[38];  src.B[3] = data[39];   ctrl_1.PID[PIDYAW].kd   = src.F;
//     src.B[0] = data[40];  src.B[1] = data[41];  src.B[2] = data[42];  src.B[3] = data[43];   ctrl_2.PID[PIDROLL].kp  = src.F;
//     src.B[0] = data[44];  src.B[1] = data[45];  src.B[2] = data[46];  src.B[3] = data[47];   ctrl_2.PID[PIDROLL].ki  = src.F;
//     src.B[0] = data[48];  src.B[1] = data[49];  src.B[2] = data[50];  src.B[3] = data[51];   ctrl_2.PID[PIDROLL].kd  = src.F;
//     src.B[0] = data[52];  src.B[1] = data[53];  src.B[2] = data[54];  src.B[3] = data[55];   ctrl_2.PID[PIDPITCH].kp = src.F;
//     src.B[0] = data[56];  src.B[1] = data[57];  src.B[2] = data[58];  src.B[3] = data[59];   ctrl_2.PID[PIDPITCH].ki = src.F;
//     src.B[0] = data[60];  src.B[1] = data[61];  src.B[2] = data[62];  src.B[3] = data[63];   ctrl_2.PID[PIDPITCH].kd = src.F;
//     src.B[0] = data[64];  src.B[1] = data[65];  src.B[2] = data[66];  src.B[3] = data[67];   ctrl_2.PID[PIDYAW].kp   = src.F;
//     src.B[0] = data[68];  src.B[1] = data[69];  src.B[2] = data[70];  src.B[3] = data[71];   ctrl_2.PID[PIDYAW].ki   = src.F;
//     src.B[0] = data[72];  src.B[1] = data[73];  src.B[2] = data[74];  src.B[3] = data[75];   ctrl_2.PID[PIDYAW].kd   = src.F;

//      if( !fly_ready )
// 		{
// 			Param_SavePID();
// 		}

		
}
//======================PID2==================================  
void Protocol_R_PID2(u8 *data)//22
{
	  union {unsigned char B[4]; float F;}src;

//     src.B[0] = data[4];   src.B[1] = data[5];   src.B[2] = data[6];   src.B[3] = data[7];    ctrl_Fixwing_I.PID[PIDROLL].kp  = src.F;
//     src.B[0] = data[8];   src.B[1] = data[9];   src.B[2] = data[10];  src.B[3] = data[11];   ctrl_Fixwing_I.PID[PIDROLL].ki  = src.F;
//     src.B[0] = data[12];  src.B[1] = data[13];  src.B[2] = data[14];  src.B[3] = data[15];   ctrl_Fixwing_I.PID[PIDROLL].kd  = src.F;
//     src.B[0] = data[16];  src.B[1] = data[17];  src.B[2] = data[18];  src.B[3] = data[19];   ctrl_Fixwing_I.PID[PIDPITCH].kp = src.F;
//     src.B[0] = data[20];  src.B[1] = data[21];  src.B[2] = data[22];  src.B[3] = data[23];   ctrl_Fixwing_I.PID[PIDPITCH].ki = src.F;
//     src.B[0] = data[24];  src.B[1] = data[25];  src.B[2] = data[26];  src.B[3] = data[27];   ctrl_Fixwing_I.PID[PIDPITCH].kd = src.F;
//     src.B[0] = data[28];  src.B[1] = data[29];  src.B[2] = data[30];  src.B[3] = data[31];   ctrl_Fixwing_I.PID[PIDYAW].kp   = src.F;
//     src.B[0] = data[32];  src.B[1] = data[33];  src.B[2] = data[34];  src.B[3] = data[35];   ctrl_Fixwing_I.PID[PIDYAW].ki   = src.F;
//     src.B[0] = data[36];  src.B[1] = data[37];  src.B[2] = data[38];  src.B[3] = data[39];   ctrl_Fixwing_I.PID[PIDYAW].kd   = src.F;
//     src.B[0] = data[40];  src.B[1] = data[41];  src.B[2] = data[42];  src.B[3] = data[43];   ctrl_Fixwing_E.PID[PIDROLL].kp  = src.F;
//     src.B[0] = data[44];  src.B[1] = data[45];  src.B[2] = data[46];  src.B[3] = data[47];   ctrl_Fixwing_E.PID[PIDROLL].ki  = src.F;
//     src.B[0] = data[48];  src.B[1] = data[49];  src.B[2] = data[50];  src.B[3] = data[51];   ctrl_Fixwing_E.PID[PIDROLL].kd  = src.F;
//     src.B[0] = data[52];  src.B[1] = data[53];  src.B[2] = data[54];  src.B[3] = data[55];   ctrl_Fixwing_E.PID[PIDPITCH].kp = src.F;
//     src.B[0] = data[56];  src.B[1] = data[57];  src.B[2] = data[58];  src.B[3] = data[59];   ctrl_Fixwing_E.PID[PIDPITCH].ki = src.F;
//     src.B[0] = data[60];  src.B[1] = data[61];  src.B[2] = data[62];  src.B[3] = data[63];   ctrl_Fixwing_E.PID[PIDPITCH].kd = src.F;
//     src.B[0] = data[64];  src.B[1] = data[65];  src.B[2] = data[66];  src.B[3] = data[67];   ctrl_Fixwing_E.PID[PIDYAW].kp   = src.F;
//     src.B[0] = data[68];  src.B[1] = data[69];  src.B[2] = data[70];  src.B[3] = data[71];   ctrl_Fixwing_E.PID[PIDYAW].ki   = src.F;
//     src.B[0] = data[72];  src.B[1] = data[73];  src.B[2] = data[74];  src.B[3] = data[75];   ctrl_Fixwing_E.PID[PIDYAW].kd   = src.F;

//     if( !fly_ready )
// 		{
// 			Param_SavePID();
// 		}

}
//======================PID3==================================  
void Protocol_R_PID3(u8 *data)//23
{
	  union {unsigned char B[4]; float F;}src;

//     src.B[0] = data[4];   src.B[1] = data[5];   src.B[2] = data[6];   src.B[3] = data[7];    AxisSpeed_x.PID.kp    = src.F;
//     src.B[0] = data[8];   src.B[1] = data[9];   src.B[2] = data[10];  src.B[3] = data[11];   AxisSpeed_x.PID.ki    = src.F;
//     src.B[0] = data[12];  src.B[1] = data[13];  src.B[2] = data[14];  src.B[3] = data[15];   AxisSpeed_x.PID.kd    = src.F;
//     src.B[0] = data[16];  src.B[1] = data[17];  src.B[2] = data[18];  src.B[3] = data[19];   AxisSpeed_y.PID.kp    = src.F;
//     src.B[0] = data[20];  src.B[1] = data[21];  src.B[2] = data[22];  src.B[3] = data[23];   AxisSpeed_y.PID.ki    = src.F;
//     src.B[0] = data[24];  src.B[1] = data[25];  src.B[2] = data[26];  src.B[3] = data[27];   AxisSpeed_y.PID.kd    = src.F;
//     src.B[0] = data[28];  src.B[1] = data[29];  src.B[2] = data[30];  src.B[3] = data[31];   AxisSpeed_z.PID.kp    = src.F;
//     src.B[0] = data[32];  src.B[1] = data[33];  src.B[2] = data[34];  src.B[3] = data[35];   AxisSpeed_z.PID.ki    = src.F;
//     src.B[0] = data[36];  src.B[1] = data[37];  src.B[2] = data[38];  src.B[3] = data[39];   AxisSpeed_z.PID.kd    = src.F;
//     src.B[0] = data[40];  src.B[1] = data[41];  src.B[2] = data[42];  src.B[3] = data[43];   AxisDistance_x.PID.kp = src.F;
//     src.B[0] = data[44];  src.B[1] = data[45];  src.B[2] = data[46];  src.B[3] = data[47];   AxisDistance_x.PID.ki = src.F;
//     src.B[0] = data[48];  src.B[1] = data[49];  src.B[2] = data[50];  src.B[3] = data[51];   AxisDistance_x.PID.kd = src.F;
//     src.B[0] = data[52];  src.B[1] = data[53];  src.B[2] = data[54];  src.B[3] = data[55];   AxisDistance_y.PID.kp = src.F;
//     src.B[0] = data[56];  src.B[1] = data[57];  src.B[2] = data[58];  src.B[3] = data[59];   AxisDistance_y.PID.ki = src.F;
//     src.B[0] = data[60];  src.B[1] = data[61];  src.B[2] = data[62];  src.B[3] = data[63];   AxisDistance_y.PID.kd = src.F;
//     src.B[0] = data[64];  src.B[1] = data[65];  src.B[2] = data[66];  src.B[3] = data[67];   AxisDistance_z.PID.kp = src.F;
//     src.B[0] = data[68];  src.B[1] = data[69];  src.B[2] = data[70];  src.B[3] = data[71];   AxisDistance_z.PID.ki = src.F;
//     src.B[0] = data[72];  src.B[1] = data[73];  src.B[2] = data[74];  src.B[3] = data[75];   AxisDistance_z.PID.kd = src.F;

//     if( !fly_ready )
// 		{
// 			Param_SavePID();
// 		}

}
//======================PID4==================================  
unsigned short Protocol_PID23_P,Protocol_PID23_I,Protocol_PID23_D,
               Protocol_PID24_P,Protocol_PID24_I,Protocol_PID24_D;
void Protocol_R_PID4(u8 *data)//24
{
	  union {unsigned char B[4]; float F;}src;

//     src.B[0] = data[4];   src.B[1] = data[5];   src.B[2] = data[6];   src.B[3] = data[7];    speed.PID.kp     = src.F;
//     src.B[0] = data[8];   src.B[1] = data[9];   src.B[2] = data[10];  src.B[3] = data[11];   speed.PID.ki     = src.F;
//     src.B[0] = data[12];  src.B[1] = data[13];  src.B[2] = data[14];  src.B[3] = data[15];   speed.PID.kd     = src.F;
//     src.B[0] = data[16];  src.B[1] = data[17];  src.B[2] = data[18];  src.B[3] = data[19];   hight.PID.kp     = src.F;
//     src.B[0] = data[20];  src.B[1] = data[21];  src.B[2] = data[22];  src.B[3] = data[23];   hight.PID.ki     = src.F;
//     src.B[0] = data[24];  src.B[1] = data[25];  src.B[2] = data[26];  src.B[3] = data[27];   hight.PID.kd     = src.F;
//     src.B[0] = data[28];  src.B[1] = data[29];  src.B[2] = data[30];  src.B[3] = data[31];   distance.PID.kp  = src.F;
//     src.B[0] = data[32];  src.B[1] = data[33];  src.B[2] = data[34];  src.B[3] = data[35];   distance.PID.ki  = src.F;
//     src.B[0] = data[36];  src.B[1] = data[37];  src.B[2] = data[38];  src.B[3] = data[39];   distance.PID.kd  = src.F;
//     src.B[0] = data[40];  src.B[1] = data[41];  src.B[2] = data[42];  src.B[3] = data[43];   Angle.PID.kp     = src.F;
//     src.B[0] = data[44];  src.B[1] = data[45];  src.B[2] = data[46];  src.B[3] = data[47];   Angle.PID.ki     = src.F;
//     src.B[0] = data[48];  src.B[1] = data[49];  src.B[2] = data[50];  src.B[3] = data[51];   Angle.PID.kd     = src.F;
//     src.B[0] = data[52];  src.B[1] = data[53];  src.B[2] = data[54];  src.B[3] = data[55];   Protocol_PID23_P = src.F;
//     src.B[0] = data[56];  src.B[1] = data[57];  src.B[2] = data[58];  src.B[3] = data[59];   Protocol_PID23_I = src.F;
//     src.B[0] = data[60];  src.B[1] = data[61];  src.B[2] = data[62];  src.B[3] = data[63];   Protocol_PID23_D = src.F;
//     src.B[0] = data[64];  src.B[1] = data[65];  src.B[2] = data[66];  src.B[3] = data[67];   Protocol_PID24_P = src.F;
//     src.B[0] = data[68];  src.B[1] = data[69];  src.B[2] = data[70];  src.B[3] = data[71];   Protocol_PID24_I = src.F;
//     src.B[0] = data[72];  src.B[1] = data[73];  src.B[2] = data[74];  src.B[3] = data[75];   Protocol_PID24_D = src.F;

//     if( !fly_ready )
// 		{
// 			Param_SavePID();
// 		}

}
//================获取航点=====================

void Protocol_R_GetRoute(u8 *data)//41
{
	   unsigned char  Protocol_Groups;
		 unsigned short Protocol_TotalPoint,
										 Protocol_CurrentPoint;
		 double         Protocol_Latitude,
										 Protocol_Longitude;
		 unsigned short Protocol_Altitude,
										 Protocol_Radius,
										 Protocol_Velocity;
		 unsigned char  Protocol_Action;
	
	   union {unsigned char B[8]; signed short D[4]; double F;}src;

     src.B[0] = data[4];                        Protocol_Groups       = src.B[0];
     src.B[0] = data[5];  src.B[1] = data[6];   Protocol_TotalPoint   = src.D[0];
     src.B[0] = data[7];  src.B[1] = data[8];   Protocol_CurrentPoint = src.D[0];

     src.B[0] = data[9];   src.B[1] = data[10];  
     src.B[2] = data[11];  src.B[3] = data[12];  
     src.B[4] = data[13];  src.B[5] = data[14];  
     src.B[6] = data[15];  src.B[7] = data[16];  Protocol_Longitude   = src.F;  
   
     src.B[0] = data[17];  src.B[1] = data[18];  
     src.B[2] = data[19];  src.B[3] = data[20];  
     src.B[4] = data[21];  src.B[5] = data[22];  
     src.B[6] = data[23];  src.B[7] = data[24];  Protocol_Latitude   = src.F;

     src.B[0] = data[25];  src.B[1] = data[26];  Protocol_Altitude   = src.D[0];
     src.B[0] = data[27];  src.B[1] = data[28];  Protocol_Radius     = src.D[0];
     src.B[0] = data[29];  src.B[1] = data[30];  Protocol_Velocity   = src.D[0];
                                                 Protocol_Action     = data[31];    

     Protocol_RouteDot[Protocol_Groups][Protocol_CurrentPoint][0] = Protocol_TotalPoint;
     Protocol_RouteDot[Protocol_Groups][Protocol_CurrentPoint][1] = Protocol_Longitude;
		 Protocol_RouteDot[Protocol_Groups][Protocol_CurrentPoint][2] = Protocol_Latitude;
		 Protocol_RouteDot[Protocol_Groups][Protocol_CurrentPoint][3] = Protocol_Altitude;
		 Protocol_RouteDot[Protocol_Groups][Protocol_CurrentPoint][4] = Protocol_Radius;
		 Protocol_RouteDot[Protocol_Groups][Protocol_CurrentPoint][5] = Protocol_Velocity;
		 Protocol_RouteDot[Protocol_Groups][Protocol_CurrentPoint][6] = Protocol_Action;

     Echo_GetRoute = 0x07;
     Protocol_Send_Echo = 1;
}


//获取飞行设置

void Protocol_R_TaskTakeOffSetting(u8 *data)//80
{
	   float Protocol_R_TakeOffTargetHight;
	   union {unsigned char B[4];float F;}src;

     src.B[0] = data[4];
     src.B[1] = data[5];
     src.B[2] = data[6];
     src.B[3] = data[7];
     Protocol_R_TakeOffTargetHight = src.F;
     //给起飞高度赋值
//      TaskTakeOff.TakeOffTargetHight = Protocol_R_TakeOffTargetHight;

}


void Protocol_R_TaskTakeOffToTravelSetting(u8 *data)//81
{
// 	   union {unsigned char B[4]; signed short D[2]; float F;}src;
}
void Protocol_R_TaskCircleSetting(u8 *data)//82
{
	   float Protocol_R_CircleChangeSpeed,Protocol_R_CircleChangeHight,Protocol_R_CircleLimitDistance;
	   float Protocol_R_CircleHight,Protocol_R_CircleSpeed,Protocol_R_CircleRadiu,Protocol_R_TurningRadiu;
	   union {unsigned char B[4]; float F;}src;

     src.B[0] = data[4];
     src.B[1] = data[5];
     src.B[2] = data[6];
     src.B[3] = data[7];
     Protocol_R_CircleChangeSpeed = src.F;

     src.B[0] = data[8];
     src.B[1] = data[9];
     src.B[2] = data[10];
     src.B[3] = data[11];
     Protocol_R_CircleChangeHight = src.F;

     src.B[0] = data[12];
     src.B[1] = data[13];
     src.B[2] = data[14];
     src.B[3] = data[15];
     Protocol_R_CircleLimitDistance = src.F;

		 src.B[0] = data[16];
     src.B[1] = data[17];
     src.B[2] = data[18];
     src.B[3] = data[19];
     Protocol_R_CircleHight = src.F;
		 
		 src.B[0] = data[20];
     src.B[1] = data[21];
     src.B[2] = data[22];
     src.B[3] = data[23];
     Protocol_R_CircleSpeed = src.F;
		 
		 src.B[0] = data[24];
     src.B[1] = data[25];
     src.B[2] = data[26];
     src.B[3] = data[27];
     Protocol_R_CircleRadiu = src.F;
		 
		 src.B[0] = data[28];
     src.B[1] = data[29];
     src.B[2] = data[30];
     src.B[3] = data[31];
     Protocol_R_TurningRadiu = src.F;
		 
     //==============================
//      TaskCircle.ChangeToCircleSpeed           = Protocol_R_CircleChangeSpeed;
//      TaskCircle.ChangeToCircleTargetHight     = Protocol_R_CircleChangeHight;
// 		 TaskCircle.TakeOffToTravelLimitDistance  = Protocol_R_CircleLimitDistance;
		 
// 		  Protocol_RouteDot[3][1][3]              = Protocol_R_CircleHight;
// 		  Protocol_RouteDot[3][1][4]              = Protocol_R_CircleSpeed;
// 		  Protocol_RouteDot[3][1][5]              = Protocol_R_CircleRadiu;
			
// 			TaskCircle.CircleRadiu                  = Protocol_R_TurningRadiu;//转弯半径

}
void Protocol_R_TaskTravelSetting(u8 *data)//83
{
// 	   union {unsigned char B[4]; signed short D[2]; float F;}src;
}
void Protocol_R_TaskTravelToLandingSetting(u8 *data)//84
{
// 	   union {unsigned char B[4]; signed short D[2]; float F;}src;
}
void Protocol_R_TaskLandingSetting(u8 *data)//85
{
	   float Protocol_R_LandingLowSpeedHight,Protocol_R_ModeDistance,Protocol_R_LandingDistance,Protocol_R_LandingChangeSpeed;
	   union {unsigned char B[4];float F;}src;

     src.B[0] = data[4];
     src.B[1] = data[5];
     src.B[2] = data[6];
     src.B[3] = data[7];
     Protocol_R_LandingLowSpeedHight = src.F;
 
     src.B[0] = data[8];
     src.B[1] = data[9];
     src.B[2] = data[10];
     src.B[3] = data[11];
     Protocol_R_ModeDistance = src.F;

     src.B[0] = data[12];
     src.B[1] = data[13];
     src.B[2] = data[14];
     src.B[3] = data[15];
     Protocol_R_LandingDistance = src.F;

     src.B[0] = data[16];
     src.B[1] = data[17];
     src.B[2] = data[18];
     src.B[3] = data[19];
     Protocol_R_LandingChangeSpeed = src.F;
    
     //=======================================
// 		 
//      TaskLanding.LowSpeedHight      = Protocol_R_LandingLowSpeedHight;
// 		 TaskLanding.ModeDistance       = Protocol_R_ModeDistance;
//      TaskLanding.LandingDistance    = Protocol_R_LandingDistance;
// 		 TaskLanding.LandingChangeSpeed =  Protocol_R_LandingChangeSpeed;

}
void Protocol_R_TaskCutOffSetting(u8 *data)//86
{
// 	   union {unsigned char B[4]; signed short D[2]; float F;}src;
}


