/*LY*/
#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "include.h"
#include "stm32f10x.h"

#define HardwareVER  3.00
#define SoftwareVER  1.86
#define ProtocolVER  1.29
#define AirplaneVER  34.20



//=====================外部需要传输到地面站的数据定义============================
//echo
extern uint8_t Echo_CMD1;
extern uint8_t Echo_CMD2;
extern uint8_t Echo_CMD3;
extern uint8_t Echo_CMD4;
extern uint8_t Echo_GetRoute;
extern uint8_t Echo_Error2;
extern uint8_t Echo_Error3;
extern uint8_t Echo_Error4;

//Route
extern double Protocol_RouteDot[5][10][7];//7.168kb大小
//姿态
extern int16_t Protocol_ACC_X,Protocol_ACC_Y,Protocol_ACC_Z;
extern int16_t Protocol_GYRO_X,Protocol_GYRO_Y,Protocol_GYRO_Z;
extern int16_t Protocol_MAG_X,Protocol_MAG_Y,Protocol_MAG_Z;
extern float Protocol_ROL,Protocol_PIT,Protocol_YAW;
//信号
extern int16_t   Protocol_RC_CH[10];//origin
extern int16_t   Protocol_PWM_motor[8];//0~100%
extern int16_t   Protocol_PWM_servo[4];//-30~+30 deg
//GPS
extern uint8_t Protocol_GPSStatus,Protocol_StarNum;
extern float Protocol_Latitude,Protocol_Longitude;
extern float Protocol_Altitude ,Protocol_Speed;
extern float Protocol_Course;
//状态
extern float Protocol_ALT_U,Protocol_ALT_P,Protocol_Votage1,Protocol_Votage2,Protocol_Current1,Protocol_Current2;
//航线状态
extern uint8_t  Protocol_CtrlMode,Protocol_AirplaneMode,Protocol_CurrentStatus,Protocol_LockStatus,Protocol_TurningStatus;
extern uint16_t Protocol_CurrentTarget;
extern float    Protocol_AirSpeed,Protocol_WaitToFlight,Protocol_DiffrentDistance,Protocol_DiffrentAngle,Protocol_DiffrentHight,Protocol_DiffrentSpeed;
extern uint16_t Protocol_TargetRoll,Protocol_TargetPitch,Protocol_TargetYaw,Protocol_TargetHight;
extern float    Protocol_UseSpeed,Protocol_wzSpeed,Protocol_BoardHight;

//CMD
extern float Protocol_R_TakeoffHight;



//===============================================================================


void Protocol_Transmission(float T);

void Protocol_T_Echo(unsigned char Protocol_Echo_CMD1,
										 unsigned char Protocol_Echo_CMD2,
										 unsigned char Protocol_Echo_CMD3,
										 unsigned char Protocol_Echo_CMD4,
										 unsigned char Protocol_Echo_Error1,
										 unsigned char Protocol_Echo_Error2,
										 unsigned char Protocol_Echo_Error3,
										 unsigned char Protocol_Echo_Error4);
void Protocol_T_Version(float Protocol_HardwareVersion,
                        float Protocol_SoftwareVersion,
                        float Protocol_ProtocolVersion,
                        float Protocol_AirplaneVersion);
void Protocol_T_Attiude(float Protocol_ACC_X, float Protocol_ACC_Y,float Protocol_ACC_Z,
                        float Protocol_GYRO_X, float Protocol_GYRO_Y,float Protocol_GYRO_Z,
                        float Protocol_MAG_X, float Protocol_MAG_Y,float Protocol_MAG_Z,
                        float Protocol_ROL, float Protocol_PIT,float Protocol_YAW);
void Protocol_T_Signal(int16_t Protocol_ROL, int16_t Protocol_PIT, int16_t Protocol_THR, int16_t Protocol_YAW,
                       int16_t Protocol_AUX1,int16_t Protocol_AUX2,int16_t Protocol_AUX3,int16_t Protocol_AUX4,
                       int16_t Protocol_AUX5,int16_t Protocol_AUX6,int16_t Protocol_PWM1,int16_t Protocol_PWM2,
                       int16_t Protocol_PWM3,int16_t Protocol_PWM4,int16_t Protocol_PWM5,int16_t Protocol_PWM6,
                       int16_t Protocol_PWM7,int16_t Protocol_PWM8,int16_t Protocol_PWM9,int16_t Protocol_PWM10,
                       int16_t Protocol_PWM11,int16_t Protocol_PWM12);
void Protocol_T_Status(float Protocol_ALT_U,   float Protocol_ALT_P,
                       float Protocol_Votage1, float Protocol_Votage2,
                       float Protocol_Current1,float Protocol_Current2);
void Protocol_T_GPS1(uint8_t Protocol_GPSStatus,uint8_t Protocol_StarNum,
                     double Protocol_Latitude,double Protocol_Longitude,
                     float Protocol_Altitude ,float Protocol_Speed,
                     float Protocol_Course);
void Protocol_T_PID1(float Protocol_PID1_P,float Protocol_PID1_I,float Protocol_PID1_D,
                     float Protocol_PID2_P,float Protocol_PID2_I,float Protocol_PID2_D,
                     float Protocol_PID3_P,float Protocol_PID3_I,float Protocol_PID3_D,
                     float Protocol_PID4_P,float Protocol_PID4_I,float Protocol_PID4_D,
                     float Protocol_PID5_P,float Protocol_PID5_I,float Protocol_PID5_D,
                     float Protocol_PID6_P,float Protocol_PID6_I,float Protocol_PID6_D);
void Protocol_T_PID2(float Protocol_PID1_P,float Protocol_PID1_I,float Protocol_PID1_D,
                     float Protocol_PID2_P,float Protocol_PID2_I,float Protocol_PID2_D,
                     float Protocol_PID3_P,float Protocol_PID3_I,float Protocol_PID3_D,
                     float Protocol_PID4_P,float Protocol_PID4_I,float Protocol_PID4_D,
                     float Protocol_PID5_P,float Protocol_PID5_I,float Protocol_PID5_D,
                     float Protocol_PID6_P,float Protocol_PID6_I,float Protocol_PID6_D);
void Protocol_T_PID3(float Protocol_PID1_P,float Protocol_PID1_I,float Protocol_PID1_D,
                     float Protocol_PID2_P,float Protocol_PID2_I,float Protocol_PID2_D,
                     float Protocol_PID3_P,float Protocol_PID3_I,float Protocol_PID3_D,
                     float Protocol_PID4_P,float Protocol_PID4_I,float Protocol_PID4_D,
                     float Protocol_PID5_P,float Protocol_PID5_I,float Protocol_PID5_D,
                     float Protocol_PID6_P,float Protocol_PID6_I,float Protocol_PID6_D);
void Protocol_T_PID4(float Protocol_PID1_P,float Protocol_PID1_I,float Protocol_PID1_D,
                     float Protocol_PID2_P,float Protocol_PID2_I,float Protocol_PID2_D,
                     float Protocol_PID3_P,float Protocol_PID3_I,float Protocol_PID3_D,
                     float Protocol_PID4_P,float Protocol_PID4_I,float Protocol_PID4_D,
                     float Protocol_PID5_P,float Protocol_PID5_I,float Protocol_PID5_D,
                     float Protocol_PID6_P,float Protocol_PID6_I,float Protocol_PID6_D);
void Protocol_T_Route(uint8_t Groups,uint16_t Protocol_TotalPoint,uint16_t Protocol_CurrentPoint,
                      float Protocol_Latitude,float Protocol_Longitude,
                      float Protocol_Altitude,float Protocol_Radius,
                      float Protocol_Velocity,uint8_t Protocol_Action);
void Protocol_T_RouteInfo(uint8_t Protocol_CtrlMode,uint8_t Protocol_AirplaneMode,uint8_t Protocol_CurrentStatus,
                          uint8_t Protocol_LockStatus,uint8_t Protocol_TurningStatus,uint16_t Protocol_CurrentTarget,
                          float Protocol_AirSpeed,float Protocol_WaitToFlight,float Protocol_DiffrentDistance,
                          float Protocol_DiffrentAngle,float Protocol_DiffrentHight,float Protocol_DiffrentSpeed,
                          uint16_t Protocol_TargetRoll,uint16_t Protocol_TargetPitch,uint16_t Protocol_TargetYaw,
                          uint16_t Protocol_TargetHight,float UseSpeed,float wzSpeed,float BoardHight);

void Protocol_T_TaskSet(u8 ID,float A,float B,float C,float D,float E,float F,float G,float H);

//============================
extern unsigned char Protocol_TakeOff,
              Protocol_StartTravel,
              Protocol_StartRoute,
              Protocol_Landing,
              Protocol_BackHome,
              Protocol_GetRoute,
              Protocol_Lock,
              Protocol_Action;//0~255
//==============================
extern u8  Protocol_R_ReadTakeOff,
					 Protocol_R_ReadTakeOffToTravel,
					 Protocol_R_ReadCircle,
					 Protocol_R_ReadTravel,
					 Protocol_R_ReadTravelTolanding,
					 Protocol_R_ReadBackHome,
					 Protocol_R_ReadLanding,
					 Protocol_R_ReadCutOff;
//==============================
void Protocol_Prepare(u8 data);
unsigned char CheckSum(u8 *buff);
unsigned char RPCheckCRC(unsigned char *data);
uint16_t SendCheckCRC(unsigned char *data);

void Protocol_R_Beat(u8 *data);
void Protocol_R_CMD1(u8 *data);
void Protocol_R_CMD2(u8 *data);
void Protocol_R_CMD3(u8 *data);
void Protocol_R_CMD4(u8 *data);
void Protocol_R_CMD5(u8 *data);
void Protocol_R_CMD6(u8 *data);

void Protocol_R_Adjust(u8 *data);
void Protocol_R_PID1(u8 *data);
void Protocol_R_PID2(u8 *data);
void Protocol_R_PID3(u8 *data);
void Protocol_R_PID4(u8 *data);
void Protocol_R_GetRoute(u8 *data);

void Protocol_R_TaskTakeOffSetting(u8 *data);//80
void Protocol_R_TaskTakeOffToTravelSetting(u8 *data);//81
void Protocol_R_TaskCircleSetting(u8 *data);//82
void Protocol_R_TaskTravelSetting(u8 *data);//83
void Protocol_R_TaskTravelToLandingSetting(u8 *data);//84
void Protocol_R_TaskLandingSetting(u8 *data);//85
void Protocol_R_TaskCutOffSetting(u8 *data);//86










#endif

