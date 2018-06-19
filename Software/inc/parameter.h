

#ifndef _PARAMETER_H_
#define _PARAMETER_H_


#include "stm32f10x.h"

#define Addr_Offset       0x0000
#define Addr_BaseSetting  0x0100

#define Addr_MultiRotorPID  0x0200
#define Addr_FixedWingPID   0x0400




typedef struct {
	
	unsigned char RC_direction[16];//正反
	unsigned char PWM_direction[8];
	unsigned char PPM_SBUS_select;
	unsigned char PWM_enable[8];
	
	unsigned char GPS_enable;
	unsigned char MPXV_enable;
	unsigned char CAN_enable;
	unsigned char ADC_enable;
	unsigned char AirPlaneModel;
	
}_basesettingDef;//64bytes

extern _basesettingDef BaseSetting;


//PID定义
typedef struct {//12
	float kp;
	float ki;
	float kd;
}_pid_f;

typedef struct {//36
	_pid_f RollRate;
	_pid_f PitchRate;
	_pid_f YawRate;
}_pid_AngleRate;

typedef struct {//36
	_pid_f Roll;
	_pid_f Pitch;
	_pid_f Yaw;
}_pid_Agnle;

typedef struct {//36
	_pid_f Speed_x;
	_pid_f Speed_y;
	_pid_f Speed_z;
}_pid_Speed;

typedef struct {//36
	_pid_f Distance_x;
	_pid_f Distance_y;
	_pid_f Distance_z;
}_pid_Distance;

typedef struct {//36
	_pid_f TravelSpeed;
	_pid_f TravelCourse;
	_pid_f TravelDistance;
}_pid_Route;

typedef struct {//180
  _pid_AngleRate AngleRate;
	_pid_Agnle     Angle;
	_pid_Speed     Speed;
	_pid_Distance  Distance;
	_pid_Route     Route;
}_pid_A;

typedef struct {
  _pid_A MultiRotor;
	_pid_A FixedWing;
}_pidDef;

extern _pidDef Parameter_PID;







void Parameter_ParameterSetToFC(void);
void Parameter_ParameterSetToEEPROM(void);

void Parameter_SaveOffset(void);
void Parameter_ReadOffset(void);

void Parameter_SavePID(void);
void Parameter_ReadPID(void);
void Parameter_SaveMultiRotorPID(void);
void Parameter_ReadMultiRotorPID(void);
void Parameter_SaveFixedWingPID(void);
void Parameter_ReadFixedWingPID(void);




void Parameter_SaveBaseSetting(void);
void Parameter_ReadBaseSetting(void);






#endif

