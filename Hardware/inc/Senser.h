#ifndef _Senser_h_
#define _Senser_h_


#include "stm32f10x.h"
#include "include.h"

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}_xyz_s;

typedef struct {
	float x;
	float y;
	float z;
}_xyz_f;

typedef struct {
	uint8_t isValid;
	_xyz_s raw;
	_xyz_s bias;
	_xyz_s real;
	_xyz_f degree;
	_xyz_f radian;
	unsigned char isGetOffset;
	unsigned char isGetOffsetOK;
}_gyro_3d;

typedef struct {
	uint8_t isValid;
	_xyz_s raw;
	_xyz_s bias;
	_xyz_s real;
	_xyz_f accelerate;
	unsigned char isGetOffset;
	unsigned char isGetOffsetOK;
}_acc_3d;

typedef struct {
	uint8_t isValid;
	_xyz_s raw;
	_xyz_s bias;
	_xyz_s real;
	_xyz_f magnetism;
	unsigned char isGetOffset;
	unsigned char isGetOffsetOK;
}_mag_3d;

typedef struct {
	uint8_t isValid;
	float roll;
	float pitch;
	float yaw360;//360度格式的航向角
	float yaw180;//180度格式的航向角
}_euler_3d;

typedef struct {
	_acc_3d  acc;
	_gyro_3d gyro;
	_mag_3d  mag;
	//辅助磁力计
	_mag_3d  mag_aux1;//内部
	_mag_3d  mag_aux2;//外部
	
	_euler_3d   euler;
}_ahrs;




typedef struct {
	uint8_t isValid;
	int16_t raw;
	float   bias;
	float   real;
	unsigned char isGetOffset;
	unsigned char isGetOffsetOK;
}_senser_1d;

//辅助的传感器
typedef struct {
	_senser_1d Temperature;
	_senser_1d AirSpeed;
	_senser_1d PressureAlt;
	_senser_1d Battery;	
}_aux;

typedef struct {
	
	uint16_t Year;
	uint8_t  Month;
	uint8_t  Day;
	uint8_t  Hour;
	uint8_t  Minute;
	uint8_t  Second;
	
}_date;


typedef struct {
	uint8_t isValid;
	uint8_t NumberSV;
	uint8_t gpsFix;
	_date   Date;
	
	double Longitude;
	double Latitude;
	float  Altitude;
	float  Course;
	float  Speed;
	
	float  NorthSpeed;
	float  EastSpeed;
	float  DownSpeed;
	
	float  NorthAccelerate;
	float  EastAccelerate;
	float  DownAccelerate;
}_gps;


typedef struct {

	_aux  AUX;
	_ahrs AHRS;
	_gps  GPS;
	
}_senser;

extern _senser Senser;


void Senser_RawToRadian(void);
void Senser_AverageFilter(void);

void Senser_AHRSinit(void);
void Senser_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float halfT);
void Senser_UpdateAttitude(void); 



#endif


