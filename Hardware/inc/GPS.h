#ifndef __GPS_H
#define __GPS_H

#include "stm32f10x.h"

extern unsigned char isGPSOK;


// typedef struct {
// 	
// 	float x;
// 	float y;
// 	float z;
// }_xyz_f;//定义一个坐标体系


// typedef struct {
// 	
// 	_xyz_f Accelerate;
// 	_xyz_f Velocity;
// 	_xyz_f Velocity_old;
//   _xyz_f Distance;
// 	_xyz_f Distance_old;
// 	float  Course;
// }_general;

// typedef struct {
// 	
// 	_general GEOCoordinate;//大地坐标系
// 	_general BodyCoordinate;//机体坐标系
// 	
// 	_general AxesCoordinate;//直角坐标系
// 	
// }_coordinate;

// extern _coordinate Coordinate;

// typedef struct {
// 	
// 	double longitude;
// 	double latitude;
// 	float  altitude;
// 	float  speed;
// 	float  course;
// }_geoinfo;


// typedef struct {
// 	
// 	 _geoinfo NewInfo;
// 	 _geoinfo OldInfo;
// }_gpsinfo;

// extern _gpsinfo GPSInfomation;


// typedef struct {
// 	
// 	float I[9][9];//单位阵
// 	
// 	float Q[9];//过程噪声协方差矩阵
// 	float R[9];//测量噪声协方差矩阵
// 	
// 	float X_last[9];
// 	
// 	float X[9];//状态量
// 	float Z[9];//观测量
// 	
// 	float A[9][9];//状态转移矩阵
// 	float B[9][9];//控制增益矩阵
// 	
// 	float H[9][9];//观测增益矩阵
// 	
// 	float P_Last[9][9];
// 	float P[9][9];//协方差矩阵
// 	float K[9][9];//卡尔曼增益矩阵
// 	
// }_kalmanfilter;



// typedef struct {
// 	_xyz_f ACC;
// 	_xyz_f VEL;
// }_velocitydef;


// void GPS_ekf(float half_T, float ax, float ay, float az,float mPitch, float mRoll, float mYaw ,double Latitude ,double Longitude ,float Altitude , float Speed , float Course ,float BorAltitude);

// void GPS_VelocityKelmanFilter(_coordinate Coord,float T,_coordinate *Target);//通过这几个量计算出速度值
// void GPS_DistanceFilter(_coordinate Coord,float T,_coordinate *Target);










void GPS_Init(void);
void GPS_Prepare(u8 data);







// GPS 模块对外引出的API 程序
extern void Initial_USART6(u32 baudrate);  //初始化
extern void GPS_Routing(void);	//GPS 线程，需要在主循环中不断调用
extern float GPS_Distance(float lat1,float lon1,float lat2,float lon2);
extern float GPS_Heading(float lat1,float lon1,float lat2,float lon2);


//------------以下数据由  void GPS_Routing(void); 子程序进行更新--------
extern unsigned char GPS_STA_Num , //使用卫星数量，从00到12
			  GPS_Update , //GPS 数据更新提示  1=数据已更新  由外部清零
			  GPS_Locked ; //定位状态，1=有效定位，0=无效定位
extern float GPS_Altitude , //天线离海平面的高度，-9999.9到9999.9米
	  Speed_GPS , //地面速率  单位  米每秒
	  Course_GPS ; //地面航向（000.0~359.9度，以真北为参考基准)
		
extern double 	  Latitude_GPS , //纬度	 单位为度
	                Longitude_GPS ; //经度  单位为度
		
		
extern volatile unsigned char GPS_Time[8]; //UTC 时间，hhmmss（时分秒）格式
extern volatile unsigned char GPS_Date[8]; //UTC日期，ddmmyy（日月年）格式

#endif

//------------------End of File----------------------------
