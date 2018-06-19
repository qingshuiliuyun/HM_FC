#ifndef __GPS_H
#define __GPS_H

#include "stm32f10x.h"

extern unsigned char isGPSOK;


// typedef struct {
// 	
// 	float x;
// 	float y;
// 	float z;
// }_xyz_f;//����һ��������ϵ


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
// 	_general GEOCoordinate;//�������ϵ
// 	_general BodyCoordinate;//��������ϵ
// 	
// 	_general AxesCoordinate;//ֱ������ϵ
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
// 	float I[9][9];//��λ��
// 	
// 	float Q[9];//��������Э�������
// 	float R[9];//��������Э�������
// 	
// 	float X_last[9];
// 	
// 	float X[9];//״̬��
// 	float Z[9];//�۲���
// 	
// 	float A[9][9];//״̬ת�ƾ���
// 	float B[9][9];//�����������
// 	
// 	float H[9][9];//�۲��������
// 	
// 	float P_Last[9][9];
// 	float P[9][9];//Э�������
// 	float K[9][9];//�������������
// 	
// }_kalmanfilter;



// typedef struct {
// 	_xyz_f ACC;
// 	_xyz_f VEL;
// }_velocitydef;


// void GPS_ekf(float half_T, float ax, float ay, float az,float mPitch, float mRoll, float mYaw ,double Latitude ,double Longitude ,float Altitude , float Speed , float Course ,float BorAltitude);

// void GPS_VelocityKelmanFilter(_coordinate Coord,float T,_coordinate *Target);//ͨ���⼸����������ٶ�ֵ
// void GPS_DistanceFilter(_coordinate Coord,float T,_coordinate *Target);










void GPS_Init(void);
void GPS_Prepare(u8 data);







// GPS ģ�����������API ����
extern void Initial_USART6(u32 baudrate);  //��ʼ��
extern void GPS_Routing(void);	//GPS �̣߳���Ҫ����ѭ���в��ϵ���
extern float GPS_Distance(float lat1,float lon1,float lat2,float lon2);
extern float GPS_Heading(float lat1,float lon1,float lat2,float lon2);


//------------����������  void GPS_Routing(void); �ӳ�����и���--------
extern unsigned char GPS_STA_Num , //ʹ��������������00��12
			  GPS_Update , //GPS ���ݸ�����ʾ  1=�����Ѹ���  ���ⲿ����
			  GPS_Locked ; //��λ״̬��1=��Ч��λ��0=��Ч��λ
extern float GPS_Altitude , //�����뺣ƽ��ĸ߶ȣ�-9999.9��9999.9��
	  Speed_GPS , //��������  ��λ  ��ÿ��
	  Course_GPS ; //���溽��000.0~359.9�ȣ����汱Ϊ�ο���׼)
		
extern double 	  Latitude_GPS , //γ��	 ��λΪ��
	                Longitude_GPS ; //����  ��λΪ��
		
		
extern volatile unsigned char GPS_Time[8]; //UTC ʱ�䣬hhmmss��ʱ���룩��ʽ
extern volatile unsigned char GPS_Date[8]; //UTC���ڣ�ddmmyy�������꣩��ʽ

#endif

//------------------End of File----------------------------
