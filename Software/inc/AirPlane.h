
#ifndef _AIRPLANE_H_
#define _AIRPLANE_H_
#include "stm32f10x.h"

//basedef
typedef struct {
	
	uint8_t x;
	uint8_t y;
	uint8_t z;
	
}_xyz_uint8_t;

typedef struct {
	
	uint16_t x;
	uint16_t y;
	uint16_t z;
	
}_xyz_uint16_t;

typedef struct {
	
	uint32_t x;
	uint32_t y;
	uint32_t z;
	
}_xyz_uint32_t;

typedef struct {
	
	float x;
	float y;
	float z;
	
}_xyz_float;

//end basedef







//���ں�·��Ķ���
typedef struct {
	
	uint32_t ID;
	
	double Longitude;
	double Latitude;
	double Altitude;
	
	float Course;
	
	float AirSpeed;
	float GroundSpeed;
	float NorthSpeed;
	float EastSpeed;
	float DownSpeed;
	
}_point;

typedef struct {
	
	_point LastPoint;
	_point CurrentPoint;
	_point TargetPoint;
	
}_waypoint;
//��·�㶨�����

//��̬���ݶ���


//��̬���ݶ������


//���в�������






//���в����������

//�ɻ�״̬����
typedef struct {
	
	uint8_t CurrentCMD;
	
}_cmd;



typedef struct {
	_cmd CMD;
	
}_status;

//�ɻ�״̬�������



typedef struct {
	
	_waypoint Point;
	_status   Status;
	
}_airplane;


extern _airplane AirPlane;

















#endif

