#ifndef _IMU_H_
#define	_IMU_H_

#include "stm32f10x.h"
#include "include.h"
#include "parameter.h"
#include "math.h"
#include "senser.h"

typedef struct 
{
	_xyz_f err;
	_xyz_f err_tmp;
	_xyz_f err_lpf;
	_xyz_f err_Int;
	_xyz_f g;
	
}ref_t;


void simple_3d_trans(_xyz_f *ref, _xyz_f *in, _xyz_f *out);

extern _xyz_f reference_v;
void IMUupdate(float half_T,
	             float gx, float gy, float gz, 
							 float ax, float ay, float az,
							 float mx, float my, float mz, 
							 float *rol,float *pit,float *yaw); 
extern float Roll,Pitch,Yaw;


#endif

