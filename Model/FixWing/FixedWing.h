



#ifndef _FIXEDWING_H_
#define _FIXEDWING_H_

#include "stm32f10x.h"

typedef struct {
	float error;
	float error_i;
	float error_d;
	float out;
}_fixedwingpid;


typedef struct {
	_fixedwingpid roll;
	_fixedwingpid pitch;
	_fixedwingpid heading;
}_fixedwingctrl;







#endif







