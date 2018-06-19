#ifndef _AK8975_H_
#define	_AK8975_H_

#include "stm32f10x.h"
#include "stdbool.h"
#include "include.h"
#include "senser.h"

#define CALIBRATING_MAG_CYCLES              2000  //校准时间持续20s

#define AK8975_ADDRESS         0x0c	// 0x18

#define AK8975_WIA     0x00
#define AK8975_HXL     0x03
#define AK8975_HXH     0x04
#define AK8975_HYL     0x05
#define AK8975_HYH     0x06
#define AK8975_HZL     0x07
#define AK8975_HZH     0x08
#define AK8975_CNTL    0x0A


bool AK8975_Run(void);
void AK8975_CalOffset_Mag(void);
void AK8975_Read(void);


extern u8 Mag_CALIBRATED;
extern u8 ak8975_ok;

#endif

