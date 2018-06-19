#ifndef __HMC5983L_H
#define __HMC5983L_H

#include "stm32f10x.h"
#include "I2C.h"
#include "sys.h"
#include "LED.h"

#define HMC5983_ADDR 0x3C // 7 bit address of the HMC5983 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC5983 register map. For details see HMC5983 datasheet
#define HMC5983_R_CONFA 0
#define HMC5983_R_CONFB 1
#define HMC5983_R_MODE 2
#define HMC5983_R_XM 3
#define HMC5983_R_XL 4

#define HMC5983_R_YM (7)  //!< Register address for YM.
#define HMC5983_R_YL (8)  //!< Register address for YL.
#define HMC5983_R_ZM (5)  //!< Register address for ZM.
#define HMC5983_R_ZL (6)  //!< Register address for ZL.

#define HMC5983_R_STATUS 9
#define HMC5983_R_IDA 10
#define HMC5983_R_IDB 11
#define HMC5983_R_IDC 12


void HMC5983L_SetUp(void);	//初始化
void HMC5983_getID(unsigned char *id);	//读芯片ID
void HMC5983_getRaw(int16_t *x,int16_t *y,int16_t *z); //读ADC
void HMC5983_mgetValues(float *arry); //IMU 专用的读取磁力计值
void HMC5983_getlastValues(int16_t *x,int16_t *y,int16_t *z); //直接读取最近一次的数值
void HMC5983_mgetValuesint(int16_t *arry);
#endif

//------------------End of File----------------------------
