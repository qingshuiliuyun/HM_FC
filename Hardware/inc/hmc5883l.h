#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "stm32f10x.h"
#include "I2C.h"
#include "sys.h"
#include "LED.h"

#define HMC58X3_ADDR 0x1E // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define HMC58X3_R_YM (7)  //!< Register address for YM.
#define HMC58X3_R_YL (8)  //!< Register address for YL.
#define HMC58X3_R_ZM (5)  //!< Register address for ZM.
#define HMC58X3_R_ZL (6)  //!< Register address for ZL.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

void HMC5883L_SetUp(void);	//��ʼ��
void HMC58X3_getID(unsigned char *id);	//��оƬID
void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z); //��ADC
void HMC58X3_mgetValues(float *arry); //IMU ר�õĶ�ȡ������ֵ
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z);
void HMC5883L_Read(void);
void HMC5883L_GetOffset(void);
#endif

//------------------End of File----------------------------