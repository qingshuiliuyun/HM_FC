

#include "senser.h"
#include "stm32f10x.h"
#include "math.h"
#include "Matrix.h"

_senser Senser;

void Senser_RawToRadian(void)
{ 
	   Senser.AHRS.acc.real.x = Senser.AHRS.acc.raw.x - Senser.AHRS.acc.bias.x;
	   Senser.AHRS.acc.real.y = Senser.AHRS.acc.raw.y - Senser.AHRS.acc.bias.y;
	   Senser.AHRS.acc.real.z = Senser.AHRS.acc.raw.z - Senser.AHRS.acc.bias.z;
	
	   Senser.AUX.Temperature.real = 36.53+((double)Senser.AUX.Temperature.raw)/340.0f;
	
	   Senser.AHRS.gyro.real.x = Senser.AHRS.gyro.raw.x - Senser.AHRS.gyro.bias.x;
	   Senser.AHRS.gyro.real.y = Senser.AHRS.gyro.raw.y - Senser.AHRS.gyro.bias.y;
	   Senser.AHRS.gyro.real.z = Senser.AHRS.gyro.raw.z - Senser.AHRS.gyro.bias.z;
	   
	   Senser.AHRS.mag.real.x = Senser.AHRS.mag.raw.x - Senser.AHRS.mag.bias.x;
	   Senser.AHRS.mag.real.y = Senser.AHRS.mag.raw.y - Senser.AHRS.mag.bias.y;
	   Senser.AHRS.mag.real.z = Senser.AHRS.mag.raw.z - Senser.AHRS.mag.bias.z;
	
	   Senser.AHRS.mag_aux1.real.x = Senser.AHRS.mag_aux1.raw.x - Senser.AHRS.mag_aux1.bias.x;
	   Senser.AHRS.mag_aux1.real.y = Senser.AHRS.mag_aux1.raw.y - Senser.AHRS.mag_aux1.bias.y;
	   Senser.AHRS.mag_aux1.real.z = Senser.AHRS.mag_aux1.raw.z - Senser.AHRS.mag_aux1.bias.z;
	
	   Senser.AHRS.mag_aux2.real.x = Senser.AHRS.mag_aux2.raw.x - Senser.AHRS.mag_aux2.bias.x;
	   Senser.AHRS.mag_aux2.real.y = Senser.AHRS.mag_aux2.raw.y - Senser.AHRS.mag_aux2.bias.y;
	   Senser.AHRS.mag_aux2.real.z = Senser.AHRS.mag_aux2.raw.z - Senser.AHRS.mag_aux2.bias.z;
	
		 Senser_AverageFilter();//做一次滑动窗口滤波
	
	   Senser.AHRS.gyro.degree.x = Senser.AHRS.gyro.real.x * RawToDeg;
		 Senser.AHRS.gyro.degree.y = Senser.AHRS.gyro.real.y * RawToDeg;
		 Senser.AHRS.gyro.degree.z = Senser.AHRS.gyro.real.z * RawToDeg;
}

/**************************实现函数********************************************
*函数原型:	   void Senser_AverageFilter(void)
*功　　能:	   原始数据滑动窗口滤波 	
输入参数：    无
输出参数：    原始数据进行滤波
*******************************************************************************/
#define FilterLen 10
#define SenserCH  15
void Senser_AverageFilter(void)
{
	static int16_t Filter[SenserCH][FilterLen];
	static int32_t FilterSum[SenserCH];
	static unsigned char FilterCount = 0,FilterSUMcount = 1;
	int i=0;
	
	for(i =0 ;i<SenserCH;i++)
	{
		  	FilterSum[i] -= Filter[i][FilterCount];
  }

  FilterSum[0]  += Senser.AHRS.acc.real.x;
	FilterSum[1]  += Senser.AHRS.acc.real.y;
	FilterSum[2]  += Senser.AHRS.acc.real.z;
	FilterSum[3]  += Senser.AHRS.gyro.real.x;
	FilterSum[4]  += Senser.AHRS.gyro.real.y;
	FilterSum[5]  += Senser.AHRS.gyro.real.z;
	FilterSum[6]  += Senser.AHRS.mag.real.x;
	FilterSum[7]  += Senser.AHRS.mag.real.y;
	FilterSum[8]  += Senser.AHRS.mag.real.z;
	FilterSum[9]  += Senser.AHRS.mag_aux1.real.x;
	FilterSum[10] += Senser.AHRS.mag_aux1.real.y;
	FilterSum[11] += Senser.AHRS.mag_aux1.real.z;
	FilterSum[12] += Senser.AHRS.mag_aux2.real.x;
	FilterSum[13] += Senser.AHRS.mag_aux2.real.y;
	FilterSum[14] += Senser.AHRS.mag_aux2.real.z;
		
	Senser.AHRS.acc.real.x       = (float)FilterSum[0] /FilterSUMcount;
	Senser.AHRS.acc.real.y       = (float)FilterSum[1] /FilterSUMcount;
	Senser.AHRS.acc.real.z       = (float)FilterSum[2] /FilterSUMcount;
	Senser.AHRS.gyro.real.x      = (float)FilterSum[3] /FilterSUMcount;
	Senser.AHRS.gyro.real.y      = (float)FilterSum[4] /FilterSUMcount;
	Senser.AHRS.gyro.real.z      = (float)FilterSum[5] /FilterSUMcount;
	Senser.AHRS.mag.real.x       = (float)FilterSum[6] /FilterSUMcount;
	Senser.AHRS.mag.real.y       = (float)FilterSum[7] /FilterSUMcount;
	Senser.AHRS.mag.real.z       = (float)FilterSum[8] /FilterSUMcount;
	Senser.AHRS.mag_aux1.real.x  = (float)FilterSum[9] /FilterSUMcount;
	Senser.AHRS.mag_aux1.real.y  = (float)FilterSum[10] /FilterSUMcount;
	Senser.AHRS.mag_aux1.real.z  = (float)FilterSum[11] /FilterSUMcount;
	Senser.AHRS.mag_aux2.real.x  = (float)FilterSum[12] /FilterSUMcount;
	Senser.AHRS.mag_aux2.real.y  = (float)FilterSum[13] /FilterSUMcount;
	Senser.AHRS.mag_aux2.real.z  = (float)FilterSum[14] /FilterSUMcount;
	
	Filter[0][FilterCount]  = Senser.AHRS.acc.real.x;
	Filter[1][FilterCount]  = Senser.AHRS.acc.real.y;
	Filter[2][FilterCount]  = Senser.AHRS.acc.real.z;
	Filter[3][FilterCount]  = Senser.AHRS.gyro.real.x;
	Filter[4][FilterCount]  = Senser.AHRS.gyro.real.y;
	Filter[5][FilterCount]  = Senser.AHRS.gyro.real.z;
	Filter[6][FilterCount]  = Senser.AHRS.mag.real.x;
	Filter[7][FilterCount]  = Senser.AHRS.mag.real.y;
	Filter[8][FilterCount]  = Senser.AHRS.mag.real.z;
	Filter[9][FilterCount]  = Senser.AHRS.mag_aux1.real.x;
	Filter[10][FilterCount] = Senser.AHRS.mag_aux1.real.y;
	Filter[11][FilterCount] = Senser.AHRS.mag_aux1.real.z;
	Filter[12][FilterCount] = Senser.AHRS.mag_aux2.real.x;
	Filter[13][FilterCount] = Senser.AHRS.mag_aux2.real.y;
	Filter[14][FilterCount] = Senser.AHRS.mag_aux2.real.z;
	
	
	FilterCount++;
	if(FilterCount >= FilterLen)
	{
		 FilterCount = 0;
  }
	
	FilterSUMcount++;
	if(FilterSUMcount >= FilterLen)
	{
		 FilterSUMcount = FilterLen;
  }
}



float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}



