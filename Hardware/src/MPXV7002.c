


#include "mpxv7002.h"
#include "math.h"
#include "senser.h"
#include "parameter.h"


float AirDensityData[66] = {//空气密度，从-15度开始，50度结束
	1.368,1.363,1.358,1.353,1.348,1.342,1.337,1.332,1.327,1.322,//-15 -> -6
	1.317,1.312,1.308,1.303,1.298,1.293,1.288,1.284,1.279,1.275,//-5  -> 4
  1.270,1.265,1.261,1.256,1.252,1.248,1.243,1.239,1.235,1.230,//5   -> 14 
  1.226,1.222,1.217,1.213,1.209,1.205,1.201,1.197,1.193,1.189,//15  -> 24
  1.185,1.181,1.177,1.173,1.169,1.165,1.161,1.157,1.154,1.150,//25  -> 34                 
  1.146,1.142,1.139,1.135,1.132,1.128,1.124,1.121,1.117,1.114,//35  -> 44
	1.110,1.107,1.103,1.100,1.096,1.093//45  -> 50
};


void MPXV7002_GetAirSpeed(void)
{
	  //Vout = VS × (0.2 × P(kPa)+0.5) ± 6.25% VFSS
	  float AirSpeed;
	  float Pressure;
	  float VoltageValue;
    float AirDensity;
	
	  u8 AirDensityIndex_1,AirDensityIndex_2;
    u8 Integer;
    float Decimal;
	
	
	
	  Integer = (uint16_t)Senser.AUX.Temperature.real;
    Decimal = Senser.AUX.Temperature.real - Integer;
	
	  if(Decimal != 0)
	 {
		AirDensityIndex_1 = Integer +15;
		AirDensityIndex_2 = Integer +16;
		AirDensity = AirDensityData[AirDensityIndex_1] + (AirDensityData[AirDensityIndex_2] - AirDensityData[AirDensityIndex_1]) *(Decimal);
		}
		else
		{
			AirDensityIndex_1 = Integer +15;
			AirDensity = AirDensityData[AirDensityIndex_1];
		}

	  VoltageValue = ((Senser.AUX.AirSpeed.raw-Senser.AUX.AirSpeed.bias) * 3300 )/4096.0f;
	  VoltageValue = (VoltageValue/1000.0f) * 2;
 	  Pressure     = (VoltageValue/5.0f - 0.5)*5 * 1000;//kPa
	  AirSpeed     = (float)(sqrt(fabsf((Pressure * 2) / AirDensity))); //公式需要的气压为Pa
	  Senser.AUX.AirSpeed.real = AirSpeed;
}


s32 MPXV_sum_temp;
u32 MPXV_sum_cnt = 0;
void MPXV7002_GetOffset(uint16_t ADC)
{
	   float BaseADC = 1551.515f;//计算出当没有空速时对应的理论ADC值
	
	   if(Senser.AUX.AirSpeed.isGetOffset)
		 {
			   MPXV_sum_cnt ++;
			   MPXV_sum_temp += (ADC - BaseADC);
			 
			   if(Senser.AUX.AirSpeed.isGetOffsetOK)
				 {
					  Senser.AUX.AirSpeed.bias = (float)MPXV_sum_temp/MPXV_sum_cnt;
					 
					  Senser.AUX.AirSpeed.isGetOffset = 0;
					  Senser.AUX.AirSpeed.isGetOffsetOK = 0;
					  MPXV_sum_cnt = 0;
					  MPXV_sum_temp = 0;
					 
					  Parameter_SaveOffset();
         }
     }
	

}





