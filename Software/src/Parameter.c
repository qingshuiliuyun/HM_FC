
#include "Senser.h"
#include "24c02.h"
#include "Parameter.h"
#include "string.h"

//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "portable.h"
#include "FreeRTOSConfig.h"

_basesettingDef BaseSetting;
_pidDef Parameter_PID;


//读取配置参数
void Parameter_ParameterSetToFC(void)
{
	   Parameter_ReadOffset();
	   Parameter_ReadPID();
	   Parameter_ReadBaseSetting();
}


//保存配置参数
u8 isSaveOffset=0,isSavePID=0,isSaveBaseSetting = 0;
void Parameter_ParameterSetToEEPROM(void)
{
	   if(isSaveOffset)
		 {
			   isSaveOffset = 0;
	       Parameter_SaveOffset();
		 }
		 if(isSavePID)
		 {
			   isSavePID = 0;
	       Parameter_SavePID();
		 }
		 if(isSaveBaseSetting)
		 {
			   isSaveBaseSetting = 0;
	       Parameter_SaveBaseSetting();
		 }
}





void Parameter_SaveOffset(void)
{
	   union {unsigned char B[64]; int16_t D[32];}src;

     src.D[0] = Senser.AHRS.acc.bias.x; 
     src.D[1] = Senser.AHRS.acc.bias.y; 
     src.D[2] = Senser.AHRS.acc.bias.z; 

     src.D[3] = Senser.AHRS.gyro.bias.x; 
     src.D[4] = Senser.AHRS.gyro.bias.y; 
     src.D[5] = Senser.AHRS.gyro.bias.z; 

     src.D[6] = Senser.AHRS.mag.bias.x; 
     src.D[7] = Senser.AHRS.mag.bias.y; 
     src.D[8] = Senser.AHRS.mag.bias.z; 

     src.D[9]  = Senser.AHRS.mag_aux1.bias.x; 
     src.D[10] = Senser.AHRS.mag_aux1.bias.y; 
     src.D[11] = Senser.AHRS.mag_aux1.bias.z; 

     src.D[12] = Senser.AHRS.mag_aux2.bias.x; 
     src.D[13] = Senser.AHRS.mag_aux2.bias.y; 
     src.D[14] = Senser.AHRS.mag_aux2.bias.z; 

     src.D[15] = Senser.AUX.AirSpeed.bias; 
     src.D[16] = Senser.AUX.Temperature.bias; 

     EEPROM_nWriteData(Addr_Offset,src.B,64);
}

void Parameter_ReadOffset(void)
{
	   union {unsigned char B[64]; int16_t D[32];}src;

     EEPROM_nReadData(Addr_Offset,src.B,64);
 
     Senser.AHRS.acc.bias.x = src.D[0]; 
     Senser.AHRS.acc.bias.y = src.D[1]; 
     Senser.AHRS.acc.bias.z = src.D[2]; 

     Senser.AHRS.gyro.bias.x = src.D[3]; 
     Senser.AHRS.gyro.bias.y = src.D[4]; 
     Senser.AHRS.gyro.bias.z = src.D[5]; 

     Senser.AHRS.mag.bias.x = src.D[6]; 
     Senser.AHRS.mag.bias.y = src.D[7]; 
     Senser.AHRS.mag.bias.z = src.D[8]; 

     Senser.AHRS.mag_aux1.bias.x = src.D[9]; 
     Senser.AHRS.mag_aux1.bias.y = src.D[10]; 
     Senser.AHRS.mag_aux1.bias.z = src.D[11]; 

     Senser.AHRS.mag_aux2.bias.x = src.D[12]; 
     Senser.AHRS.mag_aux2.bias.y = src.D[13]; 
     Senser.AHRS.mag_aux2.bias.z = src.D[14]; 

     Senser.AUX.AirSpeed.bias    = src.D[15]; 
     Senser.AUX.Temperature.bias = src.D[16]; 
}



void Parameter_SavePID(void)
{
	   Parameter_SaveMultiRotorPID();
	   vTaskDelay(pdMS_TO_TICKS(10)); 
	   Parameter_SaveFixedWingPID();
	   vTaskDelay(pdMS_TO_TICKS(10)); 
}

void Parameter_ReadPID(void)
{
	   Parameter_ReadMultiRotorPID();
	   Parameter_ReadFixedWingPID();
} 


void Parameter_SaveMultiRotorPID(void)
{
		 unsigned char B[sizeof(Parameter_PID.MultiRotor)];
		 
		 memcpy(&B,&Parameter_PID.MultiRotor,sizeof(Parameter_PID.MultiRotor));
	   EEPROM_nWriteData(Addr_MultiRotorPID,B,sizeof(Parameter_PID.MultiRotor));
}

void Parameter_ReadMultiRotorPID(void)
{
	   unsigned char B[sizeof(Parameter_PID.MultiRotor)];
     EEPROM_nReadData(Addr_MultiRotorPID,B,sizeof(Parameter_PID.MultiRotor));
		 memcpy(&Parameter_PID.MultiRotor,&B,sizeof(Parameter_PID.MultiRotor));
	
}

void Parameter_SaveFixedWingPID(void)
{
	   unsigned char B[sizeof(Parameter_PID.FixedWing)];
		 
		 memcpy(&B,&Parameter_PID.FixedWing,sizeof(Parameter_PID.FixedWing));
	   EEPROM_nWriteData(Addr_FixedWingPID,B,sizeof(Parameter_PID.FixedWing));
}

void Parameter_ReadFixedWingPID(void)
{
	   unsigned char B[sizeof(Parameter_PID.FixedWing)];
     EEPROM_nReadData(Addr_FixedWingPID,B,sizeof(Parameter_PID.FixedWing));
		 memcpy(&Parameter_PID.FixedWing,&B,sizeof(Parameter_PID.FixedWing));
}


























void Parameter_SaveBaseSetting(void)
{
	   union {unsigned char B[64];}src;

     src.B[0]  = BaseSetting.RC_direction[0]; 
     src.B[1]  = BaseSetting.RC_direction[1];
     src.B[2]  = BaseSetting.RC_direction[2];
     src.B[3]  = BaseSetting.RC_direction[3];
     src.B[4]  = BaseSetting.RC_direction[4];
     src.B[5]  = BaseSetting.RC_direction[5];
     src.B[6]  = BaseSetting.RC_direction[6]; 
     src.B[7]  = BaseSetting.RC_direction[7];
     src.B[8]  = BaseSetting.RC_direction[8];
     src.B[9]  = BaseSetting.RC_direction[9];
     src.B[10] = BaseSetting.RC_direction[10];
     src.B[11] = BaseSetting.RC_direction[11]; 
     src.B[12] = BaseSetting.RC_direction[12];
     src.B[13] = BaseSetting.RC_direction[13];
     src.B[14] = BaseSetting.RC_direction[14]; 
     src.B[15] = BaseSetting.RC_direction[15];

     src.B[16]  = BaseSetting.PWM_direction[0]; 
     src.B[17]  = BaseSetting.PWM_direction[1];
     src.B[18]  = BaseSetting.PWM_direction[2];
     src.B[19]  = BaseSetting.PWM_direction[3];
     src.B[20]  = BaseSetting.PWM_direction[4];
     src.B[21]  = BaseSetting.PWM_direction[5];
     src.B[22]  = BaseSetting.PWM_direction[6]; 
     src.B[23]  = BaseSetting.PWM_direction[7];

     src.B[24]  = BaseSetting.PPM_SBUS_select; 
		 
		 src.B[25]  = BaseSetting.PWM_enable[0]; 
     src.B[26]  = BaseSetting.PWM_enable[1];
     src.B[27]  = BaseSetting.PWM_enable[2];
     src.B[28]  = BaseSetting.PWM_enable[3];
     src.B[29]  = BaseSetting.PWM_enable[4];
     src.B[30]  = BaseSetting.PWM_enable[5];
     src.B[31]  = BaseSetting.PWM_enable[6]; 
     src.B[32]  = BaseSetting.PWM_enable[7];
		 
     src.B[33]  = BaseSetting.GPS_enable; 
     src.B[34]  = BaseSetting.MPXV_enable;
     src.B[35]  = BaseSetting.CAN_enable;
     src.B[36]  = BaseSetting.ADC_enable;
     src.B[37]  = BaseSetting.AirPlaneModel;

     EEPROM_nWriteData(Addr_BaseSetting,src.B,64);
}

void Parameter_ReadBaseSetting(void)
{
	   union {unsigned char B[64];}src;

     EEPROM_nReadData(Addr_BaseSetting,src.B,64);
 
     BaseSetting.RC_direction[0]  = src.B[0];
     BaseSetting.RC_direction[1]  = src.B[1];
     BaseSetting.RC_direction[2]  = src.B[2];
     BaseSetting.RC_direction[3]  = src.B[3];
     BaseSetting.RC_direction[4]  = src.B[4];
     BaseSetting.RC_direction[5]  = src.B[5];
     BaseSetting.RC_direction[6]  = src.B[6];
     BaseSetting.RC_direction[7]  = src.B[7];
     BaseSetting.RC_direction[8]  = src.B[8];
     BaseSetting.RC_direction[9]  = src.B[9];
     BaseSetting.RC_direction[10]  = src.B[10];
     BaseSetting.RC_direction[11]  = src.B[11];
     BaseSetting.RC_direction[12]  = src.B[12];
     BaseSetting.RC_direction[13]  = src.B[13];
     BaseSetting.RC_direction[14]  = src.B[14];
     BaseSetting.RC_direction[15]  = src.B[15];

     BaseSetting.PWM_direction[0]  = src.B[16];
     BaseSetting.PWM_direction[1]  = src.B[17];
     BaseSetting.PWM_direction[2]  = src.B[18];
     BaseSetting.PWM_direction[3]  = src.B[19];
     BaseSetting.PWM_direction[4]  = src.B[20];
     BaseSetting.PWM_direction[5]  = src.B[21];
     BaseSetting.PWM_direction[6]  = src.B[22];
     BaseSetting.PWM_direction[7]  = src.B[23];

     BaseSetting.PPM_SBUS_select  = src.B[24];
		 
		 BaseSetting.PWM_enable[0]  = src.B[25];
     BaseSetting.PWM_enable[1]  = src.B[26];
     BaseSetting.PWM_enable[2]  = src.B[27];
     BaseSetting.PWM_enable[3]  = src.B[28];
     BaseSetting.PWM_enable[4]  = src.B[29];
     BaseSetting.PWM_enable[5]  = src.B[30];
     BaseSetting.PWM_enable[6]  = src.B[31];
     BaseSetting.PWM_enable[7]  = src.B[32];
		 
     BaseSetting.GPS_enable    = src.B[33];
     BaseSetting.MPXV_enable   = src.B[34];
     BaseSetting.CAN_enable    = src.B[35];
     BaseSetting.ADC_enable    = src.B[36];
		 
		 BaseSetting.AirPlaneModel = src.B[37];
} 









