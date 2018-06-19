
//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "portable.h"
#include "FreeRTOSConfig.h"


#include "stm32f10x_conf.h"
#include "include.h"
#include "rcc.h"
#include "i2c.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "bmp180.h"
#include "senser.h"
#include "24c02.h"
#include "led.h"
#include "protocol.h"
#include "usart.h"
#include "tim.h"
#include "Thread.h"
#include "parameter.h"
#include "adc.h"
#include "MPXV7002.h"
#include "ms5611.h"
#include "ak8975.h"
#include "i2c1.h"
#include "ublox.h"
#include "sbus.h"
#include "stdio.h"

#include "imu.h"

#include "fixedwing.h"

//include softwear
#include "airplane.h"





static void Ctrl_Task( void *pvParameters );
static void Datalink_Task( void *pvParameters );
static void Senser_Task(void *pvParameters);


void Ctrl_Task(void *pvParameters)
{
	  static portTickType xLastWakeTime;  
    const portTickType xFrequency = pdMS_TO_TICKS(2);
	
		while(1)
		{
			 vTaskDelayUntil( &xLastWakeTime,xFrequency );  
			 LED_Blink(0.002f);
			
			LED.RED   = (Senser.AHRS.gyro.degree.x>=0)?(Senser.AHRS.gyro.degree.x):(-Senser.AHRS.gyro.degree.x);
			LED.GREEN = (Senser.AHRS.gyro.degree.y>=0)?(Senser.AHRS.gyro.degree.y):(-Senser.AHRS.gyro.degree.y);
			LED.BLUE  = (Senser.AHRS.gyro.degree.z>=0)?(Senser.AHRS.gyro.degree.z):(-Senser.AHRS.gyro.degree.z);
			
//			TIM3->CCR1 = 1700 + Senser.AHRS.euler.pitch * 10 - Senser.AHRS.gyro.degree.y * 0.65f  + Senser.AHRS.euler.roll * 10 - Senser.AHRS.gyro.degree.x * 0.65f;
//			TIM3->CCR2 = 1300 - Senser.AHRS.euler.pitch * 10 + Senser.AHRS.gyro.degree.y * 0.65f  + Senser.AHRS.euler.roll * 10 - Senser.AHRS.gyro.degree.x * 0.65f;
//			TIM3->CCR3 = 1700 - Senser.AHRS.euler.pitch * 10 + Senser.AHRS.gyro.degree.y * 0.65f;
//			
//			TIM3->CCR4 = 1500 + Senser.AHRS.gyro.degree.z * 3.65f;
		AirPlane.Point.CurrentPoint.ID = 0;
			
			Senser.AHRS.euler.roll = LIMIT(Senser.AHRS.euler.roll,-30,30);
			Senser.AHRS.euler.pitch = LIMIT(Senser.AHRS.euler.pitch,-30,30);
			
			Senser.AHRS.gyro.degree.x = LIMIT(Senser.AHRS.gyro.degree.x,-100,100);
			Senser.AHRS.gyro.degree.y = LIMIT(Senser.AHRS.gyro.degree.y,-100,100);
			Senser.AHRS.gyro.degree.z = LIMIT(Senser.AHRS.gyro.degree.z,-100,100);
			
			TIM3->CCR1 = 1500 - Senser.AHRS.euler.roll  * 10 + Senser.AHRS.gyro.degree.x * 0.65f;
			TIM3->CCR2 = 1500 - Senser.AHRS.euler.roll  * 10 + Senser.AHRS.gyro.degree.x * 0.65f;
			TIM3->CCR3 = 1500 - Senser.AHRS.euler.pitch * 10 + Senser.AHRS.gyro.degree.y * 0.65f;
			
			TIM3->CCR4 = 1500 + Senser.AHRS.gyro.degree.z * 3.65f;
			
			
		}
}

void Datalink_Task(void *pvParameters)
{
	  static portTickType xLastWakeTime;  
    const portTickType xFrequency = pdMS_TO_TICKS(10);
		while(1)
		{
			vTaskDelayUntil( &xLastWakeTime,xFrequency ); 
      Protocol_Transmission(0.01f);
		}
}

void Senser_Task(void *pvParameters)
{
	
	  MPU6050_Init();
	  HMC5883L_SetUp();
   	MS5611_Init();
	  AK8975_Run();	
	
	  ublox_InitGps();
	
	  static portTickType xLastWakeTime;  
    const portTickType xFrequency = pdMS_TO_TICKS(2);
	
	  static uint8_t MS5611Count = 0;
	  static uint8_t MAGCount = 0;
	  static uint8_t MPXV7002Count = 0;
	  static uint16_t ParameterCount = 0;
	
		while(1)
		{
			vTaskDelayUntil( &xLastWakeTime,xFrequency ); 
			
      MPU6050_Read();
			Senser_RawToRadian();
      
			
			ParameterCount += 2;
			if(ParameterCount == 500)
			{
				  ParameterCount = 0;
			    Parameter_ParameterSetToEEPROM();
			}
			
			MS5611Count += 2;
			if(MS5611Count == 10)
			{
				  MS5611Count = 0;
			    MS5611_Update(); 				//更新ms5611气压计数据
			}
			
			MAGCount += 2;
			if(MAGCount == 20)
			{
				  MAGCount = 0;
			    HMC5883L_Read(); 				//更新磁力数据
				  AK8975_Read();	
			}
   
			MPXV7002Count += 2;
			if(MPXV7002Count == 40)
			{
				  MPXV7002Count = 0;
			    MPXV7002_GetAirSpeed();	
			}
			 
		 
			IMUupdate(0.001f,Senser.AHRS.gyro.degree.x,Senser.AHRS.gyro.degree.y,Senser.AHRS.gyro.degree.z,
			                 Senser.AHRS.acc.real.x,Senser.AHRS.acc.real.y,Senser.AHRS.acc.real.z,
			                 Senser.AHRS.mag.real.x,Senser.AHRS.mag.real.y,Senser.AHRS.mag.real.z,
			                 &Senser.AHRS.euler.roll,&Senser.AHRS.euler.pitch,&Senser.AHRS.euler.yaw180);
			
		}
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置四组
	RCC_Config();//72M
	LED_Config();
	
	IIC_Init();	
 	Parameter_ParameterSetToFC();
	


  if(BaseSetting.PPM_SBUS_select == 0x01)//subs和ppm选其中之一
	{
  	   SBUS_Init();
	}
	else
	{
	     tim1_Config();
  }
	
 	USART2_Config(57600);//数传
	Timer_Config();//pwm初始化
	ADC_Configuration();//adc初始化

	
	
	xTaskCreate(Ctrl_Task     ,    "Ctrl",         256,NULL,tskIDLE_PRIORITY+1,NULL);
	xTaskCreate(Datalink_Task,     "Datalink",     256,NULL,tskIDLE_PRIORITY+1,NULL);
	xTaskCreate(Senser_Task    ,   "Senser",       256,NULL,tskIDLE_PRIORITY+1,NULL);
	vTaskStartScheduler();
	
	while(1)
	{				
		LED_OFF(0);
		LED_OFF(1);
		LED_OFF(2);
		
  }
}











