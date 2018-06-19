


#include "Thread.h"
#include "led.h"
#include "protocol.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "Senser.h"
#include "parameter.h"
#include "bmp180.h"
#include "imu.h"
#include "ms5611.h"
#include "ak8975.h"
#include "gps.h"
#include "mpxv7002.h"

_thread Thread;

void Thread_Event(void)
{
	    Thread.ThreadCount_1ms++;
	    Thread.ThreadCount_2ms++;
	    Thread.ThreadCount_5ms++;
	    Thread.ThreadCount_10ms++;
	    Thread.ThreadCount_20ms++;
	    Thread.ThreadCount_50ms++;
	    Thread.ThreadCount_100ms++;
	    Thread.ThreadCount_200ms++;
	    Thread.ThreadCount_500ms++;
	    Thread.ThreadCount_1000ms++;
}

void Thread_Loop(void)
{
	  if(Thread.ThreadCount_1ms >= 1)
		{
			 Thread.ThreadCount_1ms = 0;
			 Thread_1ms();
    }
		if(Thread.ThreadCount_2ms >= 2)
		{
			 Thread.ThreadCount_2ms = 0;
			 Thread_2ms();
    }
		if(Thread.ThreadCount_5ms >= 5)
		{
			 Thread.ThreadCount_5ms = 0;
			 Thread_5ms();
    }
		if(Thread.ThreadCount_10ms >= 10)
		{
			 Thread.ThreadCount_10ms = 0;
			 Thread_10ms();
    }
		if(Thread.ThreadCount_20ms >= 20)
		{
			 Thread.ThreadCount_20ms = 0;
			 Thread_20ms();
    }
		if(Thread.ThreadCount_50ms >= 50)
		{
			 Thread.ThreadCount_50ms = 0;
			 Thread_50ms();
    }
		if(Thread.ThreadCount_100ms >= 100)
		{
			 Thread.ThreadCount_100ms = 0;
			 Thread_100ms();
    }
		if(Thread.ThreadCount_200ms >= 200)
		{
			 Thread.ThreadCount_200ms = 0;
			 Thread_200ms();
    }
		if(Thread.ThreadCount_500ms >= 500)
		{
			 Thread.ThreadCount_500ms = 0;
			 Thread_500ms();
    }
		if(Thread.ThreadCount_1000ms >= 1000)
		{
			 Thread.ThreadCount_1000ms = 0;
			 Thread_1000ms();
    }
}

void Thread_1ms(void)
{
	   float Thread_1ms_Time = Get_Cycle_T(0);
	   LED_Blink(Thread_1ms_Time);
}
void Thread_2ms(void)
{
	    float Thread_2ms_Time = Get_Cycle_T(1);
	    MPU6050_Read();
 		  Senser_RawToRadian();
}

void Thread_5ms(void)
{
	    float Thread_5ms_Time = Get_Cycle_T(2);
      Protocol_Transmission(Thread_5ms_Time);//未做好
}

void Thread_10ms(void)//100hz
{
	    float Thread_10ms_Time = Get_Cycle_T(3);
 	    MS5611_Update(); 				//更新ms5611气压计数据
		  MPXV7002_GetAirSpeed();
}

void Thread_20ms(void)//50hz，控制函数在这里
{ 
	   float Thread_20ms_Time = Get_Cycle_T(4);
 		 HMC5883L_Read();
	   AK8975_Read();	   
}

void Thread_50ms(void)
{
	   float Thread_50ms_Time = Get_Cycle_T(5);
	   GPS_Routing();
}
u8 cont;
void Thread_100ms(void)
{
	   float Thread_100ms_Time = Get_Cycle_T(6);

	  cont++;
}

void Thread_200ms(void)
{
	  float Thread_200ms_Time = Get_Cycle_T(7);
}

void Thread_500ms(void)
{
	  float Thread_500ms_Time = Get_Cycle_T(8);
}

void Thread_1000ms(void)
{
	   float Thread_1000ms_Time = Get_Cycle_T(9);
}

