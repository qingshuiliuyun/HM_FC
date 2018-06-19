



#include "stm32f10x_conf.h"
#include "led.h"


#ifdef LED_EN


_LEDType LED;

void LED_Config(void)
{
	         GPIO_InitTypeDef GPIO_InitStructure;
	    
	         RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
					 GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;     
			     GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
			     GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	
					 GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	         LED.BLUE  = 0;
	         LED.GREEN = 0;
	         LED.RED   = 0;
}

void LED_ON(unsigned char LED)
{
	  switch(LED)
	  {
			   case LED_RED:
				 {
					 GPIO_ResetBits(GPIOC,GPIO_Pin_15);
				 }break;
			   case LED_GREEN:
				 {
					 GPIO_ResetBits(GPIOC,GPIO_Pin_14);
				 }break;
			   case LED_BLUE:
				 {
					 GPIO_ResetBits(GPIOC,GPIO_Pin_13);
				 }break;
			   default:
				 {
					 GPIO_ResetBits(GPIOC,GPIO_Pin_13);
					 GPIO_ResetBits(GPIOC,GPIO_Pin_14);
					 GPIO_ResetBits(GPIOC,GPIO_Pin_15);
				 }break;
    }
	  
}

void LED_OFF(unsigned char LED)
{
	  switch(LED)
	  {
			   case LED_RED:
				 {
					 GPIO_SetBits(GPIOC,GPIO_Pin_15);
				 }break;
			   case LED_GREEN:
				 {
					 GPIO_SetBits(GPIOC,GPIO_Pin_14);
				 }break;
			   case LED_BLUE:
				 {
					 GPIO_SetBits(GPIOC,GPIO_Pin_13);
				 }break;
			   default:
				 {
					 GPIO_SetBits(GPIOC,GPIO_Pin_13);
					 GPIO_SetBits(GPIOC,GPIO_Pin_14);
					 GPIO_SetBits(GPIOC,GPIO_Pin_15);
				 }break;
    }
}


void LED_Blink(float T)
{
	  static float LED_Period = 0.01167f;//20ms一个周期(50Hz)(0.04167s的时候人眼无法识别微弱的变化)
    static float Time;
	 
	  Time += T;
	  if(Time>=LED_Period)
		{
			 Time -= LED_Period;//如果已经到达一个LED周期,那么从头计时
    }
	  
	  if((LED.RED/255.0f)>=(Time/LED_Period))
		{
			  LED_ON(LED_RED);
    }
		else
		{
			 LED_OFF(LED_RED);
    }
		 	
		
		if((LED.GREEN/255.0f)>=(Time/LED_Period))
		{
			  LED_ON(LED_GREEN);
    }
		else
		{
			 LED_OFF(LED_GREEN);
    }
		
		if((LED.BLUE/255.0f)>=(Time/LED_Period))
		{
			  LED_ON(LED_BLUE);
    }	
		else
		{
			 LED_OFF(LED_BLUE);
    }
}


void LED_SetRGB(unsigned char RED,unsigned char GREEN,unsigned char BLUE)
{
	   LED.RED   = RED;
	   LED.GREEN = GREEN;
		 LED.BLUE  = BLUE;
}

void LED_SetRED(unsigned char RED)
{
	   LED.RED   = RED;
}

void LED_SetGREEN(unsigned char GREEN)
{
	   LED.GREEN = GREEN;
}

void LED_SetBLUE(unsigned char BLUE)
{
		 LED.BLUE  = BLUE;
}


#endif

















