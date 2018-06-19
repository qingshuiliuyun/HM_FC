

/*   TIM.C
*  @ HM
*  @ 2015.09.25
*  @ gxms0904hm@163.com
*  @ configuration the timer as output PWM and input PWM  and timer
*  @ funtions:
*  @          void tim2_config(void);
*  @          void tim3_config(void);
*  @          void tim4_config(void);
*  @          void PWM_OUT_Config(void);
*  @ switch define :
*  @          TIM_EN
*/


#include "stm32f10x_conf.h"
#include "tim.h"

#ifdef TIM_EN


void tim1_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
  TIM_ICInitTypeDef  TIM_ICInitStructure;	
  NVIC_InitTypeDef NVIC_InitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʱ��ʹ��  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //tim1
	
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;            
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;            
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  TIM_TimeBaseStructure.TIM_Period = 0xffff; //�����Զ���װ�ؼĴ�������ֵ
  TIM_TimeBaseStructure.TIM_Prescaler =(72-1);//����Ԥ��Ƶֵ  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���ģʽ  
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//�ظ���������  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //������ʼ��  
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);//���жϱ�־λ  

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	    //�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	          //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	    
  //�������ȼ�  
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;    
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�0��  
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�0��  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  NVIC_Init(&NVIC_InitStructure);   
  
	TIM_ITConfig(TIM1,TIM_IT_CC3,ENABLE); 
  TIM_Cmd(TIM1, ENABLE);  //ʹ��TIMx����  	
}



/*  pwm output
 *  @ HM
 *  @ 2015.09.25
 *  @ gxms0904hm@163.com
 *  @ funtion: configuration the timer for PWM output
 *  @ arg:     NONE
 *  @ retrun : NONE
 */
void tim2_config(void)
{
	  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	  TIM_OCInitTypeDef TIM_OCInitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;
	
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
		/* GPIOA and GPIOC clock enable */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //tim2

		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

	  TIM_DeInit(TIM2);
	  TIM_TimeBaseStructure.TIM_Prescaler = 71;  //1M
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseStructure.TIM_Period = 2499;   //����    1000000/2500 = 400hz = 2.5ms T
    TIM_TimeBaseStructure.TIM_ClockDivision =0x00;
	  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 1500;                                //ռ�ձ�1000~2000
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	
	  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
		
		TIM_ARRPreloadConfig(TIM2, ENABLE);
	  TIM_Cmd(TIM2, ENABLE);
	
}
/*  pwm input
 *  @ HM
 *  @ 2015.09.25
 *  @ gxms0904hm@163.com
 *  @ funtion: configuration the timer for PWM input
 *  @ arg:     NONE
 *  @ retrun : NONE
 */
void tim3_config(void)
{
	   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	   TIM_OCInitTypeDef TIM_OCInitStructure;
     GPIO_InitTypeDef GPIO_InitStructure;
	
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); //tim3 
	
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	   
	
	  TIM_DeInit(TIM3);
	  TIM_TimeBaseStructure.TIM_Prescaler = 71;  //1M
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseStructure.TIM_Period = 19999;   //����    1000000/20000 = 50hz = 20ms T
    TIM_TimeBaseStructure.TIM_ClockDivision =0x00;
	  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 1500;                                //ռ�ձ�1000~2000
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	
	  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
		
		TIM_ARRPreloadConfig(TIM3, ENABLE);
	  TIM_Cmd(TIM3, ENABLE);
	
}
/*  pwm output
 *  @ HM
 *  @ 2015.09.25
 *  @ gxms0904hm@163.com
 *  @ funtion: configuration the timer for PWM output
 *  @ arg:     NONE
 *  @ retrun : NONE
 */
void tim4_config(void)
{
	  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	  TIM_OCInitTypeDef TIM_OCInitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;
	
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //tim4

		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	  TIM_DeInit(TIM4);
	  TIM_TimeBaseStructure.TIM_Prescaler = 71;  //1M
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseStructure.TIM_Period = 2499;   //����    1000000/2500 = 400hz = 2.5ms T
    TIM_TimeBaseStructure.TIM_ClockDivision =0x00;
	  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 1500;                                //ռ�ձ�1000~2000
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
		
		TIM_ARRPreloadConfig(TIM4, ENABLE);
	  TIM_Cmd(TIM4, ENABLE);
}

/*  pwm config
 *  @ HM
 *  @ 2015.09.25
 *  @ gxms0904hm@163.com
 *  @ funtion: configuration the timer for PWM 
 *  @ arg:     NONE
 *  @ retrun : NONE
 */
void Timer_Config(void)
{
	
	tim2_config();	
	tim3_config();
	tim4_config();
}
/*  TIM1 interrupt
 *  @ HM
 *  @ 2015.09.25
 *  @ gxms0904hm@163.com
 *  @ funtion: 
 *  @ arg:     NONE
 *  @ retrun : NONE
 */
unsigned int Rise[4];
unsigned int Drop[4];
unsigned int RC_PWM_in[4];
unsigned int RC[8];
void TIM1_CC_IRQHandler(void)//���벶׽
{
	
	  static int i=0;
    if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)   //����1���������¼�
		{	
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); //����жϱ�־λ
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10) == 0) 
			{
				  TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Rising);		//CC1P=1 ����Ϊ�½��ز���
          Drop[0]=TIM_GetCapture3(TIM1);
      }
			else 
			{
				  TIM_OC3PolarityConfig(TIM1,TIM_ICPolarity_Falling); //CC1P=0 ����Ϊ�����ز���
          Rise[0]=TIM_GetCapture3(TIM1);
				  if(Drop[0]>Rise[0])  RC_PWM_in[0] = 65535-Drop[0] + Rise[0];
					else 	               RC_PWM_in[0] = Rise[0] - Drop[0];
				
				
				  if(RC_PWM_in[0]<=2500)
					{
						 RC[i] = RC_PWM_in[0];
						 i++;
          }
					else
					{
						 i=0;
          }
				
      }			
		}	
}






#endif

















