






#ifndef  _TIM_H_
#define  _TIM_H_

#define TIM_EN


#ifdef TIM_EN

extern unsigned int RC_PWM_in[4];


void tim1_Config(void);
void tim2_config(void);
void tim3_config(void);
void tim4_config(void);
void Timer_Config(void);
#endif 




#endif







