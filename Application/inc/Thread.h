
#ifndef _THREAD_H_
#define _THREAD_H_


#include "stm32f10x.h"


extern float Rol,Pit,Yaw;


typedef struct {
   uint32_t ThreadCount_1ms;
	 uint32_t ThreadCount_2ms;
	 uint32_t ThreadCount_5ms;
	 uint32_t ThreadCount_10ms;
	 uint32_t ThreadCount_20ms;
	 uint32_t ThreadCount_50ms;
	 uint32_t ThreadCount_100ms;
	 uint32_t ThreadCount_200ms;
	 uint32_t ThreadCount_500ms;
	 uint32_t ThreadCount_1000ms;
}_thread;



extern _thread Thread;


void Thread_Event(void);
void Thread_Loop(void);


void Thread_1ms(void);
void Thread_2ms(void);
void Thread_5ms(void);
void Thread_10ms(void);
void Thread_20ms(void);
void Thread_50ms(void);
void Thread_100ms(void);
void Thread_200ms(void);
void Thread_500ms(void);
void Thread_1000ms(void);






#endif
