


#ifndef _LED_H_
#define _LED_H_




#define LED_EN



#ifdef LED_EN



#define  LED_RED    0
#define  LED_GREEN  1
#define  LED_BLUE   2
	
typedef struct {

	float RED;
	float GREEN;
	float BLUE;
	
}_LEDType;

extern _LEDType LED;



void LED_Config(void);
void LED_ON(unsigned char LED);
void LED_OFF(unsigned char LED);


void LED_Blink(float T);

void LED_SetRGB(unsigned char RED,unsigned char GREEN,unsigned char BLUE);
void LED_SetRED(unsigned char RED);
void LED_SetGREEN(unsigned char GREEN);
void LED_SetBLUE(unsigned char BLUE);



#endif



#endif






























