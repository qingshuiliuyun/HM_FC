
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "adc.h"
#include "mpxv7002.h"
#include "senser.h"

#ifdef ADC_EN

void ADC_Configuration(void)
{
	   GPIO_InitTypeDef GPIO_InitStructure;
	   ADC_InitTypeDef ADC_InitStructure;
	
	   NVIC_InitTypeDef NVIC_InitStructure;  
	
     NVIC_InitStructure.NVIC_IRQChannel =DMA1_Channel1_IRQn;  
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure); 
	
	   ADC_DMA_Configuration();
	
     RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | 
	                          RCC_APB2Periph_ADC1 |
                          	RCC_APB2Periph_GPIOA, ENABLE);
	
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
     GPIO_Init(GPIOA, &GPIO_InitStructure);
	
 	   ADC_DeInit(ADC1);
	   ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	   ADC_InitStructure.ADC_ScanConvMode = ENABLE; 
		 ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		 ADC_InitStructure.ADC_NbrOfChannel = 2;
		 ADC_Init(ADC1, &ADC_InitStructure); 
		 ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);
		 ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_239Cycles5);
		
		
		 ADC_DMACmd(ADC1,ENABLE);
		
		 ADC_Cmd(ADC1, ENABLE);
		 
		 ADC_ResetCalibration(ADC1); 
		 while(ADC_GetResetCalibrationStatus(ADC1));
		 ADC_StartCalibration(ADC1);
		 while(ADC_GetCalibrationStatus(ADC1));
		 ADC_SoftwareStartConvCmd(ADC1, ENABLE); //start convering the ad value
	
	   DMA_Cmd(DMA1_Channel1, ENABLE);//ʹ��DMA
	
}
#define N 5 //ÿͨ����50��
#define M 2 //2��ͨ��
vu16 AD_Value[N][M];//�������ADCת�������Ҳ��DMA��Ŀ���ַ
vu16 After_filter[M];//���������ƽ��ֵ֮��Ľ��

void ADC_DMA_Configuration(void)
{
			DMA_InitTypeDef DMA_InitStructure;
	
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//ʹ��DMAʱ��
	
			DMA_DeInit(DMA1_Channel1);//��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
			DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;//DMA����ADC����ַ
	    DMA_InitStructure.DMA_MemoryBaseAddr     = (u32)&AD_Value;//DMA�ڴ����ַ
			DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;//�ڴ���Ϊ���ݴ����Ŀ�ĵ�
	    DMA_InitStructure.DMA_BufferSize         = N*M;//DMAͨ����DMA����Ĵ�С
			DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;//�����ַ�Ĵ�������
	    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;//�ڴ��ַ�Ĵ�������
	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���ݿ��Ϊ16λ
			DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;//���ݿ��Ϊ16λ
			DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;//������ѭ������ģʽ
			DMA_InitStructure.DMA_Priority           = DMA_Priority_High;//DMAͨ��xӵ�и����ȼ�
			DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;//DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
			DMA_Init(DMA1_Channel1,&DMA_InitStructure);//����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��
	
	    DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);//ʹ�ܴ�������ж�
}


//�жϴ�����
float Pressure_r;
void  DMA1_Channel1_IRQHandler(void)
{
   if(DMA_GetITStatus(DMA1_IT_TC1)!=RESET)
	 {
     //�Լ����жϴ������ ���Ǽ�ס����Ҫ̫����  ��ò�Ҫ�����ж�ʱ��
		 //�˲�
		  ADC_Filter();
		 
		  //��������
		  MPXV7002_GetOffset(After_filter[1]);
 		  Senser.AUX.AirSpeed.raw = After_filter[1];
		  Senser.AUX.Battery.raw  = After_filter[0];
		 
      DMA_ClearITPendingBit(DMA1_IT_TC1);
   }
 }

void ADC_Filter(void)
{
	u8 i,j;
	float SUM[M];
	for(i=0;i<M;i++)
	{
		 for(j=0;j<N;j++)
		 {
			  SUM[i] += AD_Value[j][i];
     }
  }
	for(i=0;i<M;i++)
	{
			After_filter[i] = SUM[i]/N;
  }
}

	 
	
 






#endif




