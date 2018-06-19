
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
	
	   DMA_Cmd(DMA1_Channel1, ENABLE);//使能DMA
	
}
#define N 5 //每通道采50次
#define M 2 //2个通道
vu16 AD_Value[N][M];//用来存放ADC转换结果，也是DMA的目标地址
vu16 After_filter[M];//用来存放求平均值之后的结果

void ADC_DMA_Configuration(void)
{
			DMA_InitTypeDef DMA_InitStructure;
	
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//使能DMA时钟
	
			DMA_DeInit(DMA1_Channel1);//将DMA的通道1寄存器重设为缺省值
			DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;//DMA外设ADC基地址
	    DMA_InitStructure.DMA_MemoryBaseAddr     = (u32)&AD_Value;//DMA内存基地址
			DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;//内存作为数据传输的目的地
	    DMA_InitStructure.DMA_BufferSize         = N*M;//DMA通道的DMA缓存的大小
			DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;//外设地址寄存器不变
	    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;//内存地址寄存器递增
	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//数据宽度为16位
			DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;//数据宽度为16位
			DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;//工作在循环缓存模式
			DMA_InitStructure.DMA_Priority           = DMA_Priority_High;//DMA通道x拥有高优先级
			DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;//DMA通道x没有设置为内存到内存传输
			DMA_Init(DMA1_Channel1,&DMA_InitStructure);//根据DMA_InitStruct中指定的参数初始化DMA的通道
	
	    DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);//使能传输完成中断
}


//中断处理函数
float Pressure_r;
void  DMA1_Channel1_IRQHandler(void)
{
   if(DMA_GetITStatus(DMA1_IT_TC1)!=RESET)
	 {
     //自己的中断处理代码 但是记住程序不要太复杂  最好不要超过中断时间
		 //滤波
		  ADC_Filter();
		 
		  //空速数据
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




