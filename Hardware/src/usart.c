
#include "include.h"
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "usart.h"
#include "protocol.h"
#include "sbus.h"
#include "UBLOX.h"

#ifdef USART_EN
void USART1_Config(uint32_t Baud)                                        
{
	        USART_InitTypeDef USART_InitStructure;
	        NVIC_InitTypeDef NVIC_InitStructure;
	        GPIO_InitTypeDef GPIO_InitStructure;
	
	        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);  
				  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);   
				  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); 
	
	
	        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);
	
	        GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;     
			    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
			    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;	
					GPIO_Init(GPIOA,&GPIO_InitStructure);
	
			    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;    
			    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;			     
			    GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	
	        USART_DeInit(USART1);
					USART_InitStructure.USART_BaudRate = Baud;
					USART_InitStructure.USART_WordLength = USART_WordLength_8b;
					USART_InitStructure.USART_StopBits = USART_StopBits_2;
					USART_InitStructure.USART_Parity = USART_Parity_Even;
					USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;				
					USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
					USART_Init(USART1, &USART_InitStructure);
	
	        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//使能接受中断       
	        USART_Cmd(USART1, ENABLE);
}

 u8 Tx1Buffer[256];
 u8 Tx1Counter=0;
 u8 count1=0; 

void Usart1_IRQ(void)
{
	u8 com_data;
  //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志

		com_data = USART1->DR;
 		SBUS_Prepare(com_data);
	}

 	//发送（进入移位）中断
 	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
 	{
 				
 		USART1->DR = Tx1Buffer[Tx1Counter++]; //写DR清除中断标志
           
 		if(Tx1Counter == count1)
 		{
 			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
 		}
 	}
}

 void Usart1_Send(unsigned char *DataToSend ,u8 data_num)
 {
 	u8 i;
 	for(i=0;i<data_num;i++)
 	{
 		Tx1Buffer[count1++] = *(DataToSend + i);
 	}

 	if(!(USART1->CR1 & USART_CR1_TXEIE))
 	{
 		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //打开发送中断
 	}
 }





void USART2_Config(uint32_t Baud)                                        
{
	        USART_InitTypeDef USART_InitStructure;
	        NVIC_InitTypeDef NVIC_InitStructure;
	        GPIO_InitTypeDef GPIO_InitStructure;
	
	        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);  
				  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);   
				  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); 
	
	
	        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);
	
	        GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;     
			    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
			    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;	
					GPIO_Init(GPIOA,&GPIO_InitStructure);
	
			    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;    
			    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;			     
			    GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	
	        USART_DeInit(USART2);
					
					USART_InitStructure.USART_BaudRate = Baud;
					USART_InitStructure.USART_WordLength = USART_WordLength_8b;
					USART_InitStructure.USART_StopBits = USART_StopBits_1;
					USART_InitStructure.USART_Parity = USART_Parity_No;
					USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;				
					USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
					USART_Init(USART2, &USART_InitStructure);
	
	        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//使能接受中断       
	        USART_Cmd(USART2, ENABLE);
}

u8 Tx2Buffer[256];
u8 Tx2Counter=0;
u8 count2=0; 

void Usart2_IRQ(void)
{
	u8 com_data;
  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
		com_data = com_data;
//		Usart3_Send(&com_data,1);
		
// 		Protocol_Prepare(com_data);
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = Tx2Buffer[Tx2Counter++]; //写DR清除中断标志
          
		if(Tx2Counter == count2)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
}

void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
	{
		Tx2Buffer[count2++] = *(DataToSend + i);
	}

	if(!(USART2->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //打开发送中断
	}
}




void USART3_Config(uint32_t Baud)                                        
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);   
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); 


	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;     
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;	
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;    
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;			     
	GPIO_Init(GPIOB,&GPIO_InitStructure);



	USART_DeInit(USART3);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,DISABLE); 
	USART_InitStructure.USART_BaudRate = Baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;				
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//使能接受中断       
	USART_Cmd(USART3, ENABLE);
}

u8 Tx3Buffer[256];
u8 Tx3Counter=0;
u8 count3=0; 

void Usart3_IRQ(void)
{
	u8 com_data;
  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		com_data = USART3->DR;
 		ublox_Prepare(com_data);
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = Tx3Buffer[Tx3Counter++]; //写DR清除中断标志
          
		if(Tx3Counter == count3)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
}

void Usart3_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
	{
		Tx3Buffer[count3++] = *(DataToSend + i);
	}

	if(!(USART3->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART3, USART_IT_TXE, ENABLE); //打开发送中断
	}
}












#endif




















