





#include "i2c1.h"
#include "sys.h"
#include "stm32f10x_conf.h"

// // IIC��ʱ
// // static void IIC_Delay() 
// // {
// // 	volatile int i = 50;
// // 	while(i--);
// // }

//��ʼ��IIC
void I2C1_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

        //��һ����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
// 	I2C_DeInit(I2C1);
	I2C_InitStructure.I2C_ClockSpeed         =  100000;    
	I2C_InitStructure.I2C_Mode               =  I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle          =  I2C_DutyCycle_2; 
	I2C_InitStructure.I2C_OwnAddress1        =  0XF0;
	I2C_InitStructure.I2C_Ack                =  I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress=  I2C_AcknowledgedAddress_7bit; 
	I2C_DeInit(I2C1);
	I2C_Init(I2C1,&I2C_InitStructure);
	//�ڶ������Ƚ�Pins����Ϊͨ��������ø�
	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin   = (GPIO_Pin_6|GPIO_Pin_7);
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);
	//������������ΪI2C����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_InitStructure.GPIO_Pin   = (GPIO_Pin_6|GPIO_Pin_7);
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_AF_OD ;
	GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);         

}


// // IICдһ���ֽ�����
// u8 I2C1_Write_OneByte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
// {
// 	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
// 	
//   /* 1.��ʼ */
//   I2C_GenerateSTART(I2C1, ENABLE);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

//   /* 2.�豸��ַ/д */
//   I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Transmitter);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

//   /* 3.���ݵ�ַ */
//   I2C_SendData(I2C1,REG_Address);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//   /* 4.дһ�ֽ����� */
//   I2C_SendData(I2C1, REG_data);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//   /* 5.ֹͣ */
//   I2C_GenerateSTOP(I2C1, ENABLE);
// 	
// 	return 0;
// }
// static void I2C_AcknowledgePolling(u8 SlaveAddress)
// {
//     do{
//         I2C_GenerateSTART(I2C1,ENABLE);
//         I2C_ReadRegister(I2C1,I2C_Register_SR1);//���ADDRλ

//         I2C_Send7bitAddress(I2C1,SlaveAddress,I2C_Direction_Transmitter);
//     }while(!(I2C_ReadRegister(I2C1,I2C_Register_SR1)&0x0002));
//  
//     I2C_ClearFlag(I2C1,I2C_FLAG_AF);
// }

// // IIC��1�ֽ�����
// u8 I2C1_Read_OneByte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data)
// {      		
//  	I2C_GenerateSTOP(I2C1, ENABLE);
// //   while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)){I2C_GenerateSTOP(I2C1, ENABLE);};
// //   I2C_GenerateSTOP(I2C1, ENABLE);
// 	I2C_AcknowledgeConfig(I2C1, ENABLE);                             //����Ӧ��
//  	I2C_AcknowledgePolling(SlaveAddress<<1);
//   /* 1.��ʼ */
//   I2C_GenerateSTART(I2C1, ENABLE);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

//   /* 2.�豸��ַ/д */
//   I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Transmitter);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

// 	I2C_Cmd(I2C1, ENABLE);
//   /* 3.���ݵ�ַ */
//   I2C_SendData(I2C1,REG_Address);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//   /* 4.���¿�ʼ */
//   I2C_GenerateSTART(I2C1, ENABLE);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

//   /* 5.�豸��ַ/�� */
//   I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Receiver);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

//   /* 6.��һ�ֽ����� */
//   I2C_AcknowledgeConfig(I2C1, DISABLE);                              //������Ӧ��
//   while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
//   *REG_data = I2C_ReceiveData(I2C1);                                     //��ȡ����

//   /* 7.ֹͣ */
//   I2C_GenerateSTOP(I2C1, ENABLE);
// 	
// 	return 0;
// }	

// // IICдn�ֽ�����
// u8 I2C1_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
// {	
// 	u8 count = 0;
// 	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
// 	/*  START */
// 	I2C_GenerateSTART(I2C1,ENABLE);
// 	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
// 	/*  SlaveAddress & WRITE */
// 	I2C_Send7bitAddress(I2C1,SlaveAddress<<1,I2C_Direction_Transmitter);
// 	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){};
// 	/*  REG_Address  */
//   I2C_SendData(I2C1,REG_Address);	
// 	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
// 	/*  DATA */
// 	for(count = 0;count<(len-1);count++)
// 	{
// 			I2C_SendData(I2C1,*buf++);	
// 			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
// 	}
// 	/*  ���������һ���ֽڣ���ֹHardFault */
// 	I2C_SendData(I2C1,*buf);	
// 	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
// 	/*  STOP  */
// 	I2C_GenerateSTOP(I2C1,ENABLE);
// // 	I2C_GenerateSTART(I2C1,DISABLE);

// 	return 0;
// }

// // IIC��n�ֽ�����
// u8 I2C1_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
// {	
//   uint16_t cnt;
//   while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
//   I2C_AcknowledgeConfig(I2C1, ENABLE);
//   /* 1.��ʼ */
//   I2C_GenerateSTART(I2C1, ENABLE);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

//   /* 2.�豸��ַ/д */
//   I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Transmitter);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//   
// 	I2C_Cmd(I2C1, ENABLE);
//   /* 3.���ݵ�ַ */
//   I2C_SendData(I2C1,REG_Address);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//   /* 4.���¿�ʼ */
//   I2C_GenerateSTART(I2C1, ENABLE);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

//   /* 5.�豸��ַ/�� */
//   I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Receiver);
//   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

//   /* 6.�����ֽ����� */
//   for(cnt=0; cnt<(len-1); cnt++)
//   {
//     I2C_AcknowledgeConfig(I2C1, ENABLE);                             //����Ӧ��
//     while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
//     *buf = I2C_ReceiveData(I2C1);                                  //������ȡ(Length-1)�ֽ�
//     buf++;
//   }
//   I2C_AcknowledgeConfig(I2C1, DISABLE);                              //��ȡ���1�ֽ�(������Ӧ��)
//   while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
//   *buf = I2C_ReceiveData(I2C1);                                    //��ȡ����

//   /* 7.ֹͣ */
//   I2C_GenerateSTOP(I2C1, ENABLE);
// 	
// 	return 0;
// }






// #include "stm32f10x_conf.h"
// #include "stm32f10x.h"
// // #include "24c02.h"
// #include "sys.h"
// // #include "i2c1.h"

//ע�⣬���eeprom��atmelH416 256K bit = 32kbyte,�����Ҫ�����ֽڴ洢��ַ



// EEPROMдһ���ֽ�����
u8 I2C1_Write_1Byte(u8 SlaveAddress,unsigned short REG_Address,u8 REG_data)
{
// 	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)){};

  /* 1.��ʼ */
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* 2.�豸��ַ/д */
  I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* 3.���ݵ�ַ */
  I2C_SendData(I2C1, (uint8_t)(REG_Address&0x00FF));
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* 4.дһ�ֽ����� */
  I2C_SendData(I2C1, REG_data);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* 5.ֹͣ */
  I2C_GenerateSTOP(I2C1, ENABLE);
	
	return 0;
}

static void I2C_AcknowledgePolling(u8 SlaveAddress)
{
    do{
        I2C_GenerateSTART(I2C1,ENABLE);
        I2C_ReadRegister(I2C1,I2C_Register_SR1);//���ADDRλ

        I2C_Send7bitAddress(I2C1,SlaveAddress,I2C_Direction_Transmitter);
    }while(!(I2C_ReadRegister(I2C1,I2C_Register_SR1)&0x0002));
 
    I2C_ClearFlag(I2C1,I2C_FLAG_AF);
}




// EEPROM��1�ֽ�����
u8 I2C1_Read_1Byte(u8 SlaveAddress,unsigned short REG_Address,u8 *REG_data)
{      		
//  	I2C_GenerateSTOP(I2C1, ENABLE);
//   while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) 
// 	{
// // 		   I2C_ReadRegister(I2C1,I2C_Register_CR2);
//   }
	
 	I2C_AcknowledgePolling(SlaveAddress<<1);
  /* 1.��ʼ */
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* 2.�豸��ַ/д */
  I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* 3.���ݵ�ַ */
  I2C_SendData(I2C1, (uint8_t)(REG_Address&0x00FF));
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* 4.���¿�ʼ */
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* 5.�豸��ַ/�� */
  I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* 6.��һ�ֽ����� */
  I2C_AcknowledgeConfig(I2C1, DISABLE);                              //������Ӧ��
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
  *REG_data = I2C_ReceiveData(I2C1);                                     //��ȡ����

  /* 7.ֹͣ */
  I2C_GenerateSTOP(I2C1, ENABLE);
	
	return 0;
}	


//ʹ������������������д�ٶȿ�һЩ,���ǣ�ÿ��ֻ�ܶ�д64�ֽ�
u8 I2C1_Write_nByte(u8 SlaveAddress,unsigned short address,unsigned char number,unsigned char *data)
{
	uint16_t cnt;

//   while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  /* 1.��ʼ */
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* 2.�豸��ַ/д */
  I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* 3.���ݵ�ַ */
  I2C_SendData(I2C1, (uint8_t)(address&0x00FF));
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* 4.����д���� */
  for(cnt=0; cnt<(number-1); cnt++)
  {
    I2C_SendData(I2C1, *data);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    data++;
  }
  I2C_SendData(I2C1, *data);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* 5.ֹͣ */
  I2C_GenerateSTOP(I2C1, ENABLE);
	
	return 0;
}


u8 I2C1_Read_nByte(u8 SlaveAddress,unsigned short address,unsigned char number,unsigned char *buffer)
{
  uint16_t cnt;

//   while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  /* 1.��ʼ */
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* 2.�豸��ַ/д */
  I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* 3.���ݵ�ַ */
  I2C_SendData(I2C1, (uint8_t)(address&0x00FF));
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* 4.���¿�ʼ */
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* 5.�豸��ַ/�� */
  I2C_Send7bitAddress(I2C1, SlaveAddress<<1, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* 6.�����ֽ����� */
  for(cnt=0; cnt<(number-1); cnt++)
  {
    I2C_AcknowledgeConfig(I2C1, ENABLE);                             //����Ӧ��
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
    *buffer = I2C_ReceiveData(I2C1);                                  //������ȡ(Length-1)�ֽ�
    buffer++;
  }
  I2C_AcknowledgeConfig(I2C1, DISABLE);                              //��ȡ���1�ֽ�(������Ӧ��)
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
  *buffer = I2C_ReceiveData(I2C1);                                    //��ȡ����

  /* 7.ֹͣ */
  I2C_GenerateSTOP(I2C1, ENABLE);
	
	return 0;
}







// void I2C1_EV_IRQ(void)     //I2C1 Event Interrupt 
// {
// 	    u16 clear = 0;
// 	  
// 	    if(I2C1 -> SR1 & 1<<0 )          //�ѷ�����ʼ����,д���ݼĴ����Ĳ����������λ
// 	    {
// 	        printf("\r\n I2C1 Start .. \r\n");
// 	  
// 	        switch(go)
// 	        {
// 	            case 0:{ 
// 	                I2c_Write(ADDRS_W);        //д��ӻ���ַ,дָ�������ַ
// 	                break;
// 	            }
// 	            case 1:{
// 	                I2c_Write(ADDRS_W);        //д��ӻ���ַ,дָ�������ַ
// 	                break;
// 	            }
// 	            case 2:{
// 	                I2c_Write(ADDRS_R);        //д��ӻ���ַ,�����ݲ�����ַ
// 	                break;
// 	           }
// 	        }
// 	  
// 	    }
// 	  
// 	    if(I2C1 -> SR1 & 1<<1 )        //�ӻ���ַ�ѷ���
// 	    {
// 	        printf("\r\n I2C1 has send address .. \r\n");
// 	        clear = I2C1 -> SR2; //��ȡSR2���������λ�ж�
// 	  
// 	        switch(go)
// 	        {
//              case 0:{ 
// 	                I2c_Write(0x01);    //д���д���EEPROM��Ԫ��ַ
// 	                break;
// 	            }
// 	  
// 	            case 1:{
// 	                I2c_Write(0x01);    //д���д���EEPROM��Ԫ��ַ
// 	                break;
// 	            }
// 	            case 2:{
// 	                delay(100000);
// 	                printf("\r\n Read 0x%X from At24c02 ,Address 0x01 ..  \r\n",I2c_Read());
// 	                I2c_Stop();
// 	                break;
// 	           }
// 	        }
// 	  
// 	    }
// 	  
// 	    if(I2C1 -> SR1 & 1<<2 )        //�ֽڷ��ͽ���  ���͵�ַ�ֽ�ʱ�����������ж�
// 	    {
// 	          
// 	        //printf("\r\n I2C1 send byte success .. \r\n");
// 	        switch(go)
// 	        {
// 	            case 0:{ 
// 	                I2c_Write(0x86);            //д������
// 	                printf("\r\n Write 0x%X to At24c02 ,Address 0x01 ..  \r\n",0x86);           
// 	                //I2c_Stop();
// 	      
// 	                delay(10000);
// 	                go = 1;
// 	                I2c_Start(); 
// 	                break;
// 	            }
// 	  
// 	            case 1:{
// 	  
// 	                delay(10000);
// 	                go = 2;
// 	                I2c_Start();
// 	                break;
// 	            }
// 	            case 2:{
// 	  
// 	                break;
// 	           }
// 	        }
// 	  
// 	    }

// 	  
// 	    //I2C1 -> CR2 &= ~(1<<9);          //�¼��жϹر�
// 	}
// 	  
// 	void I2C1_ER_IRQ(void)       //I2C1 Error Interrupt 
// 	{
//   
// 	  
// 	    if(I2C1->SR1 & 1<<10)          //Ӧ��ʧ��
//     {
// 	        printf("\r\n ACK ERROR .. \r\n");
// 	  
// 	        I2C1->SR1 &=~(1<<10);      //����ж�
// 	    }
// 	  
// 	    if(I2C1->SR1 & 1<<14)          //��ʱ
// 	    {
// 	        printf("\r\n Timeout .. \r\n");
// 	  
// 	        I2C1->SR1 &=~(1<<14);      //����ж�
// 	    }
// 	  
// 	    if(I2C1->SR1 & 1<<11)          //����/Ƿ��
// 	    {
// 	        printf("\r\n Overrun/Underrun .. \r\n");
//         I2C1->SR1 &=~(1<<11);      //����ж�
//     }
//   
// 	    if(I2C1->SR1 & 1<<9)           //�ٲö�ʧ
// 	    {
// 	        printf("\r\n Arbitration lost .. \r\n");
// 	        I2C1->SR1 &=~(1<<9);       //����ж�
// 	    }
//   
//     if(I2C1->SR1 & 1<<8)           //���߳���
// 	    {
//         printf("\r\n Bus error .. \r\n");
// 	        I2C1->SR1 &=~(1<<8);       //����ж�
// 	    }

// 		}






