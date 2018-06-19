/* HMC5983L.c file
��д�ߣ�lisn3188
��ַ��www.chiplab7.net
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2013-05-25
���ԣ� ���������ڵ���ʵ���ҵ�mini INS/GPS����ɲ���
���ܣ�
�ṩHMC5983L ��ʼ�� ��ȡ�����Ƶ�ǰADCת�����
------------------------------------
 */

#include "HMC5983L.h"


float HMC5983_lastx,HMC5983_lasty,HMC5983_lastz;

int16_t  HMC5983_FIFO[3][11]; //�������˲�
void HMC5983_getRaw(int16_t *x,int16_t *y,int16_t *z);

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   unsigned char HMC5983_IS_newdata(void)
*��������:	   ��ȡDRDY ���ţ��ж��Ƿ������һ��ת��
 Low for 250 ��sec when data is placed in the data output registers. 
���������  ��
���������  ������ת���������1  ������� 0
*******************************************************************************/
unsigned char HMC5983_IS_newdata(void)
{
 	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==Bit_SET){
	  return 1;
	 }
	 else return 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC5983_FIFO_init(void)
*��������:	   ������ȡ100�����ݣ��Գ�ʼ��FIFO����
���������  ��
���������  ��
*******************************************************************************/
void HMC5983_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC5983_getRaw(&temp[0],&temp[1],&temp[2]);
  delay_us(200);  //��ʱ�ٶ�ȡ����
//  LED_Change(); //LED��˸
  }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void  HMC5983_newValues(int16_t x,int16_t y,int16_t z)
*��������:	   ����һ�����ݵ�FIFO����
���������  �������������Ӧ��ADCֵ
���������  ��
*******************************************************************************/
void  HMC5983_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<10;i++){
		HMC5983_FIFO[0][i-1]=HMC5983_FIFO[0][i];
		HMC5983_FIFO[1][i-1]=HMC5983_FIFO[1][i];
		HMC5983_FIFO[2][i-1]=HMC5983_FIFO[2][i];
	}

	HMC5983_FIFO[0][9]=x;
	HMC5983_FIFO[1][9]=y;
	HMC5983_FIFO[2][9]=z;

	sum=0;
	for(i=0;i<10;i++){	//ȡ�����ڵ�ֵ���������ȡƽ��
   		sum+=HMC5983_FIFO[0][i];
	}
	HMC5983_FIFO[0][10]=sum/10;	//��ƽ��ֵ����

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5983_FIFO[1][i];
	}
	HMC5983_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5983_FIFO[2][i];
	}
	HMC5983_FIFO[2][10]=sum/10;
} //HMC5983_newValues

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void HMC5983_writeReg(unsigned char reg, unsigned char val)
*��������:	   дHMC5983L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void HMC5983_writeReg(unsigned char reg, unsigned char val) {
  IIC_Write_1Byte(HMC5983_ADDR,reg,val);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5983_getRaw(int16_t *x,int16_t *y,int16_t *z)
*��������:	   дHMC5983L�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ��
*******************************************************************************/
void HMC5983_getRaw(int16_t *x,int16_t *y,int16_t *z) {
   unsigned char vbuff[6];
   vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
   IIC_Read_nByte(HMC5983_ADDR,HMC5983_R_XM,6,vbuff);
   HMC5983_newValues(((int16_t)vbuff[0] << 8) | vbuff[1],((int16_t)vbuff[4] << 8) | vbuff[5],((int16_t)vbuff[2] << 8) | vbuff[3]);
   *x = HMC5983_FIFO[0][10];
   *y = HMC5983_FIFO[1][10];
   *z = HMC5983_FIFO[2][10];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5983_getValues(int16_t *x,int16_t *y,int16_t *z)
*��������:	   ��ȡ �����Ƶĵ�ǰADCֵ
���������    �������Ӧ�����ָ��	
���������  ��
*******************************************************************************/
void HMC5983_getlastValues(int16_t *x,int16_t *y,int16_t *z) {
  *x = HMC5983_FIFO[0][10];
  *y = HMC5983_FIFO[1][10]; 
  *z = HMC5983_FIFO[2][10]; 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5983_mgetValues(float *arry)
*��������:	   ��ȡ У����� ������ADCֵ
���������    �������ָ��	
���������  ��
*******************************************************************************/
void HMC5983_mgetValues(float *arry) {
  int16_t xr,yr,zr;
  HMC5983_getRaw(&xr, &yr, &zr);
  arry[0]= HMC5983_lastx=(float)(xr);
  arry[1]= HMC5983_lasty=(float)(yr);
  arry[2]= HMC5983_lastz=(float)(zr);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5983_mgetValues(float *arry)
*��������:	   ��ȡ У����� ������ADCֵ
���������    �������ָ��	
���������  ��
*******************************************************************************/
void HMC5983_mgetValuesint(int16_t *arry) {
  int16_t xr,yr,zr;
  HMC5983_getRaw(&xr, &yr, &zr);
  arry[0]= HMC5983_lastx=(xr);
  arry[1]= HMC5983_lasty=(yr);
  arry[2]= HMC5983_lastz=(zr);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5983_setGain(unsigned char gain)
*��������:	   ���� 5883L������
���������     Ŀ������ 0-7
���������  ��
*******************************************************************************/
void HMC5983_setGain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7) return;
  HMC5983_writeReg(HMC5983_R_CONFB, gain << 5);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5983_setMode(unsigned char mode)
*��������:	   ���� 5883L�Ĺ���ģʽ
���������     ģʽ
���������  ��
*******************************************************************************/
void HMC5983_setMode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  HMC5983_writeReg(HMC5983_R_MODE, mode);
  delay_us(100);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5983_init(u8 setmode)
*��������:	   ���� 5883L�Ĺ���ģʽ
���������     ģʽ
���������  ��
*******************************************************************************/
void HMC5983_init(u8 setmode) {

  if (setmode) {
    HMC5983_setMode(0);
  }

  HMC5983_writeReg(HMC5983_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
  HMC5983_writeReg(HMC5983_R_CONFB, 0xA0);
  HMC5983_writeReg(HMC5983_R_MODE, 0x00);

}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5983_setDOR(unsigned char DOR)
*��������:	   ���� 5883L�� �����������
���������     ����ֵ
0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 75Hz  
���������  ��
*******************************************************************************/
void HMC5983_setDOR(unsigned char DOR) {
  if (DOR>6) return;
  HMC5983_writeReg(HMC5983_R_CONFA,DOR<<2);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5983_getID(char id[3])
*��������:	   ��ȡоƬ��ID
���������     	ID��ŵ�����
���������  ��
*******************************************************************************/
void HMC5983_getID(unsigned char *id) 
{
      IIC_Read_1Byte(HMC5983_ADDR,HMC5983_R_IDA,&id[0]);  
      IIC_Read_1Byte(HMC5983_ADDR,HMC5983_R_IDB,&id[1]);
      IIC_Read_1Byte(HMC5983_ADDR,HMC5983_R_IDC,&id[2]);
}   // getID().

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC5983L_SetUp(void)
*��������:	   ��ʼ�� HMC5983L ʹ֮�������״̬
���������     	
���������  ��
*******************************************************************************/
void HMC5983L_SetUp(void)
{ 
  HMC5983_init(0); // Don't set mode yet, we'll do that later on.
  HMC5983_setMode(0);
  HMC5983_setDOR(6);  //75hz ������
  HMC5983_FIFO_init();
}


//------------------End of File----------------------------
