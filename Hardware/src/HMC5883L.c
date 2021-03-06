/* HMC5883L.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-04-25
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
提供HMC5883L 初始化 读取磁力计当前ADC转换结果
------------------------------------
 */

#include "HMC5883L.h"
#include "senser.h"
#include "parameter.h"
#include "sys.h"
#include "I2C.H"


float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;

int16_t  HMC5883_FIFO[3][11]; //磁力计滤波
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z);

/**************************实现函数********************************************
*函数原型:	   unsigned char HMC5883_IS_newdata(void)
*功　　能:	   读取DRDY 引脚，判断是否完成了一次转换
 Low for 250 μsec when data is placed in the data output registers. 
输入参数：  无
输出参数：  如果完成转换，则输出1  否则输出 0
*******************************************************************************/
unsigned char HMC5883_IS_newdata(void)
{
 	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==Bit_SET){
	  return 1;
	 }
	 else return 0;
}

/**************************实现函数********************************************
*函数原型:	   void HMC58X3_FIFO_init(void)
*功　　能:	   连续读取100次数据，以初始化FIFO数组
输入参数：  无
输出参数：  无
*******************************************************************************/
void HMC58X3_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC58X3_getRaw(&temp[0],&temp[1],&temp[2]);
//   Delay_us(200);  //延时再读取数据
  // LED_Change(); //LED闪烁
  }
}

/**************************实现函数********************************************
*函数原型:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
*******************************************************************************/
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<10;i++){
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}

	HMC5883_FIFO[0][9]=x;
	HMC5883_FIFO[1][9]=y;
	HMC5883_FIFO[2][9]=z;

	sum=0;
	for(i=0;i<10;i++){	//取数组内的值进行求和再取平均
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;	//将平均值更新

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;
} //HMC58X3_newValues

/**************************实现函数********************************************
*函数原型:	   void HMC58X3_writeReg(unsigned char reg, unsigned char val)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void HMC58X3_writeReg(unsigned char reg, unsigned char val) {
  IIC_Write_1Byte(HMC58X3_ADDR,reg,val);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) {
   unsigned char vbuff[6];
   vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
   IIC_Read_nByte(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);
   HMC58X3_newValues(((int16_t)vbuff[0] << 8) | vbuff[1],((int16_t)vbuff[4] << 8) | vbuff[5],((int16_t)vbuff[2] << 8) | vbuff[3]);
   *x = HMC5883_FIFO[0][10];
   *y = HMC5883_FIFO[1][10];
   *z = HMC5883_FIFO[2][10];
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   读取 磁力计的当前ADC值
输入参数：    三个轴对应的输出指针	
输出参数：  无
*******************************************************************************/
void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z) {
  *x = HMC5883_FIFO[0][10];
  *y = HMC5883_FIFO[1][10]; 
  *z = HMC5883_FIFO[2][10]; 
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_mgetValues(float *arry)
*功　　能:	   读取 校正后的 磁力计ADC值
输入参数：    输出数组指针	
输出参数：  无
*******************************************************************************/
void HMC58X3_mgetValues(float *arry) {
  int16_t xr,yr,zr;
  HMC58X3_getRaw(&xr, &yr, &zr);
  arry[0]= HMC5883_lastx=(float)(xr);
  arry[1]= HMC5883_lasty=(float)(yr);
  arry[2]= HMC5883_lastz=(float)(zr);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_setGain(unsigned char gain)
*功　　能:	   设置 5883L的增益
输入参数：     目标增益 0-7
输出参数：  无
*******************************************************************************/
void HMC58X3_setGain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7) return;
  HMC58X3_writeReg(HMC58X3_R_CONFB, gain << 5);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_setMode(unsigned char mode)
*功　　能:	   设置 5883L的工作模式
输入参数：     模式
输出参数：  无
*******************************************************************************/
void HMC58X3_setMode(unsigned char mode) {
  if (mode > 2) {
    return;
  }
  HMC58X3_writeReg(HMC58X3_R_MODE, mode);
//   Delay_us(100);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_init(u8 setmode)
*功　　能:	   设置 5883L的工作模式
输入参数：     模式
输出参数：  无
*******************************************************************************/
void HMC58X3_init(u8 setmode) {

  if (setmode) {
    HMC58X3_setMode(0);
  }

  HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
  HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
  HMC58X3_writeReg(HMC58X3_R_MODE,  0x00);

}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_setDOR(unsigned char DOR)
*功　　能:	   设置 5883L的 数据输出速率
输入参数：     速率值
0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 75Hz  
输出参数：  无
*******************************************************************************/
void HMC58X3_setDOR(unsigned char DOR) {
  if (DOR>6) return;
  HMC58X3_writeReg(HMC58X3_R_CONFA,DOR<<2);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getID(char id[3])
*功　　能:	   读取芯片的ID
输入参数：     	ID存放的数组
输出参数：  无
*******************************************************************************/
void HMC58X3_getID(unsigned char *id) 
{
      IIC_Read_1Byte(HMC58X3_ADDR,HMC58X3_R_IDA,&id[0]);  
      IIC_Read_1Byte(HMC58X3_ADDR,HMC58X3_R_IDB,&id[1]);
      IIC_Read_1Byte(HMC58X3_ADDR,HMC58X3_R_IDC,&id[2]);
}   // getID().

/**************************实现函数********************************************
*函数原型:	  void HMC5883L_SetUp(void)
*功　　能:	   初始化 HMC5883L 使之进入可用状态
输入参数：     	
输出参数：  无
*******************************************************************************/
void HMC5883L_SetUp(void)
{ 
  HMC58X3_init(0); // Don't set mode yet, we'll do that later on.
  HMC58X3_setMode(0);
  HMC58X3_setDOR(6);  //75hz 更新率
  HMC58X3_FIFO_init();
}


void HMC5883L_Read(void)
{
	  int16_t xr,yr,zr;
    HMC58X3_getRaw(&xr, &yr, &zr);
	
	  Senser.AHRS.mag_aux1.raw.x = xr;
	  Senser.AHRS.mag_aux1.raw.y = yr;
	  Senser.AHRS.mag_aux1.raw.z = zr;
	
	  HMC5883L_GetOffset();
}


void HMC5883L_GetOffset(void)
{
	  static _xyz_s	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 };
	  if(Senser.AHRS.mag_aux1.isGetOffset)
		{
			  if(ABS(Senser.AHRS.mag.raw.x)<400&&ABS(Senser.AHRS.mag.raw.y)<400&&ABS(Senser.AHRS.mag.raw.z)<400)
				{
					MagMAX.x = MAX(Senser.AHRS.mag_aux1.raw.x, MagMAX.x);
					MagMAX.y = MAX(Senser.AHRS.mag_aux1.raw.y, MagMAX.y);
					MagMAX.z = MAX(Senser.AHRS.mag_aux1.raw.z, MagMAX.z);
					
					MagMIN.x = MIN(Senser.AHRS.mag_aux1.raw.x, MagMIN.x);
					MagMIN.y = MIN(Senser.AHRS.mag_aux1.raw.y, MagMIN.y);
					MagMIN.z = MIN(Senser.AHRS.mag_aux1.raw.z, MagMIN.z);	
				}
				
				if(Senser.AHRS.mag_aux1.isGetOffsetOK)
				{
					
					Senser.AHRS.mag_aux1.bias.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				  Senser.AHRS.mag_aux1.bias.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				  Senser.AHRS.mag_aux1.bias.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
					
					Senser.AHRS.mag_aux1.isGetOffset = 0;
					Senser.AHRS.mag_aux1.isGetOffsetOK = 0;
					
					Parameter_SaveOffset();
				}
		}
}






//------------------End of File----------------------------
