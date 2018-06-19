
#include "stm32f10x.h"
#include "sys.h"
#include "i2c.h"
#include "mpu6050.h"
#include "senser.h"
#include "Parameter.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "portable.h"
#include "FreeRTOSConfig.h"


u8 mpu6050_buffer[14];

void MPU6050_Read(void)
{
	IIC_Read_nByte(devAddr,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
	
	Senser.AHRS.acc.raw.x = -((mpu6050_buffer[0] << 8) | mpu6050_buffer[1]);
	Senser.AHRS.acc.raw.y =  ((mpu6050_buffer[2] << 8) | mpu6050_buffer[3]);
	Senser.AHRS.acc.raw.z =  ((mpu6050_buffer[4] << 8) | mpu6050_buffer[5]);
	
	Senser.AUX.Temperature.raw = ((mpu6050_buffer[6] << 8) | mpu6050_buffer[7]);
	
	Senser.AHRS.gyro.raw.x =   ((mpu6050_buffer[8] << 8)  | mpu6050_buffer[9] );
	Senser.AHRS.gyro.raw.y = -(((mpu6050_buffer[10] << 8) | mpu6050_buffer[11]));
	Senser.AHRS.gyro.raw.z = -(((mpu6050_buffer[12] << 8) | mpu6050_buffer[13]));
	
	MPU6050_GetOffset();
	
}





s32 sum_temp[7]={0,0,0,0,0,0,0};
u32 acc_sum_cnt = 0,gyro_sum_cnt = 0;
void MPU6050_GetOffset(void)
{
	   if(Senser.AHRS.gyro.isGetOffset)
		 {
			  	gyro_sum_cnt++;
					sum_temp[0] += Senser.AHRS.gyro.raw.x;
					sum_temp[1] += Senser.AHRS.gyro.raw.y;
					sum_temp[2] += Senser.AHRS.gyro.raw.z;

					if(Senser.AHRS.gyro.isGetOffsetOK)
					{
						Senser.AHRS.gyro.bias.x = (float)sum_temp[0]/gyro_sum_cnt;
						Senser.AHRS.gyro.bias.y = (float)sum_temp[1]/gyro_sum_cnt;
						Senser.AHRS.gyro.bias.z = (float)sum_temp[2]/gyro_sum_cnt;
						gyro_sum_cnt =0;
						
						sum_temp[0] = 0;
						sum_temp[1] = 0;
						sum_temp[2] = 0;
						Senser.AHRS.gyro.isGetOffset = 0;
						Senser.AHRS.gyro.isGetOffsetOK = 0;
						
						Parameter_SaveOffset();
					}
	
     }
		 
		 if(Senser.AHRS.acc.isGetOffset)
		 {
			  	acc_sum_cnt++;
					sum_temp[4] += Senser.AHRS.acc.raw.x;
					sum_temp[5] += Senser.AHRS.acc.raw.y;
					sum_temp[6] += Senser.AHRS.acc.raw.z;

					if(Senser.AHRS.acc.isGetOffsetOK)
					{
						Senser.AHRS.acc.bias.x = (float)sum_temp[4]/acc_sum_cnt;
						Senser.AHRS.acc.bias.y = (float)sum_temp[5]/acc_sum_cnt;
						Senser.AHRS.acc.bias.z = (float)sum_temp[6]/acc_sum_cnt - 16384.0f;//16384是一个G的值
						acc_sum_cnt =0;
						
						sum_temp[4] = 0;
						sum_temp[5] = 0;
						sum_temp[6] = 0;
						
						Senser.AHRS.acc.isGetOffset = 0;
						Senser.AHRS.acc.isGetOffsetOK = 0;
						
						Parameter_SaveOffset();
					}
     }
}



/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
失败为0
*******************************************************************************/ 
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
	u8 b;
	IIC_Read_1Byte(dev, reg, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	IIC_Write_1Byte(dev, reg, b);
}
/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
reg	   寄存器地址
bitStart  目标字节的起始位
length   位长度
data    存放改变目标字节位的值
返回   成功 为1 
失败为0
*******************************************************************************/ 
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{
	
	u8 b,mask;
	IIC_Read_1Byte(dev, reg, &b);
	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	b &= mask;
	b |= data;
	IIC_Write_1Byte(dev, reg, b);
}

/**************************实现函数********************************************
*函数原型:		
*功　　能:	    设置 采样率
*******************************************************************************/
void MPU6050_set_SMPLRT_DIV(uint16_t hz)
{
	IIC_Write_1Byte(devAddr, MPU6050_RA_SMPLRT_DIV,1000/hz - 1);
		//I2C_Single_Write(devAddr,MPU_RA_SMPLRT_DIV, (1000/sample_rate - 1));
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
	IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
	
}
/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
	IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
	IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
enabled =1   睡觉
enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_setDLPF(uint8_t mode)
{
	IICwriteBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/



unsigned char MPU6050_Init(void)
{
	unsigned char who_am_I = 0;
	IIC_Read_1Byte(devAddr,MPU6050_RA_WHO_AM_I,&who_am_I);
	
	if(who_am_I != devAddr)
	{
		 return 1;
  }
	
	MPU6050_setSleepEnabled(0); //进入工作状态

	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟  0x6b   0x01

	MPU6050_set_SMPLRT_DIV(1000);

	
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500);//陀螺仪最大量程 +-500度每秒

	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G

	MPU6050_setDLPF(MPU6050_DLPF_BW_5);

	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C

	MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L

	return 0;
}



