/* GPS.c file
编写者：lisn3188
网址：www.chiplab7.net
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2013-05-25
测试： 本程序已在第七实验室的mini INS/GPS上完成测试

占用STM32 资源：
1. 使用UART3接收GPS数据
2. 确认GPS的波特率 在初始化时调用 Initial_UART3

功能：
接收并提取GPS数据。
------------------------------------
 */
#include "include.h"
#include "GPS.h"
#include "math.h"
#include "time.h"
#include "sys.h"
#include "matrix.h"
#include "ms5611.h"
#include "usart.h"


//
unsigned char isGPSOK = 0;
//接收缓冲区
volatile unsigned char GPS_buffer[256];
volatile unsigned char GPS_wr_index;
volatile unsigned char Frame_End,GPS_Data_Ready=0;
//----------------------------------
volatile unsigned char GPS_Satel[4];
volatile unsigned char GPS_Status;
volatile unsigned char GPS_Latitude[11],Lat;
volatile unsigned char GPS_Longitude[13],Lon;
volatile unsigned char GPS_Speed[7];
volatile unsigned char GPS_Course[7];
volatile unsigned char GPS_Height[9]; 
volatile unsigned char GPS_Time[8];
volatile unsigned char GPS_Date[8];
//----------------------------------
unsigned char GPS_STA_Num = 0,
			        GPS_Update = 0, 
			        GPS_Locked = 0;
float         GPS_Altitude = 0,
	            Speed_GPS = 0,
	            Course_GPS = 0,
		          Ve = 0,
		          Vn = 0;
double        Latitude_GPS = 0,
	            Longitude_GPS = 0;
		
//---------家点信息---------------------
void UBLOX_GPS_initial(void);
void UBLOX_GPS_10Hz(void);


void GPS_Send(u8 *data,u8 Len)//发送函数封装，使用函数式
{
	  Usart3_Send(data,Len);//发送出去
}

void GPS_Init(void)
{
	u8 i = 0;
	USART3_Config(9600);//初始化串口
	   
	for(i=0;i<10;i++)
	{
		  UBLOX_GPS_initial();  //配置ublox GPS  这个函数已经把GPS的通讯改成了115200L
  }
	USART3_Config(115200);//初始化串口
	
	UBLOX_GPS_10Hz(); //将GPS的发送速率改成5HZ
}



void UBLOX_GPS_10Hz(void){
	u8 index;  //以下指令只对 ublox GPS模块的设置
	u8 ublox_cmd[30]={0xB5,0x62,0x06,0x01,0x03,0x00,0xf0,0x00};

	index=3;
	ublox_cmd[index++]=0x08; //enable 10Hz 更新速度 10HZ
	ublox_cmd[index++]=0x06;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x64;
	ublox_cmd[index++]=0x00; 
	ublox_cmd[index++]=0x01;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x01;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x7A; 
	ublox_cmd[index++]=0x12;
 	GPS_Send(ublox_cmd,index);

}

/**************************实现函数********************************************
*函数原型:		void UBLOX_GPS_initial(void)
*功　　能:		初始化 UBLOX GPS 模块，以去掉一些没必要的数据，节省CPU的开支 
*******************************************************************************/
void UBLOX_GPS_initial(void){
	u8 index;  //以下指令只对 ublox GPS模块的设置
	u8 ublox_cmd[30]={0xB5,0x62,0x06,0x01,0x03,0x00,0xf0,0x00};

	index=7;
	ublox_cmd[index++]=0x01; //disable GPGLL
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xFB;
	ublox_cmd[index++]=0x11;
 	GPS_Send(ublox_cmd,index);

	index=7;
	ublox_cmd[index++]=0x02; //disable GPGSA
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xFC;
	ublox_cmd[index++]=0x13;
 	GPS_Send(ublox_cmd,index);
//  	Delay_ms(10);

	index=7;
	ublox_cmd[index++]=0x03; //disable GPGSV
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xFD;
	ublox_cmd[index++]=0x15;
	GPS_Send(ublox_cmd,index);
//  	Delay_ms(10);

	index=7;
	ublox_cmd[index++]=0x05; //disable GPVTG
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xFF;
	ublox_cmd[index++]=0x19;
	GPS_Send(ublox_cmd,index);
//  	Delay_ms(10);

	index=7;
	ublox_cmd[index++]=0x08; //disable GPZDA
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x02;
	ublox_cmd[index++]=0x1F;
	GPS_Send(ublox_cmd,index);
//  	Delay_ms(10);

	index=3;
	ublox_cmd[index++]=0x00; //波特率设置  115200bps 
	ublox_cmd[index++]=0x14;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x01;
	ublox_cmd[index++]=0x00; 
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xD0;
	ublox_cmd[index++]=0x08;
	ublox_cmd[index++]=0x00; 
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xC2;
	ublox_cmd[index++]=0x01;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x07;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x07;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0x00;
	ublox_cmd[index++]=0xC4;
	ublox_cmd[index++]=0x96;
	GPS_Send(ublox_cmd,index);
//  	Delay_ms(10);

}

/**************************实现函数********************************************
*函数原型:		void GPS_Decode(unsigned char len)
*功　　能:		将刚刚接收到的帧数据必要的信息提取出来。
输入参数：
		unsigned char len   接收到的字节数
*******************************************************************************/
void GPS_Decode(unsigned char len){
	unsigned char i , data ,j = 0 ,k = 0;
	
	if((GPS_buffer[0]==0x47)&&(GPS_buffer[1]=='N')&&(GPS_buffer[2]==0x47)&&(GPS_buffer[3]==0x47)&&(GPS_buffer[4]==0x41)){
	// $GNGGA 全球定位数据
	j = 0; 
	for(i = 4; i < len; i++ ){
		data = GPS_buffer[i]; //取数组中的数据
		if(data == ','){
    	j++; //下一个字段
		k = 0;
	}else{ //非逗号
		switch( j ){  
        	case 7:if(k<3){ //使用卫星数量，从00到12（前导位数不足则补0）
			     		GPS_Satel[k++] = data; 
                 		GPS_Satel[k] = 0;
			   			} 
						break;
        	case 9:if(k<8){ //天线离海平面的高度，-9999.9到9999.9米
			     		GPS_Height[k++] = data; 
                 		GPS_Height[k] = 0;
			   			} 
			  			break;
			case 10 :return ; //后面的数据我们不关心，return 退出   
        	default:break;
      		}	//switch 结束
		}	
		}
	}else if((GPS_buffer[0]==0x47)&&(GPS_buffer[1]=='N')&&(GPS_buffer[2]==0x52)&&(GPS_buffer[3]==0x4D)&&(GPS_buffer[4]==0x43)){
		// $GNRMC 运输定位数据
		j = 0;
		for(i = 4; i < len; i++ ){
			data = GPS_buffer[i]; //取数组中的数据
			if(data == ','){
    			j++; //下一个字段
				k = 0;
			}else{ //非逗号
			switch( j ){ 
					case 1: if(k < 6) //UTC 时间，hhmmss（时分秒）格式
                	 		GPS_Time[k++] = data;
               		 		break;
        			case 2: GPS_Status=data; //定位状态，A=有效定位，V=无效定位
                			break;
        			case 3: if(k < 9) //Latitude，纬度ddmm.mmmm（度分）格式
		         			GPS_Latitude[k++] = data;
                			break;
					case 4: Lat = data; //纬度半球N（北半球）或S（南半球）
                			break;		
        			case 5: if(k < 10) // Longitude，经度dddmm.mmmm（度分）格式
		         			GPS_Longitude[k++] = data; 
               			 	break;
        			case 6: Lon = data;//经度半球E（东经）或W（西经）
                			break;            
       				case 7: if(k < 6){ //地面速率（000.0~999.9节)
				 			GPS_Speed[k++] = data; 
                 			GPS_Speed[k] = 0;
			   				} 
                			break; 
        			case 8: if(k < 6){ //地面航向（000.0~359.9度，以真北为参考基准)
				 			GPS_Course[k++] = data; 
                 			GPS_Course[k] = 0;
			   				} 
                			break;  
        			case 9: if(k<6){ //UTC日期，ddmmyy（日月年）格式
			     			GPS_Date[k++] = data; 
                 			GPS_Date[k] = 0;
			   				} 
		       				break;      
        					default:break;
      					}
    			}
			}
	GPS_Data_Ready = 1;//数据准备好了，提示主程序可以进行提取转换了。
	} //$GPRMC 运输定位数据
}

//------GPS中断子程序-------------------------------------
void GPS_Prepare(u8 data)
{

	unsigned char indata = data;
	if(indata == 0x24){	  //帧命令起始位 NMEA-0183 '$'
		GPS_wr_index = 0;
		Frame_End = 0;
	}else if(indata == 0x0D){ //CR
		Frame_End = 0xff;
	}else if(indata == 0x0A){ //LF
		if(Frame_End != 0x00){ //上一个字节是0x0D
			GPS_Decode(GPS_wr_index);  //解出必要的数据
			Frame_End = 0;
			}
	}else{ //非关键字 
		GPS_buffer[GPS_wr_index++] = indata;  //存入缓冲区	
//     GPS_wr_index = 0;		
		Frame_End = 0;
	}
	if(GPS_wr_index == 0xff){
    		GPS_wr_index = 0;
			}
		
}


//
/**************************实现函数********************************************
*函数原型:		float Asc_to_f(volatile unsigned char *str)
*功　　能:		提取字符串中的 有效数字
输入参数：
		unsigned char *str    字符串数组
		返回数组表示的值。  比如字符串 "1230.01"  经过这个程序后，返回浮点的值为1230.01
*******************************************************************************/
float Asc_to_f(volatile unsigned char *str)
{
  signed char temp,flag1,flag2; 
  float value,count;
  flag1 = 1;
  flag2 = 0;
  value = 0;
  count = 1;
  temp = *str;
  while(((*str>='0')&&(*str<='9'))||(*str=='-')||(*str=='.')) //数字或者是符号
  { 
  	temp=*str++;
    if(temp=='-'){ 
	if(flag1)
	   	flag1=-1;
      else
	   return(0x00); //出现两次'-' 结果无效
	}
	else if(temp=='.'){ 
		 flag2=1;	  
	     }
		 else{ 
		   value=value*10+(temp&0x0f);
           if(flag2)
		    	count*=0.1f;
		 }
  }
  value*=count*flag1; //符号位
  return(value);
}

//-----------GPS模块 的线程，需要定时调用-----------
void GPS_Routing(void){
	float temp;
	if(GPS_Data_Ready == 0)//有数据需要处理吗？ 该标志在GPS数据接收中断程序中置位
		return; //没有就退出吧。

	GPS_Data_Ready = 0;	// 清标志 
	//LED_Set_Blink(Red,30,80,1); //LED 闪烁表示正在处理GPS数据
	
	if(GPS_Status == 'A'){ //定位状态，A=有效定位，V=无效定位
		GPS_Locked = 1;
		isGPSOK = 1;
		}else {
		GPS_Locked = 0;
		GPS_Update = 1; 
		return; //都没有定位，下面的数据就不要转换了，浪费时间
		}
	//开始提取有效的位置信息
	GPS_STA_Num = Asc_to_f(GPS_Satel); //使用卫星数量，从00到12

	GPS_Altitude = Asc_to_f(GPS_Height); //天线离海平面的高度，-9999.9到9999.9米
	//纬度 [2446.5241]
	temp = (float)(GPS_Latitude[2]&0x0F)*10.0f+(float)(GPS_Latitude[3]&0x0F)+
		   (float)(GPS_Latitude[5]&0x0F)*0.1f+(float)(GPS_Latitude[6]&0x0F)*0.01f+	
		   (float)(GPS_Latitude[7]&0x0F)*0.001f+(float)(GPS_Latitude[8]&0x0F)*0.0001f;
	temp /= 60.0f;  //转成度
	temp += (float)(GPS_Latitude[0]&0x0F)*10.0f+(float)(GPS_Latitude[1]&0x0F);
	Latitude_GPS = temp;
	if(Lat == 'S'){	//S（南半球）
		Latitude_GPS = -Latitude_GPS;
		}
	//经度 [12100.1536]
	temp = (float)(GPS_Longitude[3]&0x0F)*10.0f+(float)(GPS_Longitude[4]&0x0F)+
		   (float)(GPS_Longitude[6]&0x0F)*0.1f+(float)(GPS_Longitude[7]&0x0F)*0.01f+	
		   (float)(GPS_Longitude[8]&0x0F)*0.001f+(float)(GPS_Longitude[9]&0x0F)*0.0001f;
	temp /= 60.0f;  //转成度
	temp += (float)(GPS_Longitude[0]&0x0F)*100.0f+(float)(GPS_Longitude[1]&0x0F)*10.0f+(float)(GPS_Longitude[2]&0x0F);
	Longitude_GPS = temp;
	if(Lon == 'W'){	 //W（西经）
		Longitude_GPS = -Longitude_GPS;
		}
	//速度
	temp = Asc_to_f(GPS_Speed); //地面速率（000.0~999.9节
	Speed_GPS = temp * 0.51444444f; //1节＝1海里/小时＝1.852公里/小时 = 0.5144444···米每秒
	//航向
	Course_GPS = Asc_to_f(GPS_Course);//0~360,North is 0 or 360
		
	Ve = Speed_GPS * (float)sin((double)Course_GPS/57.3f);
  Vn = Speed_GPS * (float)cos((double)Course_GPS/57.3f);		

	GPS_Update = 1; //GPS位置已更新，提示导航数据可用
}


/*
计算两个点的距离。
lat1 lon1  点1的经纬度  单位度
lat2 lon2  点2的经纬度 
返回计算出来的距离   单位 米
*/
float GPS_Distance(float lat1,float lon1,float lat2,float lon2){
   	float temp;
	float mLat = (lat2 - lat1)*110946.0f;//经度
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f)*111318.0f ;
	temp = 	sqrt(mLat*mLat + mLon*mLon); 	//纬度1度 = 大约111km = 111319.5米   
	return temp;
}

/*
计算两个点的连线的 航向角， 以正北为0 。
lat1 lon1  点1的经纬度  单位度
lat2 lon2  点2的经纬度 
返回 的航向角，单位度。0-360
*/
float GPS_Heading(float lat1,float lon1,float lat2,float lon2){
	float temp;
	float mLat = lat2 - lat1;
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f);
	temp = 90.0f + atan2(-mLat, mLon) * 57.2957795f;

	if(temp < 0)temp += 360.0f;
	return temp;
}


