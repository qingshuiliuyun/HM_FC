/* GPS.c file
��д�ߣ�lisn3188
��ַ��www.chiplab7.net
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2013-05-25
���ԣ� ���������ڵ���ʵ���ҵ�mini INS/GPS����ɲ���

ռ��STM32 ��Դ��
1. ʹ��UART3����GPS����
2. ȷ��GPS�Ĳ����� �ڳ�ʼ��ʱ���� Initial_UART3

���ܣ�
���ղ���ȡGPS���ݡ�
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
//���ջ�����
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
		
//---------�ҵ���Ϣ---------------------
void UBLOX_GPS_initial(void);
void UBLOX_GPS_10Hz(void);


void GPS_Send(u8 *data,u8 Len)//���ͺ�����װ��ʹ�ú���ʽ
{
	  Usart3_Send(data,Len);//���ͳ�ȥ
}

void GPS_Init(void)
{
	u8 i = 0;
	USART3_Config(9600);//��ʼ������
	   
	for(i=0;i<10;i++)
	{
		  UBLOX_GPS_initial();  //����ublox GPS  ��������Ѿ���GPS��ͨѶ�ĳ���115200L
  }
	USART3_Config(115200);//��ʼ������
	
	UBLOX_GPS_10Hz(); //��GPS�ķ������ʸĳ�5HZ
}



void UBLOX_GPS_10Hz(void){
	u8 index;  //����ָ��ֻ�� ublox GPSģ�������
	u8 ublox_cmd[30]={0xB5,0x62,0x06,0x01,0x03,0x00,0xf0,0x00};

	index=3;
	ublox_cmd[index++]=0x08; //enable 10Hz �����ٶ� 10HZ
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UBLOX_GPS_initial(void)
*��������:		��ʼ�� UBLOX GPS ģ�飬��ȥ��һЩû��Ҫ�����ݣ���ʡCPU�Ŀ�֧ 
*******************************************************************************/
void UBLOX_GPS_initial(void){
	u8 index;  //����ָ��ֻ�� ublox GPSģ�������
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
	ublox_cmd[index++]=0x00; //����������  115200bps 
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void GPS_Decode(unsigned char len)
*��������:		���ոս��յ���֡���ݱ�Ҫ����Ϣ��ȡ������
���������
		unsigned char len   ���յ����ֽ���
*******************************************************************************/
void GPS_Decode(unsigned char len){
	unsigned char i , data ,j = 0 ,k = 0;
	
	if((GPS_buffer[0]==0x47)&&(GPS_buffer[1]=='N')&&(GPS_buffer[2]==0x47)&&(GPS_buffer[3]==0x47)&&(GPS_buffer[4]==0x41)){
	// $GNGGA ȫ��λ����
	j = 0; 
	for(i = 4; i < len; i++ ){
		data = GPS_buffer[i]; //ȡ�����е�����
		if(data == ','){
    	j++; //��һ���ֶ�
		k = 0;
	}else{ //�Ƕ���
		switch( j ){  
        	case 7:if(k<3){ //ʹ��������������00��12��ǰ��λ��������0��
			     		GPS_Satel[k++] = data; 
                 		GPS_Satel[k] = 0;
			   			} 
						break;
        	case 9:if(k<8){ //�����뺣ƽ��ĸ߶ȣ�-9999.9��9999.9��
			     		GPS_Height[k++] = data; 
                 		GPS_Height[k] = 0;
			   			} 
			  			break;
			case 10 :return ; //������������ǲ����ģ�return �˳�   
        	default:break;
      		}	//switch ����
		}	
		}
	}else if((GPS_buffer[0]==0x47)&&(GPS_buffer[1]=='N')&&(GPS_buffer[2]==0x52)&&(GPS_buffer[3]==0x4D)&&(GPS_buffer[4]==0x43)){
		// $GNRMC ���䶨λ����
		j = 0;
		for(i = 4; i < len; i++ ){
			data = GPS_buffer[i]; //ȡ�����е�����
			if(data == ','){
    			j++; //��һ���ֶ�
				k = 0;
			}else{ //�Ƕ���
			switch( j ){ 
					case 1: if(k < 6) //UTC ʱ�䣬hhmmss��ʱ���룩��ʽ
                	 		GPS_Time[k++] = data;
               		 		break;
        			case 2: GPS_Status=data; //��λ״̬��A=��Ч��λ��V=��Ч��λ
                			break;
        			case 3: if(k < 9) //Latitude��γ��ddmm.mmmm���ȷ֣���ʽ
		         			GPS_Latitude[k++] = data;
                			break;
					case 4: Lat = data; //γ�Ȱ���N�������򣩻�S���ϰ���
                			break;		
        			case 5: if(k < 10) // Longitude������dddmm.mmmm���ȷ֣���ʽ
		         			GPS_Longitude[k++] = data; 
               			 	break;
        			case 6: Lon = data;//���Ȱ���E����������W��������
                			break;            
       				case 7: if(k < 6){ //�������ʣ�000.0~999.9��)
				 			GPS_Speed[k++] = data; 
                 			GPS_Speed[k] = 0;
			   				} 
                			break; 
        			case 8: if(k < 6){ //���溽��000.0~359.9�ȣ����汱Ϊ�ο���׼)
				 			GPS_Course[k++] = data; 
                 			GPS_Course[k] = 0;
			   				} 
                			break;  
        			case 9: if(k<6){ //UTC���ڣ�ddmmyy�������꣩��ʽ
			     			GPS_Date[k++] = data; 
                 			GPS_Date[k] = 0;
			   				} 
		       				break;      
        					default:break;
      					}
    			}
			}
	GPS_Data_Ready = 1;//����׼�����ˣ���ʾ��������Խ�����ȡת���ˡ�
	} //$GPRMC ���䶨λ����
}

//------GPS�ж��ӳ���-------------------------------------
void GPS_Prepare(u8 data)
{

	unsigned char indata = data;
	if(indata == 0x24){	  //֡������ʼλ NMEA-0183 '$'
		GPS_wr_index = 0;
		Frame_End = 0;
	}else if(indata == 0x0D){ //CR
		Frame_End = 0xff;
	}else if(indata == 0x0A){ //LF
		if(Frame_End != 0x00){ //��һ���ֽ���0x0D
			GPS_Decode(GPS_wr_index);  //�����Ҫ������
			Frame_End = 0;
			}
	}else{ //�ǹؼ��� 
		GPS_buffer[GPS_wr_index++] = indata;  //���뻺����	
//     GPS_wr_index = 0;		
		Frame_End = 0;
	}
	if(GPS_wr_index == 0xff){
    		GPS_wr_index = 0;
			}
		
}


//
/**************************ʵ�ֺ���********************************************
*����ԭ��:		float Asc_to_f(volatile unsigned char *str)
*��������:		��ȡ�ַ����е� ��Ч����
���������
		unsigned char *str    �ַ�������
		���������ʾ��ֵ��  �����ַ��� "1230.01"  �����������󣬷��ظ����ֵΪ1230.01
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
  while(((*str>='0')&&(*str<='9'))||(*str=='-')||(*str=='.')) //���ֻ����Ƿ���
  { 
  	temp=*str++;
    if(temp=='-'){ 
	if(flag1)
	   	flag1=-1;
      else
	   return(0x00); //��������'-' �����Ч
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
  value*=count*flag1; //����λ
  return(value);
}

//-----------GPSģ�� ���̣߳���Ҫ��ʱ����-----------
void GPS_Routing(void){
	float temp;
	if(GPS_Data_Ready == 0)//��������Ҫ������ �ñ�־��GPS���ݽ����жϳ�������λ
		return; //û�о��˳��ɡ�

	GPS_Data_Ready = 0;	// ���־ 
	//LED_Set_Blink(Red,30,80,1); //LED ��˸��ʾ���ڴ���GPS����
	
	if(GPS_Status == 'A'){ //��λ״̬��A=��Ч��λ��V=��Ч��λ
		GPS_Locked = 1;
		isGPSOK = 1;
		}else {
		GPS_Locked = 0;
		GPS_Update = 1; 
		return; //��û�ж�λ����������ݾͲ�Ҫת���ˣ��˷�ʱ��
		}
	//��ʼ��ȡ��Ч��λ����Ϣ
	GPS_STA_Num = Asc_to_f(GPS_Satel); //ʹ��������������00��12

	GPS_Altitude = Asc_to_f(GPS_Height); //�����뺣ƽ��ĸ߶ȣ�-9999.9��9999.9��
	//γ�� [2446.5241]
	temp = (float)(GPS_Latitude[2]&0x0F)*10.0f+(float)(GPS_Latitude[3]&0x0F)+
		   (float)(GPS_Latitude[5]&0x0F)*0.1f+(float)(GPS_Latitude[6]&0x0F)*0.01f+	
		   (float)(GPS_Latitude[7]&0x0F)*0.001f+(float)(GPS_Latitude[8]&0x0F)*0.0001f;
	temp /= 60.0f;  //ת�ɶ�
	temp += (float)(GPS_Latitude[0]&0x0F)*10.0f+(float)(GPS_Latitude[1]&0x0F);
	Latitude_GPS = temp;
	if(Lat == 'S'){	//S���ϰ���
		Latitude_GPS = -Latitude_GPS;
		}
	//���� [12100.1536]
	temp = (float)(GPS_Longitude[3]&0x0F)*10.0f+(float)(GPS_Longitude[4]&0x0F)+
		   (float)(GPS_Longitude[6]&0x0F)*0.1f+(float)(GPS_Longitude[7]&0x0F)*0.01f+	
		   (float)(GPS_Longitude[8]&0x0F)*0.001f+(float)(GPS_Longitude[9]&0x0F)*0.0001f;
	temp /= 60.0f;  //ת�ɶ�
	temp += (float)(GPS_Longitude[0]&0x0F)*100.0f+(float)(GPS_Longitude[1]&0x0F)*10.0f+(float)(GPS_Longitude[2]&0x0F);
	Longitude_GPS = temp;
	if(Lon == 'W'){	 //W��������
		Longitude_GPS = -Longitude_GPS;
		}
	//�ٶ�
	temp = Asc_to_f(GPS_Speed); //�������ʣ�000.0~999.9��
	Speed_GPS = temp * 0.51444444f; //1�ڣ�1����/Сʱ��1.852����/Сʱ = 0.5144444��������ÿ��
	//����
	Course_GPS = Asc_to_f(GPS_Course);//0~360,North is 0 or 360
		
	Ve = Speed_GPS * (float)sin((double)Course_GPS/57.3f);
  Vn = Speed_GPS * (float)cos((double)Course_GPS/57.3f);		

	GPS_Update = 1; //GPSλ���Ѹ��£���ʾ�������ݿ���
}


/*
����������ľ��롣
lat1 lon1  ��1�ľ�γ��  ��λ��
lat2 lon2  ��2�ľ�γ�� 
���ؼ�������ľ���   ��λ ��
*/
float GPS_Distance(float lat1,float lon1,float lat2,float lon2){
   	float temp;
	float mLat = (lat2 - lat1)*110946.0f;//����
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f)*111318.0f ;
	temp = 	sqrt(mLat*mLat + mLon*mLon); 	//γ��1�� = ��Լ111km = 111319.5��   
	return temp;
}

/*
��������������ߵ� ����ǣ� ������Ϊ0 ��
lat1 lon1  ��1�ľ�γ��  ��λ��
lat2 lon2  ��2�ľ�γ�� 
���� �ĺ���ǣ���λ�ȡ�0-360
*/
float GPS_Heading(float lat1,float lon1,float lat2,float lon2){
	float temp;
	float mLat = lat2 - lat1;
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f);
	temp = 90.0f + atan2(-mLat, mLon) * 57.2957795f;

	if(temp < 0)temp += 360.0f;
	return temp;
}


