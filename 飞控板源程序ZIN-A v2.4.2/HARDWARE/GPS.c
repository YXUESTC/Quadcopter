/**************************************************************
 * @brief
   ZIN-A 大四轴主控板
	 飞控爱好群551883670
 * @attention
		请购买者为ZIN小店的代码提供保护。
		您可以移植和其它方式使用，但请不要放到网上。谢谢大家，祝大家学习愉快。
 * @brief	
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "GPS.h"
#include <string.h>
#include "ALL_DATA.h"


#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "USART.h" 
#include "sys.h" 
#include "math.h" 


_GpsData Gpsdata;

void clrStruct(void)//清空
{
	Gpsdata.isGetData = 0;
	Gpsdata.isParseData = 0;
	Gpsdata.isUsefull = 0;
	memset(Gpsdata.GPS_Buffer, 0, GPS_Buffer_Length);      
	memset(Gpsdata.UTCTime, 0, UTCTime_Length);
	memset(Gpsdata.latitude, 0, latitude_Length);
	memset(Gpsdata.N_S, 0, N_S_Length);
	memset(Gpsdata.longitude, 0, longitude_Length);
	memset(Gpsdata.E_W, 0, E_W_Length);
	
}

void gps_get(uint8_t dat)
{
	static uint16_t cnt;
	static uint8_t Rx_buff[GPS_Buffer_Length];
	
	if(Gpsdata.isGetData == 0)
	{
		if(dat == '$')
		{
			cnt = 0;
		}
		
		Rx_buff[cnt++] = dat;
		
		if(Rx_buff[0] == '$' && Rx_buff[1] == 'G' && Rx_buff[2] == 'P' && Rx_buff[3] == 'R' && Rx_buff[4] == 'M' && Rx_buff[5] == 'C')
		{
			if(dat == '\n')
			{
				memset(Gpsdata.GPS_Buffer, 0, GPS_Buffer_Length);      //清空
				memcpy(Gpsdata.GPS_Buffer, Rx_buff, cnt); 	//保存数据
				Gpsdata.isGetData = 1;
				cnt = 0;
				memset(Rx_buff, 0, GPS_Buffer_Length);      //清空	
			}
		}
	}
	
	
	if(cnt >= GPS_Buffer_Length)
	{
		cnt = GPS_Buffer_Length;
	}
}

void Gps_DatAnalysis(void)
{
	char *subString;
	char *subStringNext;
	char i = 0;
	
	if(Gpsdata.isGetData == 1)
	{
		Gpsdata.isGetData = 0;
		
		for(i=0; i<=6; i++)
		{
			if(i == 0)
			{
				if((subString = strstr(Gpsdata.GPS_Buffer, ",")) == NULL)
				{
					break;
				}
			}
			else
			{
				subString++;
				if ((subStringNext = strstr(subString, ",")) != NULL)
				{
					char usefullBuffer[2]; 
					switch(i)
					{
						case 1:memcpy(Gpsdata.UTCTime, subString, subStringNext - subString);break;	//获取UTC时间
						case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	//数据是否有效
						case 3:memcpy(Gpsdata.latitude, subString, subStringNext - subString);break;	//获取纬度信息
						case 4:memcpy(Gpsdata.N_S, subString, subStringNext - subString);break;	//获取N/S
						case 5:memcpy(Gpsdata.longitude, subString, subStringNext - subString);break;	//获取经度信息
						case 6:memcpy(Gpsdata.E_W, subString, subStringNext - subString);break;	//获取E/W

						default:break;
					}

					subString = subStringNext;
					Gpsdata.isParseData = 1; //数据解析完成
					if(usefullBuffer[0] == 'A')
						Gpsdata.isUsefull = 1; //数据有效
					else if(usefullBuffer[0] == 'V')
						Gpsdata.isUsefull = 0; //数据无效
				}
				else
				{
					break;
				}
			}
		}
	}
}

