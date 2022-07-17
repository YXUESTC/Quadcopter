#include "stm32f4xx.h"
#include "us100.h"
#include "USART.h"
#include "ALL_DATA.h"

s8 ultra_start_f;
u8 ultra_ok = 0;

void Ultra_Start()
{
	u8 temp[1]={0};
	
  temp[0]=0x55;
	
	ultra_start_f=1;
	
  UART5_Send(temp,1);
}

u16 ultra_distance,ultra_distance_old;
s16 ultra_delta;
void US100_Get(u8 com_data)
{
	static u8 ultra_tmp;
	
	if( ultra_start_f == 1 )
	{
		ultra_tmp = com_data;
		ultra_start_f = 2;
	}
	else if( ultra_start_f == 2 )
	{
	  ultra_distance = ((ultra_tmp<<8) + com_data)/10.0f;  //cm
		ultra_start_f = 0;
		ultra_ok = 1;
	}
	
	ultra_delta = ultra_distance - ultra_distance_old;
	
	ultra_distance_old = ultra_distance;
	FlightData.High.ultra_height = ultra_distance;
}
