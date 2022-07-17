#ifndef _PID_H_
#define _PID_H_

#include "ALL_DEFINE.h"
#include "Filter.h"

typedef struct
{
    float Expect;//期望
	  float Offset;
    float FeedBack;//反馈值
    float Err;//偏差
    float Last_Err;//上次偏差
    float Err_Max;//偏差限幅值
    float Integrate_Separation_Err;//积分分离偏差值
    float Integrate;//积分值
    float Integrate_Max;//积分限幅值
    float Kp;//控制参数Kp
    float Ki;//控制参数Ki
    float Kd;//控制参数Kd
    float Control_OutPut;//控制器总输出
    float Last_Control_OutPut;//上次控制器总输出
    float Control_OutPut_Limit;//输出限幅
    /***************************************/
    float Last_FeedBack;//上次反馈值
    float Dis_Err;//微分量
    float Dis_Error_History[5];//历史微分量
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    uint8 Err_Limit_Flag :1;//偏差限幅标志
    uint8 Integrate_Limit_Flag :1;//积分限幅标志
    uint8 Integrate_Separation_Flag :1;//积分分离标志		
    Butter_BufferData Control_Device_LPF_Buffer;//控制器低通输入输出缓冲
}PidObject;

typedef struct
{
     PidObject Pitch;
     PidObject RateX;
     PidObject Roll;
     PidObject RateY;
     PidObject Yaw;
     PidObject RateZ;
	
     PidObject HeightAccel;
     PidObject HeightRate;
     PidObject HeightHigh;
	
		 PidObject GPS_Longitude_Acce;		
     PidObject GPS_Latitude_Acce;		
     PidObject GPS_Latitude_rate;
     PidObject GPS_Longitude_rate;
		 PidObject GPS_Latitude_position;		
     PidObject GPS_Longitude_position;	
}_st_pidData;



extern _st_pidData pidData;



void  Total_PID_Init(void);

float PID_Control_High(PidObject *Controler);
float PID_Control(PidObject *Controler);
float PID_Control_Yaw(PidObject *Controler);

float PID_Control_Div_LPF(PidObject *Controler);
float PID_Control_Err_LPF(PidObject *Controler);
void  PID_Integrate_Reset(PidObject *Controler);
void  Take_Off_Reset(void);

void pidRest(PidObject *Controler,uint8_t len);
#endif


