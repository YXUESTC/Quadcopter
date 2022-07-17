#ifndef _PID_H_
#define _PID_H_

#include "ALL_DEFINE.h"
#include "Filter.h"

typedef struct
{
    float Expect;//����
	  float Offset;
    float FeedBack;//����ֵ
    float Err;//ƫ��
    float Last_Err;//�ϴ�ƫ��
    float Err_Max;//ƫ���޷�ֵ
    float Integrate_Separation_Err;//���ַ���ƫ��ֵ
    float Integrate;//����ֵ
    float Integrate_Max;//�����޷�ֵ
    float Kp;//���Ʋ���Kp
    float Ki;//���Ʋ���Ki
    float Kd;//���Ʋ���Kd
    float Control_OutPut;//�����������
    float Last_Control_OutPut;//�ϴο����������
    float Control_OutPut_Limit;//����޷�
    /***************************************/
    float Last_FeedBack;//�ϴη���ֵ
    float Dis_Err;//΢����
    float Dis_Error_History[5];//��ʷ΢����
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    uint8 Err_Limit_Flag :1;//ƫ���޷���־
    uint8 Integrate_Limit_Flag :1;//�����޷���־
    uint8 Integrate_Separation_Flag :1;//���ַ����־		
    Butter_BufferData Control_Device_LPF_Buffer;//��������ͨ�����������
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


