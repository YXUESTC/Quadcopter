#include "mpu6050.h"
#include "attitude_process.h"
#include "high_process.h"
#include "sys.h"
#include "us100.h"
#include "osc.h"
#include "kalman.h"
#include "filter.h"
_st_Height Height;

 float pos_correction;
 float acc_correction;
 float vel_correction;
/****��ѹ�����׻����˲����������ο���Դ�ɿ�APM****/
//#define TIME_CONTANST_ZER       1.5f
const float TIME_CONTANST_ZER=1.0f;
#define K_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))//20															// XY????���䨤??�̨�y,3.0
#define K_POS_ZER               (3.0f / TIME_CONTANST_ZER)
float high_test;
float Altitude_Estimate=0;
float A=0,B=0;
void Strapdown_INS_High(float high)
{
			float dt;
			
			static uint16_t delay_cnt;
		  static float high_offset;
	    {
			  static	float last_time;
				float now_time;
				now_time = micros();
				B=dt = now_time - last_time;
				dt /=1000;
				last_time = now_time;
			
			}
			if(delay_cnt<500)//�ո��ϵ�ʱ ����ѹ�Ƶ��油��
			{
				pos_correction = acc_correction = vel_correction = 0;
				high_offset = high;
				delay_cnt++;
				return;
			}
			const uint8_t High_Delay_Cnt=2;//150ms
			Altitude_Estimate=high-high_offset;//�߶ȹ۲���
			high_test = high;
      //�ɹ۲�������ѹ�ƣ��õ�״̬���
			static float History_Z[High_Delay_Cnt+1]; 
			float temp;
      temp= Altitude_Estimate- History_Z[High_Delay_Cnt];//��ѹ��(������)��SINS�������Ĳ��λcm
      //��·���ַ����������ߵ�
      acc_correction +=temp* K_ACC_ZER*dt ;//���ٶȽ�����
      vel_correction +=temp* K_VEL_ZER*dt ;//�ٶȽ�����
      pos_correction +=temp* K_POS_ZER*dt ;//λ�ý�����
      //���ٶȼƽ��������
			static float last_accZ;
			static float Current_accZ; //��ǰZ���ϵİ����ٶ�
      last_accZ=Current_accZ;//��һ�μ��ٶ���
      A=Current_accZ = Attitude.acc_yaw_sensor + acc_correction;
      //�ٶ�������������£����ڸ���λ��,���ڲ���h=0.005,��Խϳ���
      //������ö����������������΢�ַ��̣��������ø��߽׶Σ���Ϊ���ٶ��źŷ�ƽ��
			float speed_Z;
			static float t_high;
			static float t_speed;
      speed_Z =+(last_accZ+Current_accZ)*dt/2.0f;
      //ԭʼλ�ø���
      t_high += (Height.Speed+0.5f*speed_Z)*dt;
      //λ�ý��������
			Height.High = t_high + pos_correction;
      //��ֱԭʼ�ٶȸ���
      t_speed +=speed_Z;
      //��ֱ�ٶȽ��������
      Height.Speed = t_speed + vel_correction;

			//---------------------------------------------------------------
			static uint16_t Save_Cnt=0;
      Save_Cnt++;//���ݴ洢����
      if(Save_Cnt>=4)//20ms
      {
        for(uint8_t Cnt=High_Delay_Cnt;Cnt>0;Cnt--)//20ms����һ��
        {
					History_Z[Cnt]=History_Z[Cnt-1];
        }
        History_Z[0]=Height.High; 
        Save_Cnt=0;
      }
}





#define Time_contanst_z 5.0f //�޸�Ϊ5ms�Ƿ���ԣ�����  ����˰ɣ�����
#define K_ACC_Z (Time_contanst_z/(Time_contanst_z*Time_contanst_z*Time_contanst_z))
#define K_VEL_Z (Time_contanst_z/(Time_contanst_z*Time_contanst_z))
#define K_POS_Z (Time_contanst_z/Time_contanst_z)

float High_Filter[3] = {0.01, 0.2, 0.05}; //��������������

void Strapdown_INS_High_US100(float high)//��λcm
{
	static float Altitude_Dealt=0;
	const uint8_t High_Delay_Cnt=2;
	static float History_Z[High_Delay_Cnt+1]; 
	static float SpeedDealt=0;
	static float Ori_high;
	static float Ori_speed;
	static float Last_AccSpeed;
	static float count1=0;
	static float high_offset;
	if(count1<500)//�ո��ϵ�ʱ ����ѹ�Ƶ��油��
		{
				pos_correction = acc_correction = vel_correction = 0;
				high_offset = high;
				count1++;
				return;
		}
	Altitude_Estimate=high-high_offset;//�߶ȹ۲���
	Altitude_Dealt = Altitude_Estimate - History_Z[High_Delay_Cnt];   //�߶������� �������߶� - �ںϸ߶�
	
	acc_correction += High_Filter[0]*Altitude_Dealt*K_ACC_Z; //���ٶ�У����
	vel_correction += High_Filter[1]*Altitude_Dealt*K_VEL_Z; //�ٶ�У����
	pos_correction += High_Filter[2]*Altitude_Dealt*K_POS_Z; //λ��У����
	
	Height.Acc_speed = Attitude.acc_yaw_sensor + acc_correction; //���������ٶ� + ���ٶ�У����
	
	SpeedDealt = ((Height.Acc_speed + Last_AccSpeed) * 0.005f) / 2.0f; //�ٶȸı���
	
	Ori_high += (Height.Speed + 0.5f * SpeedDealt) * 0.005f; //ԭʼλ��
	Height.High = Ori_high + pos_correction; //ԭʼλ�� + λ��У����
	Ori_speed += SpeedDealt; //�ٶȸı����ۼ�
	Height.Speed = Ori_speed + vel_correction; //��ǰ�ٶ� + �ٶ�У����
	
	Last_AccSpeed = Height.Acc_speed;
	
	static uint16_t Save_Cnt=0;
	
	Save_Cnt++;//���ݴ洢����
	if(Save_Cnt>=4)//20ms
	{
		for(uint8_t Cnt=High_Delay_Cnt;Cnt>0;Cnt--)//20ms����һ��
		{
			History_Z[Cnt]=History_Z[Cnt-1];
		}
		History_Z[0]=Height.High;
		Save_Cnt=0;
	}
}
float Height_old=0,speed_time; 
void High_US100(float high)
{
	static float Z_sppeed=0;
	static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};
	  {
			static	float last_time;
			float now_time;
			now_time = micros();
			speed_time = now_time - last_time;
			speed_time /=1000;
			last_time = now_time;
		} 
	OutData[0]=(int)(high); 
	kalman_1(&ekf[2],high);  //һά������
	Height.High=ekf[2].out;		//�����ĸ߶�ֵ�˲�
			OutData[1]=(int)(Height.High);		
	/*****************Z���ٶ�*******************/
	Z_sppeed=	(Height.High-Height_old)/speed_time;
		
	kalman_1(&ekf[1],Z_sppeed);  //һά������	
	Height.Speed=ekf[1].out;
	Height_old=Height.High;
		
		
}




