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
/****气压计三阶互补滤波方案——参考开源飞控APM****/
//#define TIME_CONTANST_ZER       1.5f
const float TIME_CONTANST_ZER=1.0f;
#define K_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))//20															// XY????·′à??μêy,3.0
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
			if(delay_cnt<500)//刚刚上电时 对气压计地面补偿
			{
				pos_correction = acc_correction = vel_correction = 0;
				high_offset = high;
				delay_cnt++;
				return;
			}
			const uint8_t High_Delay_Cnt=2;//150ms
			Altitude_Estimate=high-high_offset;//高度观测量
			high_test = high;
      //由观测量（气压计）得到状态误差
			static float History_Z[High_Delay_Cnt+1]; 
			float temp;
      temp= Altitude_Estimate- History_Z[High_Delay_Cnt];//气压计(超声波)与SINS估计量的差，单位cm
      //三路积分反馈量修正惯导
      acc_correction +=temp* K_ACC_ZER*dt ;//加速度矫正量
      vel_correction +=temp* K_VEL_ZER*dt ;//速度矫正量
      pos_correction +=temp* K_POS_ZER*dt ;//位置矫正量
      //加速度计矫正后更新
			static float last_accZ;
			static float Current_accZ; //当前Z轴上的阿加速度
      last_accZ=Current_accZ;//上一次加速度量
      A=Current_accZ = Attitude.acc_yaw_sensor + acc_correction;
      //速度增量矫正后更新，用于更新位置,由于步长h=0.005,相对较长，
      //这里采用二阶龙格库塔法更新微分方程，不建议用更高阶段，因为加速度信号非平滑
			float speed_Z;
			static float t_high;
			static float t_speed;
      speed_Z =+(last_accZ+Current_accZ)*dt/2.0f;
      //原始位置更新
      t_high += (Height.Speed+0.5f*speed_Z)*dt;
      //位置矫正后更新
			Height.High = t_high + pos_correction;
      //垂直原始速度更新
      t_speed +=speed_Z;
      //垂直速度矫正后更新
      Height.Speed = t_speed + vel_correction;

			//---------------------------------------------------------------
			static uint16_t Save_Cnt=0;
      Save_Cnt++;//数据存储周期
      if(Save_Cnt>=4)//20ms
      {
        for(uint8_t Cnt=High_Delay_Cnt;Cnt>0;Cnt--)//20ms滑动一次
        {
					History_Z[Cnt]=History_Z[Cnt-1];
        }
        History_Z[0]=Height.High; 
        Save_Cnt=0;
      }
}





#define Time_contanst_z 5.0f //修改为5ms是否可以？？？  搞错了吧？？？
#define K_ACC_Z (Time_contanst_z/(Time_contanst_z*Time_contanst_z*Time_contanst_z))
#define K_VEL_Z (Time_contanst_z/(Time_contanst_z*Time_contanst_z))
#define K_POS_Z (Time_contanst_z/Time_contanst_z)

float High_Filter[3] = {0.01, 0.2, 0.05}; //参数待定！！！

void Strapdown_INS_High_US100(float high)//单位cm
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
	if(count1<500)//刚刚上电时 对气压计地面补偿
		{
				pos_correction = acc_correction = vel_correction = 0;
				high_offset = high;
				count1++;
				return;
		}
	Altitude_Estimate=high-high_offset;//高度观测量
	Altitude_Dealt = Altitude_Estimate - History_Z[High_Delay_Cnt];   //高度误差计算 超声波高度 - 融合高度
	
	acc_correction += High_Filter[0]*Altitude_Dealt*K_ACC_Z; //加速度校正量
	vel_correction += High_Filter[1]*Altitude_Dealt*K_VEL_Z; //速度校正量
	pos_correction += High_Filter[2]*Altitude_Dealt*K_POS_Z; //位置校正量
	
	Height.Acc_speed = Attitude.acc_yaw_sensor + acc_correction; //传感器加速度 + 加速度校正量
	
	SpeedDealt = ((Height.Acc_speed + Last_AccSpeed) * 0.005f) / 2.0f; //速度改变量
	
	Ori_high += (Height.Speed + 0.5f * SpeedDealt) * 0.005f; //原始位置
	Height.High = Ori_high + pos_correction; //原始位置 + 位置校正量
	Ori_speed += SpeedDealt; //速度改变量累计
	Height.Speed = Ori_speed + vel_correction; //当前速度 + 速度校正量
	
	Last_AccSpeed = Height.Acc_speed;
	
	static uint16_t Save_Cnt=0;
	
	Save_Cnt++;//数据存储周期
	if(Save_Cnt>=4)//20ms
	{
		for(uint8_t Cnt=High_Delay_Cnt;Cnt>0;Cnt--)//20ms滑动一次
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
	kalman_1(&ekf[2],high);  //一维卡尔曼
	Height.High=ekf[2].out;		//解算后的高度值滤波
			OutData[1]=(int)(Height.High);		
	/*****************Z轴速度*******************/
	Z_sppeed=	(Height.High-Height_old)/speed_time;
		
	kalman_1(&ekf[1],Z_sppeed);  //一维卡尔曼	
	Height.Speed=ekf[1].out;
	Height_old=Height.High;
		
		
}




