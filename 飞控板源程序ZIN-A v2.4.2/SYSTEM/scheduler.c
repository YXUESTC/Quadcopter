/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "STM32F4XX.h"
#include "ALL_DEFINE.h"
#include "ALL_DATA.h"
#include "led.h"
#include "motor.h"
#include "attitude_process.h"
#include "high_process.h"
#include "postion_process.h"
#include "imu.h"

//设备数据采集
#include "MPU6050.h"
#include "AK8975.h"
#include "US100.h"
#include "spl06.h"
#include "REMOTE.h"
#include "GPS.h"
//控制
#include "control.h"
#include "senror.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1
/**************************************************************
 * 整个系统，分为轮询和中断有节奏的进行。
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void TIM2_IRQHandler(void)   //TIM3中断 5ms  //飞行固定扫描周期控制
{	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
			static uint8_t cnt_15ms = 0; 		
			static uint8_t cnt_10ms = 0;	 		
			static uint8_t cnt_50ms = 0;
			cnt_15ms++;
			cnt_10ms++;
			cnt_50ms++;	
		
		 Mpu6050_Updata();
		if(cnt_10ms == 4)
		{
			cnt_10ms = 0;
			spl06_001_get_data();
		}
//------------------------------------------------//校准判断,只有锁定状态才能进行校准，否则进入校准函数会立刻返回	
//		Gyro_Calibration();//角速度静止零偏校准
//		Accel_six_Calibartion();//加速度六面校准
//		Mag_Calibartion();//磁力计校准
//------------------------------------------------//数据采集
//	      Mag8975_calibrate();
//			MpuGetData(); //MPU6050
			if(cnt_15ms == 3) //15ms更新一次
			{
				cnt_15ms = 0;
//				Mag8975_Updata();		//AK8975磁力计
			}		
//			if(cnt_10ms == 2) //10ms更新一次
//			{	
//				cnt_10ms=0;
//				user_spl0601_get();	//气压计更新
//			}
	/***********US100超声波**************************/
      if(cnt_50ms>=20)	
			{
				cnt_50ms=0;
				Ultra_Start();
			}				
//------------------------------------------------//数据融合
		
//			GYRO_IMU_Filter(Device.MPU6050.gyroX,Device.MPU6050.gyroY,Device.MPU6050.gyroZ); //角速度滤波处理
//			ACC_IMU_Filter(Device.MPU6050.accX,Device.MPU6050.accY,Device.MPU6050.accZ);	 //加速度滤波处理
			//四元数梯度下降法
			AHRSUpdate_GraDes_Delay_Corretion(Attitude.gyroX_IMU, Attitude.gyroY_IMU, Attitude.gyroZ_IMU, Attitude.accX_IMU, Attitude.accY_IMU, Attitude.accZ_IMU);
		  IMU_Calculation();
//      Yaw_Lock(Sensor_Data.Mag8975.MAG_x, Sensor_Data.Mag8975.MAG_y, Sensor_Data.Mag8975.MAG_z);			
		  SINS_Prepare(); //数据融合准备
			
			spl06_001_HeightHighProcess(); //获取高度数据
			
//			Strapdown_INS_High(ultra_distance); //高度融合处理  High.bara_height ultra_distance
//			Strapdown_INS_High_US100(ultra_distance);				
		    High_US100(ultra_distance);
//			GPS_Run();	 //GPS数据解析	

//			if(GPS.Home_Ready == 1)//已设置HOME点后就开始解算GPS数据
//			{
//				Filter_Horizontal(); //GPS位置信息融合
//			}
//------------------------------------------------//控制
			
			RC_Analy();//遥控数据分析
			
			AttitudePidControl();//姿态控制//定高控制//定点控制
			MotorControl(); //电机输出
			
			TIM2->SR = (uint16_t)~TIM_IT_Update; //清除TIMx的中断待处理位:TIM 中断源 		
	}
}


