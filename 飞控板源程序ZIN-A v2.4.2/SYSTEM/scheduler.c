/**************************************************************
 * 
 * @brief
   ZIN-7�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
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

//�豸���ݲɼ�
#include "MPU6050.h"
#include "AK8975.h"
#include "US100.h"
#include "spl06.h"
#include "REMOTE.h"
#include "GPS.h"
//����
#include "control.h"
#include "senror.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1
/**************************************************************
 * ����ϵͳ����Ϊ��ѯ���ж��н���Ľ��С�
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void TIM2_IRQHandler(void)   //TIM3�ж� 5ms  //���й̶�ɨ�����ڿ���
{	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
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
//------------------------------------------------//У׼�ж�,ֻ������״̬���ܽ���У׼���������У׼���������̷���	
//		Gyro_Calibration();//���ٶȾ�ֹ��ƫУ׼
//		Accel_six_Calibartion();//���ٶ�����У׼
//		Mag_Calibartion();//������У׼
//------------------------------------------------//���ݲɼ�
//	      Mag8975_calibrate();
//			MpuGetData(); //MPU6050
			if(cnt_15ms == 3) //15ms����һ��
			{
				cnt_15ms = 0;
//				Mag8975_Updata();		//AK8975������
			}		
//			if(cnt_10ms == 2) //10ms����һ��
//			{	
//				cnt_10ms=0;
//				user_spl0601_get();	//��ѹ�Ƹ���
//			}
	/***********US100������**************************/
      if(cnt_50ms>=20)	
			{
				cnt_50ms=0;
				Ultra_Start();
			}				
//------------------------------------------------//�����ں�
		
//			GYRO_IMU_Filter(Device.MPU6050.gyroX,Device.MPU6050.gyroY,Device.MPU6050.gyroZ); //���ٶ��˲�����
//			ACC_IMU_Filter(Device.MPU6050.accX,Device.MPU6050.accY,Device.MPU6050.accZ);	 //���ٶ��˲�����
			//��Ԫ���ݶ��½���
			AHRSUpdate_GraDes_Delay_Corretion(Attitude.gyroX_IMU, Attitude.gyroY_IMU, Attitude.gyroZ_IMU, Attitude.accX_IMU, Attitude.accY_IMU, Attitude.accZ_IMU);
		  IMU_Calculation();
//      Yaw_Lock(Sensor_Data.Mag8975.MAG_x, Sensor_Data.Mag8975.MAG_y, Sensor_Data.Mag8975.MAG_z);			
		  SINS_Prepare(); //�����ں�׼��
			
			spl06_001_HeightHighProcess(); //��ȡ�߶�����
			
//			Strapdown_INS_High(ultra_distance); //�߶��ںϴ���  High.bara_height ultra_distance
//			Strapdown_INS_High_US100(ultra_distance);				
		    High_US100(ultra_distance);
//			GPS_Run();	 //GPS���ݽ���	

//			if(GPS.Home_Ready == 1)//������HOME���Ϳ�ʼ����GPS����
//			{
//				Filter_Horizontal(); //GPSλ����Ϣ�ں�
//			}
//------------------------------------------------//����
			
			RC_Analy();//ң�����ݷ���
			
			AttitudePidControl();//��̬����//���߿���//�������
			MotorControl(); //������
			
			TIM2->SR = (uint16_t)~TIM_IT_Update; //���TIMx���жϴ�����λ:TIM �ж�Դ 		
	}
}


