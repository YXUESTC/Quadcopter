/**************************************************************
 * 
 * @brief
   ZIN-A�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "stm32f4xx.h" 
#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"
#include "imu.h"
#include "GPS.h"
#include "attitude_process.h"
#include "high_process.h"
#include "postion_process.h"
#include "myMath.h"
#include "osc.h"
#include "us100.h"
//------------------------------------------------------------------------------
#undef NULL
#define NULL 0
#undef DISABLE 
#define DISABLE 0
#undef ENABLE 
#define ENABLE 1
#undef REST
#define REST 0
#undef SET 
#define SET 1 
#undef EMERGENT
#define EMERGENT LOCK
//------------------------------------------------------------------------------
#define REMOTE_THR Device.Remote.thr
#define REMOTE_PITCH Device.Remote.pitch
#define REMOTE_ROLL Device.Remote.roll
#define REMOTE_YAW Device.Remote.yaw
#define measured FeedBack
#define desired Expect	


 int Throttle_out; //�ɿ��������ֵ //ң�� + ���� ���ֵ

/**************************************************************
 *  Height control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
static uint8_t set_high_desired = 0; //���߸߶����趨
void HeightPidControl(void)
{
	volatile static uint8_t status=WAITING_1;
	const float dt = 0.005f;
	static float High_Hold_Throttle;
	//----------------------------------------------������ֹ����
	if(Command.FlightMode == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;
	//----------------------------------------------����
	switch(status)
	{
		case WAITING_1:
		  if(Command.FlightMode != LOCK)  //������
			{
				status = WAITING_2;
			}
			break;
		case WAITING_2: //
			if(Command.FlightMode == HEIGHT||Command.FlightMode == GPS_POSITION)//������붨�߷���ģʽ//���߶���ģʽ
			{
				status = WAITING_3;
			}
			
			break;
		case WAITING_3:
			{
					High_Hold_Throttle = LIMIT(REMOTE_THR-1000,0,1000);
					pidRest((PidObject*)&pidData.HeightAccel,3);  //��λ�ϴ����������Ķ���PID����
					pidData.HeightHigh.desired = Height.High;//��¼�߶�	
					status = PROCESS_31;
			}
			break;
		case PROCESS_31://���붨��	
			{
				//-------------------------�߶Ȼ�-------------------------------
					#define Deadzone_Max 1750
					#define Deadzone_Min 1150
		
								
//					if(REMOTE_THR > Deadzone_Max)
//					{
//						pidData.HeightRate.desired = 300*(REMOTE_THR-Deadzone_Max)/(2000-Deadzone_Max);//��������ٶ�30cm/s
//						set_high_desired = 0;
//					}
//					else if(REMOTE_THR<Deadzone_Min)
//					{
//						pidData.HeightRate.desired = 200*(REMOTE_THR-Deadzone_Min)/(Deadzone_Min-1000);//����½��ٶ�20cm/s
//						set_high_desired = 0;
//					}
//					else 
//					{
						static float last_high_out;

					pidData.HeightHigh.desired =0.12*(REMOTE_THR-1000);
						
//						if (set_high_desired ==  0)
//						{
//							pidData.HeightHigh.desired =Height.High; //Height.High;//��¼�߶�	
//							set_high_desired = 1;
//						}		
						static uint8_t high_delay = 0;
						high_delay++;
						if(high_delay>1)
						{
							  high_delay=0;
								//------------�߶Ȼ�����-----------
								pidData.HeightHigh.measured =Height.High; //Height.High; //��ȡ�߶����
			
								last_high_out = pidData.HeightHigh.Control_OutPut; //��¼�ϴεĸ߶����
								PID_Control(&pidData.HeightHigh);    //����PID������������
								#ifndef HighFeedforward//���ٶȻ�ǰ������			
								pidData.HeightAccel.Offset = 0.225f*(pidData.HeightHigh.Control_OutPut - last_high_out)/dt;			
								#endif
								pidData.HeightRate.desired =pidData.HeightHigh.Control_OutPut;	
						}
//					}
					//-------------------------�ٶȻ�-------------------------------
					pidData.HeightRate.measured = Height.Speed;	
					PID_Control_Div_LPF(&pidData.HeightRate);//����PID������������
					//-------------------------���ٶȻ�-------------------------------
//					pidData.HeightAccel.desired = pidData.HeightRate.Control_OutPut;
//					pidData.HeightAccel.measured = 	Attitude.acc_high_feedback ;
//					PID_Control_Err_LPF(&pidData.HeightAccel);//����PID������������
					Throttle_out = 440 + pidData.HeightRate.Control_OutPut;
					 if(REMOTE_THR<1100)
					 {
						 Throttle_out=400;
					 }
					if(Command.FlightMode != HEIGHT && Command.FlightMode != GPS_POSITION) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
							status = EXIT_255;
			}
			break;
		case EXIT_255: //�˳�����
			pidRest((PidObject*)&pidData.HeightAccel,3);  //��λ�ϴ����������Ķ���PID����
			status = WAITING_1;
			break;
		default:
			status = WAITING_1;
			break;	
	}			
}



/**************************************************************
 * ��̬����
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void AttitudePidControl(void)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{		
		case WAITING_1: //�ȴ�����
			if(Command.FlightMode != LOCK)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //׼���������
			
			pidRest(&pidData.Pitch,8); //������λPID���ݣ���ֹ�ϴ�����������Ӱ�챾�ο���

		  
			status = PROCESS_31;
		
			break;			
		case PROCESS_31: //��ʽ�������
			
		
			//--------------------���߿���-----------------------------------------
			Throttle_out = LIMIT(REMOTE_THR-1000,0,1000);
			HeightPidControl(); //���߿���
			//--------------------��̬����-----------------------------------------
			pidData.RateX.measured = Attitude.gyroX_IMU * Gyro_G; //�ڻ�����ֵ �Ƕ�/��
			pidData.RateY.measured = Attitude.gyroY_IMU * Gyro_G;
			pidData.RateZ.measured = Attitude.gyroZ_IMU * Gyro_G;
		
			pidData.Pitch.measured = IMU.pitch; //�⻷����ֵ ��λ���Ƕ�
			pidData.Roll.measured = IMU.roll;
			pidData.Yaw.measured = IMU.yaw;
			if(/*GPS.Quality<3 && GPS.satellite_num >9 && GPS.Home_Ready == 1 && */Command.FlightMode == GPS_POSITION) //GPS�ź�����������,���Ҵ��ڶ���״̬
			{
					pidData.Roll.desired = pidData.GPS_Latitude_rate.Control_OutPut;  //�ɿ�����ϵ�µ�λ���ٶȿ������
					pidData.Pitch.desired = -pidData.GPS_Longitude_rate.Control_OutPut; 
			}
			else
			{ //ң������ֵ����
						#define DEADBAND 40
						const float pitch_roll_remote_ratio = 0.06f;
						if(REMOTE_PITCH>(1500+DEADBAND))
						{
							pidData.Pitch.desired = -(REMOTE_PITCH -(1500+DEADBAND))* pitch_roll_remote_ratio; 
						}
						else if(REMOTE_PITCH<(1500-DEADBAND))
						{
							pidData.Pitch.desired = -(REMOTE_PITCH -(1500-DEADBAND))*pitch_roll_remote_ratio;
						}
						else
							pidData.Pitch.desired = 0;
					
						if(REMOTE_ROLL>(1500+DEADBAND))
						{
							pidData.Roll.desired = -(REMOTE_ROLL -(1500+DEADBAND))*pitch_roll_remote_ratio;
						}
						else if(REMOTE_ROLL<(1500-DEADBAND))
						{
							pidData.Roll.desired = -(REMOTE_ROLL -(1500-DEADBAND))*pitch_roll_remote_ratio;
						}	
						else
							pidData.Roll.desired = 0;
			}

			//--------------------����ǿ���-----------------------------------------
		 	PID_Control(&pidData.Roll);    //����PID�������������⻷	�����PID		
			pidData.RateY.desired =pidData.Roll.Control_OutPut; //���⻷��PID�����Ϊ�ڻ�PID������ֵ��Ϊ����PID
			PID_Control_Div_LPF(&pidData.RateY);  //�ٵ����ڻ�
			//--------------------�����ǿ���-----------------------------------------
		 	PID_Control(&pidData.Pitch);    //����PID�������������⻷	������PID	
			pidData.RateX.desired =pidData.Pitch.Control_OutPut;  
			PID_Control_Div_LPF(&pidData.RateX); //�ٵ����ڻ�
			//--------------------ƫ���ǿ���-----------------------------------------
			static uint8_t lock_yaw_flag = 0;
			if(REMOTE_YAW < 1600 && REMOTE_YAW > 1400)//ƫ����������λ
			 {					
							static float yaw_lock_value;
							if(lock_yaw_flag == 0)
							{
								lock_yaw_flag = 1;
								yaw_lock_value = pidData.Yaw.measured;
								
							}
							pidData.Yaw.Expect = yaw_lock_value;
							PID_Control_Yaw(&pidData.Yaw);//ƫ���Ƕȿ���
							pidData.RateZ.Expect=pidData.Yaw.Control_OutPut;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
							
			 }
			 else//����ƫ������˺�ֻ�����ڻ����ٶȿ���
			 {
				  lock_yaw_flag = 0;
					pidData.Yaw.Expect=0;//ƫ����������0,�����нǶȿ���
					pidData.RateZ.Expect= (1500 - REMOTE_YAW)*0.2f;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
			 }
 
			PID_Control_Div_LPF(&pidData.RateZ);
			pidData.RateZ.Control_OutPut = pidData.RateZ.Control_OutPut;// + 0.35F*pidData.Yaw.Control_OutPut; 
			if(pidData.RateZ.Control_OutPut>=pidData.Yaw.Control_OutPut_Limit)
				pidData.RateZ.Control_OutPut=pidData.Yaw.Control_OutPut_Limit;
		  if(pidData.RateZ.Control_OutPut<=-pidData.Yaw.Control_OutPut_Limit)
				pidData.RateZ.Control_OutPut=-pidData.Yaw.Control_OutPut_Limit;
 
			break;
		case EXIT_255:  //�˳�����
			pidRest(&pidData.Pitch,8); //������λPID���ݣ���ֹ�ϴ�����������Ӱ�챾�ο���
			status = WAITING_1;//���صȴ�����
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(Command.FlightMode == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;
}

/**************************************************************
 * ������
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
#define MOTOR1 TIM3->CCR4 //��Ӧ�ɿذ�PWM_OUT1
#define MOTOR2 TIM3->CCR3 //��Ӧ�ɿذ�PWM_OUT2
#define MOTOR3 TIM3->CCR2 //��Ӧ�ɿذ�PWM_OUT3
#define MOTOR4 TIM3->CCR1 //��Ӧ�ɿذ�PWM_OUT4
#define MOTOR5 TIM5->CCR4 //��Ӧ�ɿذ�PWM_OUT5
#define MOTOR6 TIM5->CCR3 //��Ӧ�ɿذ�PWM_OUT6 
#define MOTOR7 TIM8->CCR3 //��Ӧ�ɿذ�PWM_OUT7
#define MOTOR8 TIM8->CCR4 //��Ӧ�ɿذ�PWM_OUT8

static int16_t PWM_OUT[8] = {0};

#define ACCURACY 10000 //u16(2500/0.25) //accuracy
//PWM 400hz ռ�ձ�40-80%
const uint16_t INIT_DUTY = 4000;
const uint8_t PWM_RADIO = 4;

void MotorControl(void)
{	
	volatile static uint8_t status=WAITING_1;
	
	if(Command.FlightMode == LOCK) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;	
	switch(status)
	{		
		case WAITING_1: //�ȴ�����	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //������������������Ϊ0
			if(Command.FlightMode != LOCK)
			{
				status = WAITING_2;
			}
		case WAITING_2: //������ɺ��ж�ʹ�����Ƿ�ʼ����ң�˽��з��п���
			if(REMOTE_THR>1050) //���������ת����
			{	
				status = PROCESS_31;
			}
			break;
		case PROCESS_31://�������������������̫�಻���п�������
				if(REMOTE_THR<1100) 
				{
					pidRest(&pidData.Pitch,6);//��̬������0
					PWM_OUT[0] = PWM_OUT[1]	= PWM_OUT[2] = PWM_OUT[3]  = REMOTE_THR-1000;//������ֱ��ȡ�����������//��ʱ���Բ����Ƿ�ͬ����ת
				}
				else 
					status = PROCESS_32;//��ʽ����������
			break;
		case PROCESS_32:
			{
				if(Command.FlightMode == NORMOL&&REMOTE_THR<1050) //��̬ģʽ�������½��뵡��״̬
				{
					pidRest(&pidData.Pitch,6);//��̬������0
					PWM_OUT[0] = PWM_OUT[1]	= PWM_OUT[2] = PWM_OUT[3]  = 50;
				}
				else
				{
					//����������������ȡ���ڵ��PWM�ֲ���ɿ�������ϵ���뿴�ɿ�������ϵͼ�⣬���ĸ����PWM�ֲ��ֲ�	
					//           ��ͷ      
					//PWM_OUT2    ��    WM_OUT1
					//      *           *
					//      	*       *
					//    		  *   *
					//      			*  
					//    		  *   *
					//      	*       *
					//      *           *
					// PWM_OUT3        PWM_OUT4
					//		pidData.RateY.Control_OutPut ����Ǵ���PID��� �������ң����Կ���1 4��2 3������������ͬ��ͬ��
					//    pidData.RateX.Control_OutPut �����Ǵ���PID��� ����ǰ�󣬿��Կ���1 2��3 4��ǰ��������ͬ��ͬ��
					//		pidData.RateZ.Control_OutPut ����Ǵ���PID��� ������ת�����Կ���1 3��2 4������Խ��ߵ��ͬ��ͬ��	
					// ������ȡ�����㷨��� ������������Ļ�  ��ǰ��Ȼ��β�������������,���ұ�Ȼ����������������									
					PWM_OUT[0] = PWM_OUT[1]	= PWM_OUT[2] = PWM_OUT[3]  = LIMIT(Throttle_out,0,900);// = PWM_OUT[5]	= PWM_OUT[6] = PWM_OUT[7] = LIMIT(temp,0,900); //��100����̬����
					PWM_OUT[0] +=    + pidData.RateX.Control_OutPut - pidData.RateY.Control_OutPut - pidData.RateZ.Control_OutPut;//; ��̬����������������Ŀ�����//X������	
					PWM_OUT[1] +=    + pidData.RateX.Control_OutPut + pidData.RateY.Control_OutPut + pidData.RateZ.Control_OutPut ;//;
					PWM_OUT[2] +=    - pidData.RateX.Control_OutPut + pidData.RateY.Control_OutPut - pidData.RateZ.Control_OutPut;
					PWM_OUT[3] +=    - pidData.RateX.Control_OutPut - pidData.RateY.Control_OutPut + pidData.RateZ.Control_OutPut;//;
			



//					PWM_OUT[0] +=  - pidData.RateY.Control_OutPut  ;//; ��̬����������������Ŀ�����//X������	
//					PWM_OUT[1] += +  pidData.RateY.Control_OutPut   ;//;
//					PWM_OUT[2] +=  + pidData.RateY.Control_OutPut  ;
//					PWM_OUT[3] +=  - pidData.RateY.Control_OutPut  ;//;

					
					
					PWM_OUT[0] = LIMIT(PWM_OUT[0],0,1000);//�����κ������������ϸ�����Ϊ������ܽ��յķ�Χ
					PWM_OUT[1] = LIMIT(PWM_OUT[1],0,1000);
					PWM_OUT[2] = LIMIT(PWM_OUT[2],0,1000);
					PWM_OUT[3] = LIMIT(PWM_OUT[3],0,1000);
				}
			}	
			break;
		case EXIT_255:
			PWM_OUT[0] = PWM_OUT[1] = PWM_OUT[2] = PWM_OUT[3] = 0;  //������������������Ϊ0
			status = WAITING_1;	
			break;
		default:
			break;
	}
				MOTOR1 = PWM_RADIO *( PWM_OUT[0] ) + INIT_DUTY;				//1	
				MOTOR2 = PWM_RADIO *( PWM_OUT[1] ) + INIT_DUTY;				//2
				MOTOR3 = PWM_RADIO *( PWM_OUT[2] ) + INIT_DUTY;				//3	
				MOTOR4 = PWM_RADIO *( PWM_OUT[3] ) + INIT_DUTY;				//4	
	
} 




/************************************END OF FILE********************************************/ 
