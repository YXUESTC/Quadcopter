/**************************************************************
 * 
 * @brief
   ZIN-A套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
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


 int Throttle_out; //飞控油门输出值 //遥控 + 定高 输出值

/**************************************************************
 *  Height control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
static uint8_t set_high_desired = 0; //定高高度已设定
void HeightPidControl(void)
{
	volatile static uint8_t status=WAITING_1;
	const float dt = 0.005f;
	static float High_Hold_Throttle;
	//----------------------------------------------紧急终止飞行
	if(Command.FlightMode == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;
	//----------------------------------------------控制
	switch(status)
	{
		case WAITING_1:
		  if(Command.FlightMode != LOCK)  //检测解锁
			{
				status = WAITING_2;
			}
			break;
		case WAITING_2: //
			if(Command.FlightMode == HEIGHT||Command.FlightMode == GPS_POSITION)//如果进入定高飞行模式//或者定点模式
			{
				status = WAITING_3;
			}
			
			break;
		case WAITING_3:
			{
					High_Hold_Throttle = LIMIT(REMOTE_THR-1000,0,1000);
					pidRest((PidObject*)&pidData.HeightAccel,3);  //复位上次遗留下来的定高PID数据
					pidData.HeightHigh.desired = Height.High;//记录高度	
					status = PROCESS_31;
			}
			break;
		case PROCESS_31://进入定高	
			{
				//-------------------------高度环-------------------------------
					#define Deadzone_Max 1750
					#define Deadzone_Min 1150
		
								
//					if(REMOTE_THR > Deadzone_Max)
//					{
//						pidData.HeightRate.desired = 300*(REMOTE_THR-Deadzone_Max)/(2000-Deadzone_Max);//最大上升速度30cm/s
//						set_high_desired = 0;
//					}
//					else if(REMOTE_THR<Deadzone_Min)
//					{
//						pidData.HeightRate.desired = 200*(REMOTE_THR-Deadzone_Min)/(Deadzone_Min-1000);//最大下降速度20cm/s
//						set_high_desired = 0;
//					}
//					else 
//					{
						static float last_high_out;

					pidData.HeightHigh.desired =0.12*(REMOTE_THR-1000);
						
//						if (set_high_desired ==  0)
//						{
//							pidData.HeightHigh.desired =Height.High; //Height.High;//记录高度	
//							set_high_desired = 1;
//						}		
						static uint8_t high_delay = 0;
						high_delay++;
						if(high_delay>1)
						{
							  high_delay=0;
								//------------高度环控制-----------
								pidData.HeightHigh.measured =Height.High; //Height.High; //获取高度情况
			
								last_high_out = pidData.HeightHigh.Control_OutPut; //记录上次的高度输出
								PID_Control(&pidData.HeightHigh);    //调用PID处理函数来处理
								#ifndef HighFeedforward//加速度环前馈控制			
								pidData.HeightAccel.Offset = 0.225f*(pidData.HeightHigh.Control_OutPut - last_high_out)/dt;			
								#endif
								pidData.HeightRate.desired =pidData.HeightHigh.Control_OutPut;	
						}
//					}
					//-------------------------速度环-------------------------------
					pidData.HeightRate.measured = Height.Speed;	
					PID_Control_Div_LPF(&pidData.HeightRate);//调用PID处理函数来处理
					//-------------------------加速度环-------------------------------
//					pidData.HeightAccel.desired = pidData.HeightRate.Control_OutPut;
//					pidData.HeightAccel.measured = 	Attitude.acc_high_feedback ;
//					PID_Control_Err_LPF(&pidData.HeightAccel);//调用PID处理函数来处理
					Throttle_out = 440 + pidData.HeightRate.Control_OutPut;
					 if(REMOTE_THR<1100)
					 {
						 Throttle_out=400;
					 }
					if(Command.FlightMode != HEIGHT && Command.FlightMode != GPS_POSITION) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
							status = EXIT_255;
			}
			break;
		case EXIT_255: //退出定高
			pidRest((PidObject*)&pidData.HeightAccel,3);  //复位上次遗留下来的定高PID数据
			status = WAITING_1;
			break;
		default:
			status = WAITING_1;
			break;	
	}			
}



/**************************************************************
 * 姿态控制
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void AttitudePidControl(void)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{		
		case WAITING_1: //等待解锁
			if(Command.FlightMode != LOCK)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //准备进入控制
			
			pidRest(&pidData.Pitch,8); //批量复位PID数据，防止上次遗留的数据影响本次控制

		  
			status = PROCESS_31;
		
			break;			
		case PROCESS_31: //正式进入控制
			
		
			//--------------------定高控制-----------------------------------------
			Throttle_out = LIMIT(REMOTE_THR-1000,0,1000);
			HeightPidControl(); //定高控制
			//--------------------姿态控制-----------------------------------------
			pidData.RateX.measured = Attitude.gyroX_IMU * Gyro_G; //内环测量值 角度/秒
			pidData.RateY.measured = Attitude.gyroY_IMU * Gyro_G;
			pidData.RateZ.measured = Attitude.gyroZ_IMU * Gyro_G;
		
			pidData.Pitch.measured = IMU.pitch; //外环测量值 单位：角度
			pidData.Roll.measured = IMU.roll;
			pidData.Yaw.measured = IMU.yaw;
			if(/*GPS.Quality<3 && GPS.satellite_num >9 && GPS.Home_Ready == 1 && */Command.FlightMode == GPS_POSITION) //GPS信号能用作定点,并且处于定点状态
			{
					pidData.Roll.desired = pidData.GPS_Latitude_rate.Control_OutPut;  //飞控坐标系下的位置速度控制输出
					pidData.Pitch.desired = -pidData.GPS_Longitude_rate.Control_OutPut; 
			}
			else
			{ //遥控期望值更新
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

			//--------------------横滚角控制-----------------------------------------
		 	PID_Control(&pidData.Roll);    //调用PID处理函数来处理外环	横滚角PID		
			pidData.RateY.desired =pidData.Roll.Control_OutPut; //将外环的PID输出作为内环PID的期望值即为串级PID
			PID_Control_Div_LPF(&pidData.RateY);  //再调用内环
			//--------------------俯仰角控制-----------------------------------------
		 	PID_Control(&pidData.Pitch);    //调用PID处理函数来处理外环	俯仰角PID	
			pidData.RateX.desired =pidData.Pitch.Control_OutPut;  
			PID_Control_Div_LPF(&pidData.RateX); //再调用内环
			//--------------------偏航角控制-----------------------------------------
			static uint8_t lock_yaw_flag = 0;
			if(REMOTE_YAW < 1600 && REMOTE_YAW > 1400)//偏航杆置于中位
			 {					
							static float yaw_lock_value;
							if(lock_yaw_flag == 0)
							{
								lock_yaw_flag = 1;
								yaw_lock_value = pidData.Yaw.measured;
								
							}
							pidData.Yaw.Expect = yaw_lock_value;
							PID_Control_Yaw(&pidData.Yaw);//偏航角度控制
							pidData.RateZ.Expect=pidData.Yaw.Control_OutPut;//偏航角速度环期望，来源于偏航角度控制器输出
							
			 }
			 else//波动偏航方向杆后，只进行内环角速度控制
			 {
				  lock_yaw_flag = 0;
					pidData.Yaw.Expect=0;//偏航角期望给0,不进行角度控制
					pidData.RateZ.Expect= (1500 - REMOTE_YAW)*0.2f;//偏航角速度环期望，直接来源于遥控器打杆量
			 }
 
			PID_Control_Div_LPF(&pidData.RateZ);
			pidData.RateZ.Control_OutPut = pidData.RateZ.Control_OutPut;// + 0.35F*pidData.Yaw.Control_OutPut; 
			if(pidData.RateZ.Control_OutPut>=pidData.Yaw.Control_OutPut_Limit)
				pidData.RateZ.Control_OutPut=pidData.Yaw.Control_OutPut_Limit;
		  if(pidData.RateZ.Control_OutPut<=-pidData.Yaw.Control_OutPut_Limit)
				pidData.RateZ.Control_OutPut=-pidData.Yaw.Control_OutPut_Limit;
 
			break;
		case EXIT_255:  //退出控制
			pidRest(&pidData.Pitch,8); //批量复位PID数据，防止上次遗留的数据影响本次控制
			status = WAITING_1;//返回等待解锁
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(Command.FlightMode == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;
}

/**************************************************************
 * 电机输出
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
#define MOTOR1 TIM3->CCR4 //对应飞控板PWM_OUT1
#define MOTOR2 TIM3->CCR3 //对应飞控板PWM_OUT2
#define MOTOR3 TIM3->CCR2 //对应飞控板PWM_OUT3
#define MOTOR4 TIM3->CCR1 //对应飞控板PWM_OUT4
#define MOTOR5 TIM5->CCR4 //对应飞控板PWM_OUT5
#define MOTOR6 TIM5->CCR3 //对应飞控板PWM_OUT6 
#define MOTOR7 TIM8->CCR3 //对应飞控板PWM_OUT7
#define MOTOR8 TIM8->CCR4 //对应飞控板PWM_OUT8

static int16_t PWM_OUT[8] = {0};

#define ACCURACY 10000 //u16(2500/0.25) //accuracy
//PWM 400hz 占空比40-80%
const uint16_t INIT_DUTY = 4000;
const uint8_t PWM_RADIO = 4;

void MotorControl(void)
{	
	volatile static uint8_t status=WAITING_1;
	
	if(Command.FlightMode == LOCK) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;	
	switch(status)
	{		
		case WAITING_1: //等待解锁	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //如果锁定，则电机输出都为0
			if(Command.FlightMode != LOCK)
			{
				status = WAITING_2;
			}
		case WAITING_2: //解锁完成后判断使用者是否开始拨动遥杆进行飞行控制
			if(REMOTE_THR>1050) //如果大于起转油门
			{	
				status = PROCESS_31;
			}
			break;
		case PROCESS_31://初解锁，低于起飞油门太多不进行控制运算
				if(REMOTE_THR<1100) 
				{
					pidRest(&pidData.Pitch,6);//姿态数据清0
					PWM_OUT[0] = PWM_OUT[1]	= PWM_OUT[2] = PWM_OUT[3]  = REMOTE_THR-1000;//电机输出直接取决于油门输出//此时可以测试是否同步起转
				}
				else 
					status = PROCESS_32;//正式进入控制输出
			break;
		case PROCESS_32:
			{
				if(Command.FlightMode == NORMOL&&REMOTE_THR<1050) //姿态模式低油门下进入怠速状态
				{
					pidRest(&pidData.Pitch,6);//姿态数据清0
					PWM_OUT[0] = PWM_OUT[1]	= PWM_OUT[2] = PWM_OUT[3]  = 50;
				}
				else
				{
					//以下输出的脉冲分配取决于电机PWM分布与飞控坐标体系。请看飞控坐标体系图解，与四个电机PWM分布分布	
					//           机头      
					//PWM_OUT2    ♂    WM_OUT1
					//      *           *
					//      	*       *
					//    		  *   *
					//      			*  
					//    		  *   *
					//      	*       *
					//      *           *
					// PWM_OUT3        PWM_OUT4
					//		pidData.RateY.Control_OutPut 横滚角串级PID输出 控制左右，可以看出1 4和2 3，左右两组电机同增同减
					//    pidData.RateX.Control_OutPut 俯仰角串级PID输出 控制前后，可以看出1 2和3 4，前后两组电机同增同减
					//		pidData.RateZ.Control_OutPut 横滚角串级PID输出 控制旋转，可以看出1 3和2 4，两组对角线电机同增同减	
					// 正负号取决于算法输出 比如输出是正的话  往前必然是尾巴两个电机增加,往右必然是左边两个电机增加									
					PWM_OUT[0] = PWM_OUT[1]	= PWM_OUT[2] = PWM_OUT[3]  = LIMIT(Throttle_out,0,900);// = PWM_OUT[5]	= PWM_OUT[6] = PWM_OUT[7] = LIMIT(temp,0,900); //留100给姿态控制
					PWM_OUT[0] +=    + pidData.RateX.Control_OutPut - pidData.RateY.Control_OutPut - pidData.RateZ.Control_OutPut;//; 姿态输出分配给各个电机的控制量//X型四轴	
					PWM_OUT[1] +=    + pidData.RateX.Control_OutPut + pidData.RateY.Control_OutPut + pidData.RateZ.Control_OutPut ;//;
					PWM_OUT[2] +=    - pidData.RateX.Control_OutPut + pidData.RateY.Control_OutPut - pidData.RateZ.Control_OutPut;
					PWM_OUT[3] +=    - pidData.RateX.Control_OutPut - pidData.RateY.Control_OutPut + pidData.RateZ.Control_OutPut;//;
			



//					PWM_OUT[0] +=  - pidData.RateY.Control_OutPut  ;//; 姿态输出分配给各个电机的控制量//X型四轴	
//					PWM_OUT[1] += +  pidData.RateY.Control_OutPut   ;//;
//					PWM_OUT[2] +=  + pidData.RateY.Control_OutPut  ;
//					PWM_OUT[3] +=  - pidData.RateY.Control_OutPut  ;//;

					
					
					PWM_OUT[0] = LIMIT(PWM_OUT[0],0,1000);//不管任何情况，输出都严格限制为电调所能接收的范围
					PWM_OUT[1] = LIMIT(PWM_OUT[1],0,1000);
					PWM_OUT[2] = LIMIT(PWM_OUT[2],0,1000);
					PWM_OUT[3] = LIMIT(PWM_OUT[3],0,1000);
				}
			}	
			break;
		case EXIT_255:
			PWM_OUT[0] = PWM_OUT[1] = PWM_OUT[2] = PWM_OUT[3] = 0;  //如果锁定，则电机输出都为0
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
