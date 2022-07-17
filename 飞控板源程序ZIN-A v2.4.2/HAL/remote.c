/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
 #include "TIM.h"
#include "ALL_DATA.h"
//#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "GPS.h"
#include "postion_process.h"


#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

/*****************************************************************************************
 *  通道数据处理
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	

static uint16_t RC_LOST;	
	
void remote_unlock(void);	

void RC_Analy(void)  
{
					//-----------------------模式切换-----------------------------------------------
					if(Command.FlightMode != LOCK) //如果不是锁定状态则进行模式判断
					{
						if(Device.Remote.AUX2 >1700) //如果处于定点状态
						{
								//if(GPS.satellite_num>9 &&GPS.Quality<3.0f && Earth_Position.Filter_Defeated_Flag)//定位卫星小于10个或者水平精度因子小于3不可用  //或者融合失败
								//{
									Command.FlightMode = GPS_POSITION;
							//	}
						}
						else if(Device.Remote.AUX1>1700) //不处于定点状态则判断是否处于定高状态
						{				
							Command.FlightMode = HEIGHT;			
						}
						else
							Command.FlightMode = NORMOL;
						
					}
						
					//---------模式切换改变灯的闪烁状态--------------
							switch(Command.FlightMode)
							{
									case LOCK:
										if(Command.AccOffset==0 && Command.GyroOffset == 0&&Command.MagOffset == 0 && Command.six_acc_offset == 0) //不处于校准状态
										{
//											LED.color = CYAN;
										}	
										break;
									case NORMOL://正常飞行操作
										LED.color = BLUE;  
										break;									
									case HEIGHT://定高颜色
										LED.color = GREE;	 			
										break;
									case GPS_POSITION://定点颜色
										LED.color = WHITE;
										break;
									default:
										break;								
							}
					//---------------------------------------------------------------------------------
						remote_unlock();  //解锁判断							
					//---------------------------------------------------------------------------------
						if(RC_LOST++ > 300)//如果1.5S没RC看门狗没被清0 则判定为遥控信号丢失，此时应当紧急处理。
							//这里进入自动降落模式，飞行器会慢慢降落，请在无人地区进行飞行。切勿过远（理论1000米，实际最好不要大于300米）飞行。
						  //请在飞行前检查接线，排除接线松动而导致中途失控。
						{
								Device.Remote.yaw = Device.Remote.roll = Device.Remote.pitch = 1500;
								Device.Remote.thr = 1000;
								if( Command.FlightMode != LOCK)  //失控进入降落模式
								{
									 Device.Remote.thr = 1200;
									 Command.FlightMode = HEIGHT; //进入定高降落模式
								}
						}
}



void Feed_Rc_Dog(void) //1S内必须调用一次
{
	RC_LOST = 0;
}

/*****************************************************************************************
 *  解锁和锁定判断
 * @brief解锁单步进行，
				 通用简单解锁方式，油门遥杆右下角掰到最顶，回到正中最低后完成解锁动作。
				 解锁极限条件：THR小于1150，YAW大于1800。如果无法达到此条件请用上位机查看你的遥控数据是否满足。
				 通用简单锁定方式，油门遥杆左下角掰至最顶
				 锁定极限条件：THR小于1150，YAW小于1200。
 * @warining 特别提醒①：解锁动作很简单
						 特别提醒②：遇到紧急情况请用遥控立刻锁定飞行器
 * @autor 
   ZIN-A套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ******************************************************************************************/	
void remote_unlock(void)
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

	if(Device.Remote.thr<1150 &&Device.Remote.yaw<1200)                         //油门遥杆左下角锁定
	{
		status = EXIT_255;
	}
	switch(status)
	{
		
			//使用右下解锁
			case WAITING_1://等待解锁动作
				if(Device.Remote.thr<1050 &&Device.Remote.yaw>1800)                         //油门遥杆左下角锁定
				{
					status = WAITING_2;
				}
				break;
		case WAITING_2://发生解锁动作后等待遥杆回复初始状态
				if(Device.Remote.thr<1050 &&Device.Remote.yaw<1600)                         //油门遥杆左下角锁定
				{
					status = WAITING_4;
				}
				break;				
		case WAITING_4:	//解锁前准备	               
				Command.FlightMode = NORMOL;   //解锁标志位
				status = PROCESS_31;   //进入控制
										
				 break;		
		case PROCESS_31:	//进入解锁状态 

				if(Device.Remote.thr<1020)
				{
					if(cnt++ > 3000 && Command.FlightMode == NORMOL)   // 姿态状态低油门15S自动上锁
					{								
						status = EXIT_255;								
					}
				}				
				if(Command.FlightMode == LOCK)                           //其它紧急情况可直接锁定飞控
				{
					status = EXIT_255;				
				}
				else					
					cnt = 0;
			break;
		case EXIT_255: //进入锁定
			LED.color = CYAN;     //锁定后变成青色闪烁		
			cnt = 0; 		 		
			Command.FlightMode = LOCK;
			status = WAITING_1;
			break;
		default:
			status = EXIT_255;
			break;
	}
}
/*****************************************************************************************
 * 函数名：电调校准
 * 使用方法：打开此函数，上电后看到白色LED灯闪烁，即此函数运行正常，欲校准正确，请详细查看以下这段话
 * 用途：在线方便校准电调，解决接收机单个电调校准的偏差，做到四个电机同步起转
 * @brief 首次校准后电调会自动保存参数，屏蔽此函数无需再次校准
          校准三步骤：
					①遥控上电，油门保持最高
					②飞行器整机装好（最好手压着飞行器或者不要上浆叶，防止意外事故），上电听到滴滴响声，根据自己的电调需求保持最高油门时间后拉油门到最低
					③再次听到滴滴响声即校准好电调，检验标准同步起转

					步骤顺序是这样，但时序还得根据你们自己电调的手册去抓，比如乐天电调听到滴滴后就要立刻拉低，迟了就没作用了
	@warining 由于校准过程是带着电操作电调和电机，请格外注意安全。
 * @autor 
   ZIN-A套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ******************************************************************************************/	
#include "LED.h"
#include "delay.h"
void ESC_Calibartion(void)
{
				TIM1_TIM4_PWM_IN_Init();//PWM捕获初始化
			  delay_ms(3000);//延时等待捕获到遥控数据
				if(Device.Remote.thr>1900)//上电时油门是否处于最高
				{
						LEDInit();
						LED.color = WHITE;//白色
						LED.status = Flash;
						LED.FlashTime = 100;//快闪
						TIM3_TIM5_TIM8_PWM_Out_Init(400,80);//PWM输出最高占空比百分之80						
				}
				else
				{
					return;
				}
				while(Device.Remote.thr>1050);//等待拉到最低
				TIM3_TIM5_TIM8_PWM_Out_Init(400,40);//PWM输出最低占空比百分之40	
				delay_ms(3000);	//延时等待3S，此时遥控不得有任何动作
				LED.status = AlwaysOn;
}
/***********************END OF FILE*************************************/







