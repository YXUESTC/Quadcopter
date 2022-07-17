/**************************************************************
 * ZIN小店
 * @brief
   ZIN-A套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "init.h"
#include "ANO_DT.h"
#include "AK8975.h"
#include "remote.h"
#include "nrf24l01.h"
#include "osc.h"
#include "ALL_DATA.h"
#include "pid.h"
#include "control.h"
#include "imu.h"
#include "us100.h"
#include "high_process.h"
#include "attitude_process.h"
/**************************************************************
USART1~6
串口1：备用
串口2：上位机、数传
串口3：备用
串口4：GPS
串口5：备用
串口6：备用

I2C x 1 
与陀螺仪同一条总线上，支持硬件I2C（慎用）

SPI x 1
8pin接口，可扩展NRF24L01

CAN x 1
STM内部集成CAN单元，但CAN的性质所致，只支持带电平转换器的设备

ADC x 2
预备电流、电压检测

USB x 1
USB_HID 免驱，上位机双向传输 PID调试 飞控校准

PWM_OUT x 8
最大支持8路，目前只用到4路支持4轴，用户可扩展6/8轴

PWM_IN x 8
最大支持8通道
			 
***************************************************************/
//请将飞行器机头置于正前方。
//每次上电进行进行水平校准，请水平静止给飞控上电，飞控会进行自动校准
//         机头      
//   PWM2     ♂       PWM1
//      *         	  *
//      	*       *
//    		  *   *
//      		*  
//    		  *   *
//      	*       *
//      *           *
//    PWM3           PWM4.
/**************************************************************
指示灯状态
红色：MPU6050或者AK8975或者SPL06或者AT24C02有故障，初始化时会自检
青色：未解锁，飞控正常运行
蓝色：解锁，姿态模式
绿色：解锁，定高模式
黄色：解锁，GPS定点模式
粉色：未解锁，磁力计校准
*****************************************************************
控制规则 
打入定点模式 = 定高模式 + 姿态模式 + GPS定点   定点使能同时运行在定高和姿态控制
打入定高模式 = 定高模式 + 姿态模式             定高使能同时运行在定高姿态
打入姿态模式 = 姿态模式                        姿态模式为正常飞行操控
***************************************************************
遥控
通道1：横滚  通道2：俯仰   通道3：油门   通道4：偏航  通道5：定高控制  通道6定点控制
********************************************************************/



//阅读程序分为两条主线：5ms中断（飞控程序运行节奏）、主函数循环执行不重要的东西
uint8_t tmp_buf[33];
extern _st_pidData pidData;
#define desired Expect
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//中断优先级组别设置
	SysTick_Configuration();  //滴答时钟配置
	#if 0
	ESC_Calibartion();
	#endif
	//所有外设，设备初始化
	ALL_Init();
	
	//飞行器5ms更新一次，请看scheduler.c
  while(1)
	{
		ANTO_polling(); //轮询发送匿名上位机数据
		
//  		OutData[0]=(int)(ultra_distance); 
//			OutData[1]=(int)(Height.Speed);
			OutData[2]=(int)(Height.High);
			OutData[3]=(int)(Attitude.acc_yaw_sensor);
		
		OutPut_Data();
				if(NRF24L01_TxPacket(tmp_buf)==TX_OK)
				{
						for(int i=0;i<10;i++)
							tmp_buf[i]=databuf[i];
				}  
	}
}


