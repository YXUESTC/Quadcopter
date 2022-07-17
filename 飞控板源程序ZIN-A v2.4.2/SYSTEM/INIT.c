/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "ALL_DEFINE.h"
#include "usbd_user_hid.h"
#include "pid.h"
#include "ALL_DATA.h"
#include "TIM.h"
#include "i2c.h"
#include "USART.h"
#include "MPU6050.h"
#include "AK8975.h"
#include "spl06.h"
#include "AT24C02.h"
#include "GPS.h"
#include "nrf24l01.h"
#include "senror.h"
#include "delay.h"
#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

/**************************************************************
 * 所有全局变量初始化
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
 
//设备的数据
__align(4)  _st_Device Device;
//飞行数据
 _st_FlightData FlightData;
//飞控命令
 _st_Command Command;


//设备检查
__align(4) _st_DeviceCheck DeviceCheck;
//其它重要的全局变量
volatile uint32_t SysTick_count; //系统滴答时钟
void pid_param_Init(void); //PID控制参数初始化，改写PID并不会保存数据，请调试完成后直接在程序里更改 再烧录到飞控


/**************************************************************
 *  整个系统外设和传感器初始化
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void ALL_Init(void)
{	
	volatile static uint8_t status = 0;
	
//----------功能配置初始化-----------------
	Total_PID_Init();
	LEDInit();
	Total_PID_Init();         //PID参数初始化	
	Usb_Hid_Init();						//飞控usb接口的hid初始化
	TIM1_TIM4_PWM_IN_Init();
	TIM3_TIM5_TIM8_PWM_Out_Init(400,40); //400hz的PWM输出//初始占空比40%
	for(int i=0; i<2; i++) delay_ms(1000);
	IICx_Init();
	/*****************24L01*****************/
	NRF24L01_Configuration();
	My_NRF24L01_Init();
	  while(NRF24L01_Check())	//检查NRF24L01是否在位.		
	{
		LED.color = RED; 
	}
	NRF24L01_TX_Mode();
	
	
  UART5_init(9600);  //超声波初始化
	USART2_init(115200);  //2M波特率，匿名数传
	
//	UART4_init(9600); //GPS串口初始化
	
//----------传感器等设备初始化-----------------	
//	DeviceCheck.MPU6050 =  MpuInit();              //MPU6050初始化		
//	DeviceCheck.AK8975  =  AK8975_init();
//	DeviceCheck.AT24C02  =  test_AT24C02();
//	DeviceCheck.SPL06  =  spl0601_init();
   while(MPU6050_Init())
	{
    LED.color = RED; 
	}
	delay_ms(2000);
	mpu6050_calibrate();
	delay_ms(1000);
	while(SPL06_001_Init())
	{
		LED.color = RED;
	}
	
//	AK8975_Init();
	
	if(*(uint32_t*)&DeviceCheck >=FAILED)
	{
	  	SYSTEM_ERROR;
		  while(1);
	}	
	else
	{
			SYSTEM_NORMOL;
	}

//----------启动扫描执行初始化-----------------	
	TIM2_config();					//系统工作周期初始化 

}













