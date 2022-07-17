/**************************************************************
 * 
 * @brief
   ZIN-7�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
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
 * ����ȫ�ֱ�����ʼ��
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
 
//�豸������
__align(4)  _st_Device Device;
//��������
 _st_FlightData FlightData;
//�ɿ�����
 _st_Command Command;


//�豸���
__align(4) _st_DeviceCheck DeviceCheck;
//������Ҫ��ȫ�ֱ���
volatile uint32_t SysTick_count; //ϵͳ�δ�ʱ��
void pid_param_Init(void); //PID���Ʋ�����ʼ������дPID�����ᱣ�����ݣ��������ɺ�ֱ���ڳ�������� ����¼���ɿ�


/**************************************************************
 *  ����ϵͳ����ʹ�������ʼ��
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void ALL_Init(void)
{	
	volatile static uint8_t status = 0;
	
//----------�������ó�ʼ��-----------------
	Total_PID_Init();
	LEDInit();
	Total_PID_Init();         //PID������ʼ��	
	Usb_Hid_Init();						//�ɿ�usb�ӿڵ�hid��ʼ��
	TIM1_TIM4_PWM_IN_Init();
	TIM3_TIM5_TIM8_PWM_Out_Init(400,40); //400hz��PWM���//��ʼռ�ձ�40%
	for(int i=0; i<2; i++) delay_ms(1000);
	IICx_Init();
	/*****************24L01*****************/
	NRF24L01_Configuration();
	My_NRF24L01_Init();
	  while(NRF24L01_Check())	//���NRF24L01�Ƿ���λ.		
	{
		LED.color = RED; 
	}
	NRF24L01_TX_Mode();
	
	
  UART5_init(9600);  //��������ʼ��
	USART2_init(115200);  //2M�����ʣ���������
	
//	UART4_init(9600); //GPS���ڳ�ʼ��
	
//----------���������豸��ʼ��-----------------	
//	DeviceCheck.MPU6050 =  MpuInit();              //MPU6050��ʼ��		
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

//----------����ɨ��ִ�г�ʼ��-----------------	
	TIM2_config();					//ϵͳ�������ڳ�ʼ�� 

}













