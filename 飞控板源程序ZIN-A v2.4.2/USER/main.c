/**************************************************************
 * ZINС��
 * @brief
   ZIN-A�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
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
����1������
����2����λ��������
����3������
����4��GPS
����5������
����6������

I2C x 1 
��������ͬһ�������ϣ�֧��Ӳ��I2C�����ã�

SPI x 1
8pin�ӿڣ�����չNRF24L01

CAN x 1
STM�ڲ�����CAN��Ԫ����CAN���������£�ֻ֧�ִ���ƽת�������豸

ADC x 2
Ԥ����������ѹ���

USB x 1
USB_HID ��������λ��˫���� PID���� �ɿ�У׼

PWM_OUT x 8
���֧��8·��Ŀǰֻ�õ�4·֧��4�ᣬ�û�����չ6/8��

PWM_IN x 8
���֧��8ͨ��
			 
***************************************************************/
//�뽫��������ͷ������ǰ����
//ÿ���ϵ���н���ˮƽУ׼����ˮƽ��ֹ���ɿ��ϵ磬�ɿػ�����Զ�У׼
//         ��ͷ      
//   PWM2     ��       PWM1
//      *         	  *
//      	*       *
//    		  *   *
//      		*  
//    		  *   *
//      	*       *
//      *           *
//    PWM3           PWM4.
/**************************************************************
ָʾ��״̬
��ɫ��MPU6050����AK8975����SPL06����AT24C02�й��ϣ���ʼ��ʱ���Լ�
��ɫ��δ�������ɿ���������
��ɫ����������̬ģʽ
��ɫ������������ģʽ
��ɫ��������GPS����ģʽ
��ɫ��δ������������У׼
*****************************************************************
���ƹ��� 
���붨��ģʽ = ����ģʽ + ��̬ģʽ + GPS����   ����ʹ��ͬʱ�����ڶ��ߺ���̬����
���붨��ģʽ = ����ģʽ + ��̬ģʽ             ����ʹ��ͬʱ�����ڶ�����̬
������̬ģʽ = ��̬ģʽ                        ��̬ģʽΪ�������вٿ�
***************************************************************
ң��
ͨ��1�����  ͨ��2������   ͨ��3������   ͨ��4��ƫ��  ͨ��5�����߿���  ͨ��6�������
********************************************************************/



//�Ķ������Ϊ�������ߣ�5ms�жϣ��ɿس������н��ࣩ��������ѭ��ִ�в���Ҫ�Ķ���
uint8_t tmp_buf[33];
extern _st_pidData pidData;
#define desired Expect
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//�ж����ȼ��������
	SysTick_Configuration();  //�δ�ʱ������
	#if 0
	ESC_Calibartion();
	#endif
	//�������裬�豸��ʼ��
	ALL_Init();
	
	//������5ms����һ�Σ��뿴scheduler.c
  while(1)
	{
		ANTO_polling(); //��ѯ����������λ������
		
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


