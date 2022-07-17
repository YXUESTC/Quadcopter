/**************************************************************
 * 
 * @brief
   ZIN-7�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
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
 *  ͨ�����ݴ���
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	

static uint16_t RC_LOST;	
	
void remote_unlock(void);	

void RC_Analy(void)  
{
					//-----------------------ģʽ�л�-----------------------------------------------
					if(Command.FlightMode != LOCK) //�����������״̬�����ģʽ�ж�
					{
						if(Device.Remote.AUX2 >1700) //������ڶ���״̬
						{
								//if(GPS.satellite_num>9 &&GPS.Quality<3.0f && Earth_Position.Filter_Defeated_Flag)//��λ����С��10������ˮƽ��������С��3������  //�����ں�ʧ��
								//{
									Command.FlightMode = GPS_POSITION;
							//	}
						}
						else if(Device.Remote.AUX1>1700) //�����ڶ���״̬���ж��Ƿ��ڶ���״̬
						{				
							Command.FlightMode = HEIGHT;			
						}
						else
							Command.FlightMode = NORMOL;
						
					}
						
					//---------ģʽ�л��ı�Ƶ���˸״̬--------------
							switch(Command.FlightMode)
							{
									case LOCK:
										if(Command.AccOffset==0 && Command.GyroOffset == 0&&Command.MagOffset == 0 && Command.six_acc_offset == 0) //������У׼״̬
										{
//											LED.color = CYAN;
										}	
										break;
									case NORMOL://�������в���
										LED.color = BLUE;  
										break;									
									case HEIGHT://������ɫ
										LED.color = GREE;	 			
										break;
									case GPS_POSITION://������ɫ
										LED.color = WHITE;
										break;
									default:
										break;								
							}
					//---------------------------------------------------------------------------------
						remote_unlock();  //�����ж�							
					//---------------------------------------------------------------------------------
						if(RC_LOST++ > 300)//���1.5SûRC���Ź�û����0 ���ж�Ϊң���źŶ�ʧ����ʱӦ����������
							//��������Զ�����ģʽ�����������������䣬�������˵������з��С������Զ������1000�ף�ʵ����ò�Ҫ����300�ף����С�
						  //���ڷ���ǰ�����ߣ��ų������ɶ���������;ʧ�ء�
						{
								Device.Remote.yaw = Device.Remote.roll = Device.Remote.pitch = 1500;
								Device.Remote.thr = 1000;
								if( Command.FlightMode != LOCK)  //ʧ�ؽ��뽵��ģʽ
								{
									 Device.Remote.thr = 1200;
									 Command.FlightMode = HEIGHT; //���붨�߽���ģʽ
								}
						}
}



void Feed_Rc_Dog(void) //1S�ڱ������һ��
{
	RC_LOST = 0;
}

/*****************************************************************************************
 *  �����������ж�
 * @brief�����������У�
				 ͨ�ü򵥽�����ʽ������ң�����½���������ص�������ͺ���ɽ���������
				 ��������������THRС��1150��YAW����1800������޷��ﵽ������������λ���鿴���ң�������Ƿ����㡣
				 ͨ�ü�������ʽ������ң�����½������
				 ��������������THRС��1150��YAWС��1200��
 * @warining �ر����Ѣ٣����������ܼ�
						 �ر����Ѣڣ����������������ң����������������
 * @autor 
   ZIN-A�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ******************************************************************************************/	
void remote_unlock(void)
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

	if(Device.Remote.thr<1150 &&Device.Remote.yaw<1200)                         //����ң�����½�����
	{
		status = EXIT_255;
	}
	switch(status)
	{
		
			//ʹ�����½���
			case WAITING_1://�ȴ���������
				if(Device.Remote.thr<1050 &&Device.Remote.yaw>1800)                         //����ң�����½�����
				{
					status = WAITING_2;
				}
				break;
		case WAITING_2://��������������ȴ�ң�˻ظ���ʼ״̬
				if(Device.Remote.thr<1050 &&Device.Remote.yaw<1600)                         //����ң�����½�����
				{
					status = WAITING_4;
				}
				break;				
		case WAITING_4:	//����ǰ׼��	               
				Command.FlightMode = NORMOL;   //������־λ
				status = PROCESS_31;   //�������
										
				 break;		
		case PROCESS_31:	//�������״̬ 

				if(Device.Remote.thr<1020)
				{
					if(cnt++ > 3000 && Command.FlightMode == NORMOL)   // ��̬״̬������15S�Զ�����
					{								
						status = EXIT_255;								
					}
				}				
				if(Command.FlightMode == LOCK)                           //�������������ֱ�������ɿ�
				{
					status = EXIT_255;				
				}
				else					
					cnt = 0;
			break;
		case EXIT_255: //��������
			LED.color = CYAN;     //����������ɫ��˸		
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
 * �����������У׼
 * ʹ�÷������򿪴˺������ϵ�󿴵���ɫLED����˸�����˺���������������У׼��ȷ������ϸ�鿴������λ�
 * ��;�����߷���У׼�����������ջ��������У׼��ƫ������ĸ����ͬ����ת
 * @brief �״�У׼�������Զ�������������δ˺��������ٴ�У׼
          У׼�����裺
					��ң���ϵ磬���ű������
					�ڷ���������װ�ã������ѹ�ŷ��������߲�Ҫ�Ͻ�Ҷ����ֹ�����¹ʣ����ϵ������ε������������Լ��ĵ�����󱣳��������ʱ��������ŵ����
					���ٴ������ε�������У׼�õ���������׼ͬ����ת

					����˳������������ʱ�򻹵ø��������Լ�������ֲ�ȥץ�����������������εκ��Ҫ�������ͣ����˾�û������
	@warining ����У׼�����Ǵ��ŵ��������͵���������ע�ⰲȫ��
 * @autor 
   ZIN-A�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ******************************************************************************************/	
#include "LED.h"
#include "delay.h"
void ESC_Calibartion(void)
{
				TIM1_TIM4_PWM_IN_Init();//PWM�����ʼ��
			  delay_ms(3000);//��ʱ�ȴ�����ң������
				if(Device.Remote.thr>1900)//�ϵ�ʱ�����Ƿ������
				{
						LEDInit();
						LED.color = WHITE;//��ɫ
						LED.status = Flash;
						LED.FlashTime = 100;//����
						TIM3_TIM5_TIM8_PWM_Out_Init(400,80);//PWM������ռ�ձȰٷ�֮80						
				}
				else
				{
					return;
				}
				while(Device.Remote.thr>1050);//�ȴ��������
				TIM3_TIM5_TIM8_PWM_Out_Init(400,40);//PWM������ռ�ձȰٷ�֮40	
				delay_ms(3000);	//��ʱ�ȴ�3S����ʱң�ز������κζ���
				LED.status = AlwaysOn;
}
/***********************END OF FILE*************************************/







