/**************************************************************
 *  ������λ��������봮�ڣ�������50��
 * @brief
   ZIN-7�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include <stdlib.h>
#include <string.h>
#include "ALL_DATA.h"
#include "ANO_DT.h"
#include "USART.h"
#include "delay.h"
#include "usbd_user_hid.h"
#include "pid.h"
#include "imu.h"
#include "GPS.h"
#include "postion_process.h"
#include "high_process.h"
#include "attitude_process.h"
#include "control.h"
#include "us100.h"
#include "spl06.h"
/******************************************************************/
//--------------------------
//��ʱ������λ����������PID���� ��ֹ�������������������һ�鸲��
static uint8_t RatePID[18];
static uint8_t AnglePID[18];
static uint8_t HighPID[18];

//���յ���λ�������������־λ
static struct{
	uint8_t PID1 :1; //���ܵ���λ��PID��1
	uint8_t PID2 :1; //���ܵ���λ��PID��2
	uint8_t PID3 :1; //���ܵ���λ��PID��3
	uint8_t PID4 :1; //���ܵ���λ��PID��4
	uint8_t PID5 :1; //���ܵ���λ��PID��5
	uint8_t PID6 :1; //���ܵ���λ��PID��6	
	uint8_t CMD2_READ_PID:1; //���ܵ���λ����ȡPID������
}ANTO_Recived_flag;


int16_t checkPID;


#define ANTO_RATE_PID  ANTO_PID1
#define ANTO_ANGLE_PID  ANTO_PID2
#define ANTO_HEIGHT_PID  ANTO_PID3
/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/	
void ANO_Recive(int8_t *pt)                   //���յ���λ��������
{
	switch(pt[2])
	{
		case ANTO_RATE_PID:
			ANTO_Recived_flag.PID1 = 1;             //���յ���λ��������PID����
			memcpy(RatePID,&pt[4],18);              //�Ȱѽ��յ����������������ֹ����һ��PID���ݸ��ǣ������PID�Ǹ��ٶȻ��õ�
			break;
		case ANTO_ANGLE_PID:                      //�����PID�Ǹ��ǶȻ��õ�
			memcpy(AnglePID,&pt[4],18);
			ANTO_Recived_flag.PID2 = 1;
			break;
		case ANTO_HEIGHT_PID:                     //�����PID�Ǹ��߶Ȼ��õ�
			memcpy(HighPID,&pt[4],18);
			ANTO_Recived_flag.PID3 = 1;
			break;
		case ANTO_PID4:
			ANTO_Recived_flag.PID4 = 1;
			break;
		case ANTO_PID5: 
			ANTO_Recived_flag.PID5 = 1;	
			break;
		case ANTO_PID6:
			ANTO_Recived_flag.PID6 = 1;
			break;
		case 0x01:                                //��λ��������CMD1 ��������У׼
			{
					enum                                  //��λ����ɿ�����
					{
						ACCOFFSET = 0x01,
						GYROOFFSET = 0x02,
						MAGOFFET = 0X04,
						BAROOFFSET = 0x05,
						EXIT_SIX_ACC_OFFSET = 0X20,//�˳�����У׼
						FIRST_SIX_ACC_OFFSET = 0X21,//����У׼,��һ��
						SECOND_SIX_ACC_OFFSET = 0X22,//����У׼���ڶ���
						THREE_SIX_ACC_OFFSET = 0X23,//����У׼��������
						FOUR_SIX_ACC_OFFSET = 0X24,//����У׼�����Ĳ�
						FIVE_SIX_ACC_OFFSET = 0X25,//����У׼�����岽
						SIX_SIX_ACC_OFFSET = 0X26,//����У׼��������
					};
					switch(*(uint8_t*)&pt[4])
					{
						case ACCOFFSET:
							Command.AccOffset = 1;
							break;
						case GYROOFFSET:
							Command.GyroOffset = 1;
							break;						
						case MAGOFFET:
							Command.MagOffset = 1;
							break;
						case BAROOFFSET:
							break;
					  case EXIT_SIX_ACC_OFFSET:
							Command.six_acc_offset = 0x20;
							break;
					  case FIRST_SIX_ACC_OFFSET:
							Command.six_acc_offset = 0x21;
							break;
					  case SECOND_SIX_ACC_OFFSET:
							Command.six_acc_offset = 0x22;
							break;
					  case THREE_SIX_ACC_OFFSET:
							Command.six_acc_offset = 0x23;
							break;
					  case FOUR_SIX_ACC_OFFSET:
							Command.six_acc_offset = 0x24;
							break;
					  case FIVE_SIX_ACC_OFFSET:
							Command.six_acc_offset = 0x25;
							break;
					  case SIX_SIX_ACC_OFFSET:
							Command.six_acc_offset = 0x26;
							break;						
						default: 
							break;						
					}
			}
			break;
		case 0x02:                                //��λ��������CMD2 ���������ȡPID��
			{
			   enum                                  //��λ����ɿ�����
				{
					READ_PID = 0X01,                    //��ȡ�ɿص�PID����
					READ_MODE = 0x02,                   //��ȡ����ģʽ
					READ_ROUTE = 0x21,                  //��ȡ������Ϣ
					READ_VERSION = 0XA0,                //��ȡ�ɿذ汾
					RETURN_DEFAULT_PID = 0xA1           //�ָ�Ĭ��PID
				 };

				switch(*(uint8_t*)&pt[4])             //�ж���λ������CMD������
				{
					case READ_PID:                      //��λ�������ȡ�ɿ�PID����
						ANTO_Recived_flag.CMD2_READ_PID = 1;
						break;
					case READ_MODE: 
						break;
					case READ_ROUTE: 
						break;					
					case READ_VERSION:  
						break;
					case RETURN_DEFAULT_PID:  
						break;					
					default: 
						break;					
				}
			
			}
			break;
		case ANTO_RCDATA: //Immediately deal with 
			break;
		default:
			break;			
	}
	return;
}
/***********************************************************************
 * //�������ݵ���λ��
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void ANTO_Send(const enum ANTO_SEND FUNCTION) 
{
	uint8_t i;
	uint8_t len=2;
	int16_t Anto[15];
	int8_t *pt = (int8_t*)(Anto);
	PidObject *pidX=0;
	PidObject *pidY=0;
	PidObject *pidZ=0;

	switch(FUNCTION)
	{
		case ANTO_RATE_PID:      //����PID1����λ��
				 pidX = &pidData.RateX;   //ָ����������Ϊ���ٶ��ڻ�
				 pidY = &pidData.RateY;
				 pidZ = &pidData.RateZ;
         goto send_pid;		
		case ANTO_ANGLE_PID:       //����PID2����λ��
				 pidX = &pidData.Roll;
				 pidY = &pidData.Pitch;
				 pidZ = &pidData.Yaw;
				 goto send_pid;				
		case ANTO_HEIGHT_PID:     //����PID3����λ��			
				 pidX = &pidData.HeightRate; //�ٶȻ�PID
				 pidY = &pidData.HeightHigh;  //�߶Ȼ�PID	
				 pidZ = &pidData.HeightAccel; //���ٶȻ�PID	
				 goto send_pid;							
		case ANTO_PID4:	  //PID4
		case ANTO_PID5:	         //PID5
    case ANTO_PID6:
send_pid:
			if(pidX!=NULL)

			{
				Anto[2] = (int16_t)(pidX->Kp *1000);
				Anto[3] = (int16_t)(pidX->Ki *1000);
				Anto[4] = (int16_t)(pidX->Kd *1000);
			}
			if(pidY!=NULL)
			{
				Anto[5] = (int16_t)(pidY->Kp *1000);
				Anto[6] = (int16_t)(pidY->Ki *1000);
				Anto[7] = (int16_t)(pidY->Kd *1000);
			}
			if(pidZ!=NULL)
			{
				Anto[8] = (int16_t)(pidZ->Kp *1000);
				Anto[9] = (int16_t)(pidZ->Ki *1000);
				Anto[10] = (int16_t)(pidZ->Kd *1000);
			}
			len = 18;
			break;
		case ANTO_MOTOR:    //send motor

				len = 8;
			break;	
		case ANTO_RCDATA: //send RC data
			Anto[2] = Device.Remote.thr;
			Anto[3] = Device.Remote.yaw;
			Anto[4] = Device.Remote.roll;
			Anto[5] = Device.Remote.pitch;
			Anto[6] = MOTOR1; //Device.Remote.AUX1;
			Anto[7] = MOTOR2;//Device.Remote.AUX2;
			Anto[8] = MOTOR3;//Device.Remote.AUX3;
			Anto[9] = MOTOR4;//Device.Remote.AUX4;
			Anto[10] = 0; 
			Anto[11] = 0; 
			len = 20;
			break;
		case ANTO_MPU_MAGIC:     //����MPU6050�ʹ����Ƶ�����
//			memcpy(&Anto[2],(int8_t*)&Device.MPU6050,sizeof(Device.MPU6050));//��ԭʼ������
			Anto[2] = Attitude.accX_IMU;//��������������
			Anto[3] = Attitude.accY_IMU;
			Anto[4] = Attitude.accZ_IMU;
			Anto[5] = Attitude.gyroX_IMU;
			Anto[6] = Attitude.gyroY_IMU;
			Anto[7] = Attitude.gyroZ_IMU;  
//			memcpy(&Anto[8],(int8_t*)&Device.AK8975,sizeof(Device.AK8975));
			len = 18;
			break;
		case ANTO_SENSER2:
			{
				long _temp =(long) ultra_distance*100;//FlightData.High.bara_height;
				*(uint16_t*)&Anto[2] = *((uint16_t*)&_temp+1);
				*(uint16_t*)&Anto[3] = *((uint16_t*)&_temp);
				Anto[4] = (uint32_t)(Height.Speed*100);//FlightData.High.ultra_height;Height.High
				len = 6;
			}
			break;
		case ANTO_GPSDATA:
//			Anto[2] = (GPS.status << 8) | GPS.satellite_num; //��λ״̬����������
//			int32_t __temp = 10000000 * GPS.Longitude_W_E_Position; //���������HOME���λ����Ϣ
//			Anto[3] =  *((uint16_t*)&__temp+1);
//			Anto[4] =  *((uint16_t*)&__temp);
//			__temp = 10000000 *  GPS.Lattitude_N_S_Position; //γ�� 
//			Anto[5] =  *((uint16_t*)&__temp+1);
//			Anto[6] =  *((uint16_t*)&__temp);
//			Anto[7]	=  GPS.Angle;	
//			len = 12;
			break;
		case ANTO_SPEED:
			{
//				int16_t temp = GPS.Longitude_W_E_speed*100;
//				Anto[2] =  *((uint16_t*)&temp);
//				temp = GPS.Lattitude_N_S_speed*100;
//				Anto[3] =  *((uint16_t*)&temp);					
//				temp = Height.Speed*100;
//				Anto[4] =  *((uint16_t*)&temp);
//							
//				len = 6;
			}
			break;
		case ANTO_STATUS:     //send angle
			
				Anto[2] =(int16_t)(IMU.roll*100);
				Anto[3] = (int16_t)(IMU.pitch*100);
				Anto[4] = (int16_t)(-IMU.yaw*100);
				Anto[5] = 0;
				Anto[6] = 0;
				switch(Command.FlightMode)
				{
						case LOCK:
							Anto[7] = 0;  //��λ����ģʽ
							break;
						case NORMOL:
							 Anto[7] = 0x0101;
							break;
						case HEIGHT:
							Anto[7] = 0x0201;
						  break;
						case GPS_POSITION:
							Anto[7] = 0x0301;
							break;
						default:
							Anto[7] = 0;  
							break;
				}

				len = 12;
			break;
		case ANTO_POWER:

				break;
		case ANTO_CHECK:
				Anto[2] = checkPID;
				len = 2;
				break;		
		default:
			break;			
	}
	
	Anto[0] = 0XAAAA;
	Anto[1] = len | FUNCTION<<8;
	pt[len+4] = (int8_t)(0xAA+0xAA);
	for(i=2;i<len+4;i+=2)    //a swap with b;
	{
		pt[i] ^= pt[i+1];
		pt[i+1] ^= pt[i];
		pt[i] ^= pt[i+1];
		pt[len+4] += pt[i] + pt[i+1];
	}	

#ifdef 	USB_HID_TO_PC
	Usb_Hid_Adddata((u8*)pt,len+5);
	
	Usb_Hid_Send();	
#endif
#ifdef 	USART2_TO_PC
	USART2_Send((const u8*)pt,len+5);
#endif
}


/***********************************************************************
 * ��ѯɨ����λ���˿�.
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void ANTO_polling(void) //
{
	volatile static uint8_t status = 0;
	switch(status)
	{
		case 0:
			
			status = 1;
			break;
		case 1:
			ANTO_Send(ANTO_MPU_MAGIC);
			delay_ms(2);
			ANTO_Send(ANTO_SENSER2);
			delay_ms(2);
			ANTO_Send(ANTO_RCDATA);
			delay_ms(2);
			ANTO_Send(ANTO_STATUS);
			delay_ms(2);
			ANTO_Send(ANTO_GPSDATA);
			delay_ms(2);	
			ANTO_Send(ANTO_SPEED);
			delay_ms(2);	
			if(*(uint8_t*)&ANTO_Recived_flag != 0) //һ�����յ���λ�������ݣ�����ͣ�������ݵ���λ����ת��ȥ�ж���λ��Ҫ��ɿ���ʲô��
			{
				status = 2;
			}
		 	break;
		case 2:
			if(*(uint8_t*)&ANTO_Recived_flag == 0)//��λ���ķ����������ݶ��������ˣ��򷵻�ר�ĵķ������ݵ���λ��
			{
				status = 1;
			}
	
			if(ANTO_Recived_flag.CMD2_READ_PID) //�ж���λ���Ƿ����󷢷��ͷɿ�PID���ݵ���λ��
			{		
					ANTO_Send(ANTO_RATE_PID);
					delay_ms(1);
					ANTO_Send(ANTO_ANGLE_PID);
					delay_ms(1);
					ANTO_Send(ANTO_HEIGHT_PID);
					delay_ms(1);
					ANTO_Recived_flag.CMD2_READ_PID = 0;
			}
			
			if(*(uint8_t*)&ANTO_Recived_flag & 0x3f) //���յ���λ��������PID����
			{
					PidObject *pidX=0;
					PidObject *pidY=0;
					PidObject *pidZ=0;
				  uint8_t *P;
				
					if(ANTO_Recived_flag.PID1)
					{
						 pidX = &pidData.RateX;
						 pidY = &pidData.RateY;
						 pidZ = &pidData.RateZ;
						 P = RatePID;
						 ANTO_Recived_flag.PID1 = 0;
					}
					else if(ANTO_Recived_flag.PID2)
					{
						 pidX = &pidData.Roll;
						 pidY = &pidData.Pitch;
						 pidZ = &pidData.Yaw;
						 P = AnglePID;	
						 ANTO_Recived_flag.PID2 = 0;                             
					}
					else if(ANTO_Recived_flag.PID3)
					{
						 pidX = &pidData.HeightRate;
						 pidY = &pidData.HeightHigh;
						 pidZ = &pidData.HeightAccel;
						 P = HighPID;	
						 ANTO_Recived_flag.PID3 = 0;     					
					}
					else if(ANTO_Recived_flag.PID4)
					{
						ANTO_Recived_flag.PID4 = 0; 
					}
					else if(ANTO_Recived_flag.PID5)
					{
						ANTO_Recived_flag.PID5 = 0; 
					}
					else if(ANTO_Recived_flag.PID6)
					{
						ANTO_Recived_flag.PID6 = 0; 
					}	
				
					{
							union {
								uint16_t _16;
								uint8_t _u8[2];
							}data;
							
							if(pidX!=NULL)
							{
								data._u8[1] = P[0]; 
								data._u8[0] = P[1];
								pidX->Kp =  data._16 /1000.0f;
								data._u8[1] = P[2]; 
								data._u8[0] = P[3];
								pidX->Ki =  data._16 /1000.0f;
								data._u8[1] = P[4]; 
								data._u8[0] = P[5];
								pidX->Kd =  data._16 /1000.0f;				

				
							}
							if(pidY!=NULL)
							{
								data._u8[1] = P[6]; 
								data._u8[0] = P[7];
								pidY->Kp =  data._16 /1000.0f;
								data._u8[1] = P[8]; 
								data._u8[0] = P[9];
								pidY->Ki =  data._16 /1000.0f;
								data._u8[1] = P[10]; 
								data._u8[0] = P[11];
								pidY->Kd =  data._16 /1000.0f;		
							}
							if(pidZ!=NULL)
							{
								data._u8[1] = P[12]; 
								data._u8[0] = P[13];
								pidZ->Kp =  data._16 /1000.0f;
								data._u8[1] = P[14]; 
								data._u8[0] = P[15];
								pidZ->Ki =  data._16 /1000.0f;
								data._u8[1] = P[16]; 
								data._u8[0] = P[17];
								pidZ->Kd =  data._16 /1000.0f;		
							}				
					}				
			}
			break;
		default:
			break;
	}

}
/***********************************************************************
 * ���մ��ڷ�����������
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void USART2_IRQHandler(void) //���ڽ���
{ 	
  static  int8_t ReciveBuffer[25];
  static uint8_t count;
	static int8_t CheckSum;
		
	if ((USART2->SR & USART_IT_ORE))//�Ƿ���ռĴ������
	{
	
	}	
  if((USART2->SR & USART_IT_RXNE))
  {
		ReciveBuffer[count] = USART2->DR;	
		switch(count)
		{
		case 0:
			if(ReciveBuffer[0]==(int8_t)0xAA)
				count++;
				break;
		case 1:	
			if(ReciveBuffer[1]==(int8_t)0xAF)
			{
					CheckSum = (int8_t)(0xAA + 0xAF);
					count++;
			}
			else 
					count = 0;
				break;
		default:
			if(count < ReciveBuffer[3]+4)
			{
				CheckSum += ReciveBuffer[count];
				count++;
				break;
			}
			else
			{
//				uint8_t i;
//				int8_t CheckSum=0;
//							
//				for(i=0;i<count;i++)
//				{
//					CheckSum += ReciveBuffer[i];			
//				}		
				if(CheckSum == ReciveBuffer[count])  //if the data calculate sum equal to the  final data from PC.
				{
//						static int8_t CheckSend[7]={0xAA,0XAA,0xEF,2,0,0,0};	
//						static int8_t temp = (int8_t)(0xAA+0XAA+0XEF+2);
//						CheckSend[4] = ;
//						CheckSend[5] = CheckSum;
//						CheckSend[6] = temp+ReciveBuffer[2]+CheckSum;
//						USART2_Send((uint8_t*)CheckSend,7);
							if(ReciveBuffer[2] >= 0x10 && ReciveBuffer[2]<=0x15)
							{	
									checkPID = ReciveBuffer[2]<<8 | CheckSum;
									ANTO_Send(ANTO_CHECK);
							}			
							ANO_Recive(ReciveBuffer);			//To arrange the data	and give the result to control argument.	
				}			
				count = 0;                  //return to the first data point,and retore from the head buffer next time.
				ReciveBuffer[0] = 0;  //reset the data buffer.
				ReciveBuffer[1] = 0;
			}
			break;							
		}
	}
}



//void USART3_IRQHandler(void) //���ڽ���
//{ 	
//  static  int8_t ReciveBuffer[25];
//  static uint8_t count;
//	static int8_t CheckSum;
//		
//	if ((USART3->SR & USART_IT_ORE))//�Ƿ���ռĴ������
//	{
//	
//	}	
//  if((USART3->SR & USART_IT_RXNE))
//  {
//		ReciveBuffer[count] = USART3->DR;	
//		switch(count)
//		{
//		case 0:
//			if(ReciveBuffer[0]==(int8_t)0xAA)
//				count++;
//				break;
//		case 1:	
//			if(ReciveBuffer[1]==(int8_t)0xAF)
//			{
//					CheckSum = (int8_t)(0xAA + 0xAF);
//					count++;
//			}
//			else 
//					count = 0;
//				break;
//		default:
//			if(count < ReciveBuffer[3]+4)
//			{
//				CheckSum += ReciveBuffer[count];
//				count++;
//				break;
//			}
//			else
//			{
////				uint8_t i;
////				int8_t CheckSum=0;
////							
////				for(i=0;i<count;i++)
////				{
////					CheckSum += ReciveBuffer[i];			
////				}		
//				if(CheckSum == ReciveBuffer[count])  //if the data calculate sum equal to the  final data from PC.
//				{
////						static int8_t CheckSend[7]={0xAA,0XAA,0xEF,2,0,0,0};	
////						static int8_t temp = (int8_t)(0xAA+0XAA+0XEF+2);
////						CheckSend[4] = ;
////						CheckSend[5] = CheckSum;
////						CheckSend[6] = temp+ReciveBuffer[2]+CheckSum;
////						USART2_Send((uint8_t*)CheckSend,7);
//							if(ReciveBuffer[2] >= 0x10 && ReciveBuffer[2]<=0x15)
//							{	
//									checkPID = ReciveBuffer[2]<<8 | CheckSum;
//									ANTO_Send(ANTO_CHECK);
//							}			
//							ANO_Recive(ReciveBuffer);			//To arrange the data	and give the result to control argument.	
//				}			
//				count = 0;                  //return to the first data point,and retore from the head buffer next time.
//				ReciveBuffer[0] = 0;  //reset the data buffer.
//				ReciveBuffer[1] = 0;
//			}
//			break;							
//		}
//	}
//}

/************************END OF FILE********************/





