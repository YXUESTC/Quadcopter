/**************************************************************
 *  匿名上位机，请接入串口，波特率50万
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
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
//临时保存上位机发过来的PID数据 防止数据来不及保存而被下一组覆盖
static uint8_t RatePID[18];
static uint8_t AnglePID[18];
static uint8_t HighPID[18];

//接收到上位机的数据种类标志位
static struct{
	uint8_t PID1 :1; //接受到上位机PID组1
	uint8_t PID2 :1; //接受到上位机PID组2
	uint8_t PID3 :1; //接受到上位机PID组3
	uint8_t PID4 :1; //接受到上位机PID组4
	uint8_t PID5 :1; //接受到上位机PID组5
	uint8_t PID6 :1; //接受到上位机PID组6	
	uint8_t CMD2_READ_PID:1; //接受到上位机读取PID的请求
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
void ANO_Recive(int8_t *pt)                   //接收到上位机的数据
{
	switch(pt[2])
	{
		case ANTO_RATE_PID:
			ANTO_Recived_flag.PID1 = 1;             //接收到上位机发来的PID数据
			memcpy(RatePID,&pt[4],18);              //先把接收到的数据提出来，防止被下一组PID数据覆盖，这组的PID是给速度环用的
			break;
		case ANTO_ANGLE_PID:                      //这组的PID是给角度环用的
			memcpy(AnglePID,&pt[4],18);
			ANTO_Recived_flag.PID2 = 1;
			break;
		case ANTO_HEIGHT_PID:                     //这组的PID是给高度环用的
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
		case 0x01:                                //上位机发来的CMD1 包含各种校准
			{
					enum                                  //上位请求飞控类型
					{
						ACCOFFSET = 0x01,
						GYROOFFSET = 0x02,
						MAGOFFET = 0X04,
						BAROOFFSET = 0x05,
						EXIT_SIX_ACC_OFFSET = 0X20,//退出六面校准
						FIRST_SIX_ACC_OFFSET = 0X21,//六面校准,第一步
						SECOND_SIX_ACC_OFFSET = 0X22,//六面校准，第二步
						THREE_SIX_ACC_OFFSET = 0X23,//六面校准，第三步
						FOUR_SIX_ACC_OFFSET = 0X24,//六面校准，第四步
						FIVE_SIX_ACC_OFFSET = 0X25,//六面校准，第五步
						SIX_SIX_ACC_OFFSET = 0X26,//六面校准，第六步
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
		case 0x02:                                //上位机发来的CMD2 包含请求读取PID等
			{
			   enum                                  //上位请求飞控类型
				{
					READ_PID = 0X01,                    //读取飞控的PID请求
					READ_MODE = 0x02,                   //读取飞行模式
					READ_ROUTE = 0x21,                  //读取航点信息
					READ_VERSION = 0XA0,                //读取飞控版本
					RETURN_DEFAULT_PID = 0xA1           //恢复默认PID
				 };

				switch(*(uint8_t*)&pt[4])             //判断上位机发来CMD的内容
				{
					case READ_PID:                      //上位机请求读取飞控PID数据
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
 * //发送数据到上位机
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
		case ANTO_RATE_PID:      //发送PID1到上位机
				 pidX = &pidData.RateX;   //指定发送数据为角速度内环
				 pidY = &pidData.RateY;
				 pidZ = &pidData.RateZ;
         goto send_pid;		
		case ANTO_ANGLE_PID:       //发送PID2到上位机
				 pidX = &pidData.Roll;
				 pidY = &pidData.Pitch;
				 pidZ = &pidData.Yaw;
				 goto send_pid;				
		case ANTO_HEIGHT_PID:     //发送PID3到上位机			
				 pidX = &pidData.HeightRate; //速度环PID
				 pidY = &pidData.HeightHigh;  //高度环PID	
				 pidZ = &pidData.HeightAccel; //加速度环PID	
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
		case ANTO_MPU_MAGIC:     //发送MPU6050和磁力计的数据
//			memcpy(&Anto[2],(int8_t*)&Device.MPU6050,sizeof(Device.MPU6050));//最原始的数据
			Anto[2] = Attitude.accX_IMU;//六面矫正后的数据
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
//			Anto[2] = (GPS.status << 8) | GPS.satellite_num; //定位状态，卫星数量
//			int32_t __temp = 10000000 * GPS.Longitude_W_E_Position; //经度相对于HOME点的位移信息
//			Anto[3] =  *((uint16_t*)&__temp+1);
//			Anto[4] =  *((uint16_t*)&__temp);
//			__temp = 10000000 *  GPS.Lattitude_N_S_Position; //纬度 
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
							Anto[7] = 0;  //高位飞行模式
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
 * 轮询扫描上位机端口.
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
			if(*(uint8_t*)&ANTO_Recived_flag != 0) //一旦接收到上位机的数据，则暂停发送数据到上位机，转而去判断上位机要求飞控做什么。
			{
				status = 2;
			}
		 	break;
		case 2:
			if(*(uint8_t*)&ANTO_Recived_flag == 0)//上位机的发过来的数据都被处理了，则返回专心的发送数据到上位机
			{
				status = 1;
			}
	
			if(ANTO_Recived_flag.CMD2_READ_PID) //判断上位机是否请求发发送飞控PID数据到上位机
			{		
					ANTO_Send(ANTO_RATE_PID);
					delay_ms(1);
					ANTO_Send(ANTO_ANGLE_PID);
					delay_ms(1);
					ANTO_Send(ANTO_HEIGHT_PID);
					delay_ms(1);
					ANTO_Recived_flag.CMD2_READ_PID = 0;
			}
			
			if(*(uint8_t*)&ANTO_Recived_flag & 0x3f) //接收到上位机发来的PID数据
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
 * 接收串口发过来的数据
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void USART2_IRQHandler(void) //串口接收
{ 	
  static  int8_t ReciveBuffer[25];
  static uint8_t count;
	static int8_t CheckSum;
		
	if ((USART2->SR & USART_IT_ORE))//是否接收寄存器溢出
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



//void USART3_IRQHandler(void) //串口接收
//{ 	
//  static  int8_t ReciveBuffer[25];
//  static uint8_t count;
//	static int8_t CheckSum;
//		
//	if ((USART3->SR & USART_IT_ORE))//是否接收寄存器溢出
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





