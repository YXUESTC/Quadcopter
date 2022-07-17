/**************************************************************
 * @brief
   ZIN-A ���������ذ�
	 �ɿذ���Ⱥ551883670
 * @attention
		�빺����ΪZINС��Ĵ����ṩ������
		��������ֲ��������ʽʹ�ã����벻Ҫ�ŵ����ϡ�лл��ң�ף���ѧϰ��졣
 * @brief	
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include <math.h>
#include "mpu6050.h"
#include "imu.h"
#include "attitude_process.h"
#include "postion_process.h"
#include "GPS.h"



//_st_Earth_Position Earth_Position;


////Longitude����
////Latitudeγ��

/**************************************************************
 * ���׻����㷨  λ�� �ٶ� ���ٶ�
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
//#define TIME_CONTANST_XY      2.5f //����
//#define K_ACC_XY	     (1.0f / (TIME_CONTANST_XY * TIME_CONTANST_XY * TIME_CONTANST_XY))
//#define K_VEL_XY             (3.0f / (TIME_CONTANST_XY * TIME_CONTANST_XY))														
//#define K_POS_XY             (3.0f / TIME_CONTANST_XY)


////Longitude����  ��ͷ��������
////Latitudeγ��   


//#define  Num 30

//void Strapdown_INS_Horizontal(void)
//{
//			 float CNTLCYCLE; //��ȷ���ںϼ����
//	    {
//			  static	float last_time;
//				float now_time;
//				now_time = micros();
//				CNTLCYCLE = now_time - last_time;
//				CNTLCYCLE /=1000;
//				last_time = now_time;
//			
//			}	
//			const uint8_t GPS_SINS_Delay_Cnt=20;//10ms
//		     
//			static float Pos_W_E_History[Num]; //��ʷλ������
//			static float Pos_N_S_History[Num];
//			static		float pos_correct[2]={0,0}; //ˮƽλ�� �ٶ� ���ٶ���ά�ϳ�
//			static		float acc_correct[2]={0,0};
//			static		float vel_correct[2]={0,0};	
//			static uint8_t GPS_Save_Period_Cnt=0;
//			
//			
//		if(GPS.Quality>3|| GPS.satellite_num <9 || GPS.Home_Ready == 0) //GPS�ź�̫�����HOME�㻹û���ã��������ں�
//		{
//			return;
//		}
//			
//		 Earth_Position.Longitude_W_E_Origin = GPS.Longitude_W_E_Position*100;//�ص�������ϵ����������λ��ƫ��,��λΪCM
//		 Earth_Position.Latitude_N_S_Origin =  GPS.Lattitude_N_S_Position*100;//�ص�������ϵ����������λ��ƫ��,��λΪCM

//	
//      GPS_Save_Period_Cnt++;  //5ms
//      if(GPS_Save_Period_Cnt>=1)
//      {
//            for(uint8_t Cnt=Num-1;Cnt>0;Cnt--)//10ms����һ��
//            {
//              Pos_W_E_History[Cnt]=Pos_W_E_History[Cnt-1];
//              Pos_N_S_History[Cnt]=Pos_N_S_History[Cnt-1];
//            }
//              Pos_W_E_History[0]=Earth_Position.Longitude_W_E_Distance ;
//              Pos_N_S_History[0]=Earth_Position.Latitude_N_S_Distance;
//              GPS_Save_Period_Cnt=0;
//      }
//	  
//      //GPS��������ϵ�£���������������λ��ƫ����SINS�������Ĳ��λcm
//      float Earth_East_Distance = Earth_Position.Longitude_W_E_Origin -Pos_W_E_History[GPS_SINS_Delay_Cnt];//����
//      float Earth_North_Distance = Earth_Position.Latitude_N_S_Origin -Pos_N_S_History[GPS_SINS_Delay_Cnt];//����

//			#define W_E 0
//			#define N_S 1
//      acc_correct[W_E] += Earth_East_Distance*K_ACC_XY*CNTLCYCLE;//���ٶȽ�����
//      vel_correct[W_E] += Earth_East_Distance* K_VEL_XY*CNTLCYCLE;//�ٶȽ�����
//      pos_correct[W_E] += Earth_East_Distance* K_POS_XY*CNTLCYCLE;//λ�ý�����

//      acc_correct[N_S] += Earth_North_Distance* K_ACC_XY*CNTLCYCLE;//���ٶȽ�����
//      vel_correct[N_S] += Earth_North_Distance* K_VEL_XY*CNTLCYCLE;//�ٶȽ�����
//      pos_correct[N_S] += Earth_North_Distance* K_POS_XY*CNTLCYCLE;//λ�ý�����
//		
//			float speed;
//			static float W_E_Speed;
//			static float W_E_Distance;
//			static float N_S_Speed;
//			static float N_S_Distance;
//      /*************************************************************///�ں�ͶӰ�ڶ��������ϵļ��ٶ�
//			
//      //ˮƽ�˶����ٶȼ�У��			
//      //�ٶ�������������£����ڸ���λ��
//      speed= (Attitude.acc_pitch_sensor + acc_correct[W_E]) *CNTLCYCLE;
//      //ԭʼλ�ø���
//      W_E_Distance +=(Earth_Position.Longitude_W_E_Speed+0.5f*speed)*CNTLCYCLE;
//      //λ�ý��������
//      Earth_Position.Longitude_W_E_Distance = W_E_Distance + pos_correct[W_E];
//      //ԭʼ�ٶȸ���
//      W_E_Speed+=speed;
//      //�ٶȽ��������
//      Earth_Position.Longitude_W_E_Speed = W_E_Speed +vel_correct[W_E];

//      /*************************************************************///�ں�ͶӰ���ϱ������ϵļ��ٶ�
//			
//			//ˮƽ�˶����ٶȼ�У��
//      //�ٶ�������������£����ڸ���λ�� 
//      speed=(Attitude.acc_roll_sensor + acc_correct[N_S])*CNTLCYCLE;
//      //ԭʼλ�ø���
//      N_S_Distance += (Earth_Position.Latitude_N_S_Speed+0.5f*speed)*CNTLCYCLE;
//      //λ�ý��������
//      Earth_Position.Latitude_N_S_Distance =  N_S_Distance+pos_correct[N_S];
//      //ԭʼ�ٶȸ���
//      N_S_Speed  +=  speed;    
//      //�ٶȽ��������
//      Earth_Position.Latitude_N_S_Speed     =  N_S_Speed+vel_correct[N_S];
//}



/**************************************************************
 * �������ں�
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
//float R_GPS[2]={0.05f,0.008f}; //��������
//float Q_GPS[2]={2,3};          //��������
//float R_Acce_bias[2]={0.0001,0.000005}; //���ٶȹ�������
//double Pre_conv_GPS[2][4]=         //������Э�������
//{
//  0.0001 ,    0.00001,  0.00001    , 0.003,
//  0.0001 ,    0.00001,  0.00001    , 0.003,
//};//��һ��Э����
//double K_GPS[2][2]={0};  //�������
//float Acce_Bias[2];  //���ٶȻ���ֵ

//void   KalmanFilter_Horizontal_GPS(float Position_GPS,float Vel_GPS,float Position_Last,float Vel_Last,
//                                   float *Position,float *Vel,
//                                   float *Acce,float *R,
//                                   float *Q,float dt,uint8_t Axis)
//{
//      float Conv_Z=0;
//      float Z_Delta[2]={0};
//      float Conv_Temp=0;
//      double Temp_conv[4]={0};//����Э����
//      uint8 Label=0;
//      if(Axis=='X') Label=0;
//      else Label=1;
//      //����״̬
//      *Position +=*Vel*dt+((*Acce+Acce_Bias[Label])*dt*dt)/2.0f; //�۲Ȿ�ε�λ������
//      *Vel+=*Acce*dt; //�۲Ȿ�ε��ٶ�����
//      //����Э����
//      Conv_Temp=Pre_conv_GPS[Label][1]+Pre_conv_GPS[Label][3]*dt;
//      Temp_conv[0]=Pre_conv_GPS[Label][0]+Pre_conv_GPS[Label][2]*dt+Conv_Temp*dt+R_GPS[0];
//      Temp_conv[1]=Conv_Temp;
//      Temp_conv[2]=Pre_conv_GPS[Label][2]+Pre_conv_GPS[Label][3]*dt;
//      //Temp_conv[1]=Conv_Temp+R_GPS[0]*0.5*0.00001;
//      //Temp_conv[2]=Temp_conv[1];
//      Temp_conv[3]=Pre_conv_GPS[Label][3]+R_GPS[1];
//      //�ټ��㿨��������
//      Conv_Z=1.0/(Temp_conv[0]+Q_GPS[0]*GPS.Quality)*(Temp_conv[3]+Q_GPS[1]*GPS.Quality)-Temp_conv[1]*Temp_conv[2];
//      //K_GPS[0][0]=( Temp_conv[0]*(Temp_conv[3]+Q_GPS[1])-Temp_conv[1]*Temp_conv[2])*Conv_Z;
//      //K_GPS[0][1]=(-Temp_conv[0]*Temp_conv[1]+Temp_conv[1]*(Temp_conv[0]+Q_GPS[0]))*Conv_Z;
//      //K_GPS[1][0]=( Temp_conv[2]*(Temp_conv[3]+Q_GPS[1])-Temp_conv[2]*Temp_conv[3])*Conv_Z;
//      //K_GPS[1][1]=(-Temp_conv[1]*Temp_conv[2]+Temp_conv[3]*(Temp_conv[0]+Q_GPS[0]))*Conv_Z;
//      //��������
//      K_GPS[0][0]=( Temp_conv[0]*(Temp_conv[3]+Q_GPS[1]*GPS.Quality)-Temp_conv[1]*Temp_conv[2])*Conv_Z;
//      K_GPS[0][1]=(Temp_conv[1]*Q_GPS[0]*GPS.Quality)*Conv_Z;
//      K_GPS[1][0]=(Temp_conv[2]*Q_GPS[1]*GPS.Quality)*Conv_Z;
//      K_GPS[1][1]=(-Temp_conv[1]*Temp_conv[2]+Temp_conv[3]*(Temp_conv[0]+Q_GPS[0]*GPS.Quality))*Conv_Z;
//      //�ں��������
//      //Z_Delta[0]=Position_GPS-*Position;
//      //Z_Delta[1]=Vel_GPS-*Vel;
//      Z_Delta[0]=Position_GPS-Position_Last;
//      Z_Delta[1]=Vel_GPS-Vel_Last;
//      *Position +=K_GPS[0][0]*Z_Delta[0]  //�õ����ε�λ�����
//                 +K_GPS[0][1]*Z_Delta[1];
//      *Vel +=K_GPS[1][0]*Z_Delta[0]  //�õ����ε��ٶ����
//            +K_GPS[1][1]*Z_Delta[1];
//      Acce_Bias[Label]=R_Acce_bias[0]*Z_Delta[0]  //���ٶȻ���
//                       +R_Acce_bias[1]*Z_Delta[1];
//      //����״̬Э�������
//      Pre_conv_GPS[Label][0]=(1-K_GPS[0][0])*Temp_conv[0]-K_GPS[0][1]*Temp_conv[2];
//      Pre_conv_GPS[Label][1]=(1-K_GPS[0][0])*Temp_conv[1]-K_GPS[0][1]*Temp_conv[3];
//      Pre_conv_GPS[Label][2]=(1-K_GPS[1][1])*Temp_conv[2]-K_GPS[1][0]*Temp_conv[0];
//      Pre_conv_GPS[Label][3]=(1-K_GPS[1][1])*Temp_conv[3]-K_GPS[1][0]*Temp_conv[1];
//}



//#define NUM_BUF 20

//void Filter_Horizontal(void)
//{
//		const uint8_t GPS_Pos_Delay_Cnt = 10;
//		const uint8_t GPS_Vel_Delay_Cnt = 10;
//	
//		static float Position_History[2][NUM_BUF];
//		static float Vel_History[2][NUM_BUF];
//		static int8_t GPS_Position_Cnt=0;
//	  static float last_time;
//	  float dt;//�ںϼ��
//	//-------------------------------------------------------------------------------------------		
//		if(GPS.Quality>3|| GPS.satellite_num <9 || GPS.Home_Ready == 0) //GPS�źŲ�����������
//		{
//			return;
//		}
// 	//-------------------------------------------------------------------------------------------		
//		float now_time;
//		now_time = micros(); //��ȡ�����ںϵļ��ʱ��
//		dt = (now_time-last_time)/1000;
//		last_time = now_time;
//	//-------------------------------------------------------------------------------------------		
//		 Earth_Position.Longitude_W_E_Origin = GPS.Longitude_W_E_Position*100;//�ص�������ϵ����������λ��ƫ��,��λΪCM
//		 Earth_Position.Latitude_N_S_Origin =  GPS.Lattitude_N_S_Position*100;//�ص�������ϵ����������λ��ƫ��,��λΪCM


//		//-------------------------------------------------------------------------------------------		
//		#define W_E  0
//		#define N_S  1
//		GPS_Position_Cnt++;
//		if(GPS_Position_Cnt>=2)//ÿ10ms����һ��
//		{
//				uint8_t i;
//				for(i=NUM_BUF-1;i>0;i--)
//				{
//				 Position_History[W_E][i]=Position_History[W_E][i-1];
//				 Position_History[N_S][i]=Position_History[N_S][i-1];

//				 Vel_History[W_E][i]=Vel_History[W_E][i-1];
//				 Vel_History[N_S][i]=Vel_History[N_S][i-1];
//				}
//				 Position_History[W_E][0]=Earth_Position.Longitude_W_E_Distance;
//				 Position_History[N_S][0]=Earth_Position.Latitude_N_S_Distance;

//				 Vel_History[W_E][0]=Earth_Position.Longitude_W_E_Speed;
//				 Vel_History[N_S][0]=Earth_Position.Latitude_N_S_Speed;

//				 GPS_Position_Cnt=0;
//		}
//		
//		
//		
//		//-------------------------------------------------------------------------------------------		
//		if(GPS.Analy_ok==1)
//		{
//			GPS.Analy_ok = 0;

//				KalmanFilter_Horizontal_GPS(Earth_Position.Longitude_W_E_Origin,
//																		GPS.Longitude_W_E_speed,
//																		Position_History[W_E][GPS_Pos_Delay_Cnt],
//																		Vel_History[W_E][GPS_Vel_Delay_Cnt],
//																		&Earth_Position.Longitude_W_E_Distance,
//																		&Earth_Position.Longitude_W_E_Speed,
//																		&Attitude.acc_pitch_sensor,
//																		R_GPS,Q_GPS,0.005f,'X');


//				KalmanFilter_Horizontal_GPS(Earth_Position.Latitude_N_S_Origin,
//																		GPS.Lattitude_N_S_speed,
//																		Position_History[N_S][GPS_Pos_Delay_Cnt],
//																		Vel_History[N_S][GPS_Vel_Delay_Cnt],
//																		&Earth_Position.Latitude_N_S_Distance,
//																		&Earth_Position.Latitude_N_S_Speed,
//																		&Attitude.acc_roll_sensor,
//																		R_GPS,Q_GPS,0.005f,'Y');				
//			//-------------------------------------------------------------------------------------------
//		}
//		else
//		{
//			Earth_Position.Longitude_W_E_Distance +=Earth_Position.Longitude_W_E_Speed*dt
//																		+((Attitude.acc_pitch_sensor)*dt*dt)/2.0f;
//			Earth_Position.Longitude_W_E_Speed+=((Attitude.acc_pitch_sensor))*dt;

//			Earth_Position.Latitude_N_S_Distance +=Earth_Position.Latitude_N_S_Speed*dt
//																		+((Attitude.acc_roll_sensor)*dt*dt)/2.0f;
//			Earth_Position.Latitude_N_S_Speed+=(Attitude.acc_roll_sensor )*dt;					
//		}
//			//-------------------------------------------------------------------------------------------
//		
//		if(ABS(Earth_Position.Longitude_W_E_Distance-Earth_Position.Longitude_W_E_Origin)>10000
//			 ||ABS(Earth_Position.Latitude_N_S_Distance-Earth_Position.Latitude_N_S_Origin)>10000
//				 ||ABS(Earth_Position.Longitude_W_E_Speed-GPS.Longitude_W_E_speed)>10000
//					 ||ABS(Earth_Position.Latitude_N_S_Speed-GPS.Lattitude_N_S_speed)>10000
//				 )
//					Earth_Position.Filter_Defeated_Flag=1;//�ں�ʧ�ܱ�־		
//}




