#include "math.h"
#include "ALL_DATA.h"
#include "attitude_process.h"
#include "kalman.h"
#include "Filter.h"
#include "IMU.h"
#include "osc.h"
#define GRAVITY_MSS     9.80665f
#define AcceMax_1G      4096.0f
#define One_G_TO_Accel  AcceMax_1G/GRAVITY_MSS

_st_Attitude Attitude;

//200hz---30hz
Butter_Parameter Gyro_Parameter={
1,  -0.7477891782585,    0.272214937925,
0.1311064399166,   0.2622128798333,   0.1311064399166
};

Butter_BufferData Gyro_BufferData[4];

//���װ�����˹�˲�
float GYRO_LPF(float curr_inputer,   
               Butter_BufferData *Buffer,
               Butter_Parameter *Parameter)
{
	/* ���ٶȼ�Butterworth�˲� */
	/* ��ȡ����x(n) */
	Buffer->Input_Butter[2]=curr_inputer;
	/* Butterworth�˲� */
	Buffer->Output_Butter[2]=
	Parameter->b[0] * Buffer->Input_Butter[2]
	+Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
	-Parameter->a[1] * Buffer->Output_Butter[1]
	-Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) ���б��� */
	Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
	Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) ���б��� */
	Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
	Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
	return (Buffer->Output_Butter[2]);
}

void GYRO_IMU_Filter(short gx,short gy,short gz)//���ٶȵ�ͨ�˲���������̬����
{

        Attitude.gyroX_IMU=GYRO_LPF(gx,
                        &Gyro_BufferData[0],
                        &Gyro_Parameter
                        );
        Attitude.gyroY_IMU=GYRO_LPF(gy,
                        &Gyro_BufferData[1],
                        &Gyro_Parameter
                        );
        Attitude.gyroZ_IMU=GYRO_LPF(gz,
                        &Gyro_BufferData[2],
                        &Gyro_Parameter
                        );
}

float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
        /* ���ٶȼ�Butterworth�˲� */
	/* ��ȡ����x(n) */
        static int LPB_Cnt=0;
        Buffer->Input_Butter[2]=curr_input;
        if(LPB_Cnt>=100)
        {
	/* Butterworth�˲� */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
        }
        else
        {
          Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
          LPB_Cnt++;
        }
	/* x(n) ���б��� */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) ���б��� */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];


        return Buffer->Output_Butter[2];
}

Butter_BufferData Butter_Buffer_Correct[3];

//200hz---1hz
Butter_Parameter Butter_1HZ_Parameter_Acce={
  1,   -1.955578240315,   0.9565436765112,
  0.000241359049042, 0.000482718098084, 0.000241359049042
};

struct _ButterWorth2d_Acc_Tag
{
	int16_t input[3];
	int16_t output[3];
};

struct _ButterWorth2d_Acc_Tag accButter[3] =
{
	/* input[3] output[3] */
	{0, 0, 0, 0, 0, 0},		// X-axis
	{0, 0, 0, 0, 0, 0},		// Y-axis
	{0, 0, 0, 0, 0, 0},		// Z-axis
};

void ACC_IMU_Filter(int16_t ax,int16_t ay,int16_t az)
{
	const float K[3]={1.0,1.0,1.0};//Ĭ�ϱ�����
	const float B[3]={0,0,0};//Ĭ����λ���

//------------���ڽ������ٶ���,�������ƺͼ��ٶȼƵ�У׼----------------------------------------------
	Attitude.accX_correct=(int16_t)(LPButterworth(ax, &Butter_Buffer_Correct[0], &Butter_1HZ_Parameter_Acce));
	Attitude.accY_correct=(int16_t)(LPButterworth(ay, &Butter_Buffer_Correct[1], &Butter_1HZ_Parameter_Acce));											
	Attitude.accZ_correct=(int16_t)(LPButterworth(az, &Butter_Buffer_Correct[2], &Butter_1HZ_Parameter_Acce));
														
   //Acce_Correct_Update_Flag=1;
//---------------------------//��������У�����������ٶ���--------------------------------------------
	Attitude.accX_origion=K[0]*ax-B[0]*One_G_TO_Accel;
	Attitude.accY_origion=K[1]*ay-B[1]*One_G_TO_Accel;
	Attitude.accZ_origion=K[2]*az-B[2]*One_G_TO_Accel;
//---------------------------���ڹ۲�--------------------------------------------
//        Attitude.acc_yaw = 
//                      -Sin_Roll* Attitude.accX_origion
//                        + Sin_Pitch *Cos_Roll *Attitude.accY_origion
//                           + Cos_Pitch * Cos_Roll *Attitude.accZ_origion;
//        Attitude.acc_pitch=
//                   Cos_Yaw* Cos_Roll * Attitude.accX_origion
//                        +(Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw) * Attitude.accY_origion
//                          +(Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw) * Attitude.accZ_origion;
//        Attitude.acc_roll=
//                   Sin_Yaw* Cos_Roll * Attitude.accX_origion
//                        +(Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw) * Attitude.accY_origion
//                          + (Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw) * Attitude.accZ_origion;


//        Attitude.acc_yaw *= AcceGravity/AcceMax;
//        Attitude.acc_yaw -=AcceGravity;
//        Attitude.acc_yaw *=100;//���ٶ�cm/s^2
//        Attitude.acc_pitch*=AcceGravity/AcceMax;
//        Attitude.acc_pitch*=100;//���ٶ�cm/s^2
//        Attitude.acc_roll*=AcceGravity/AcceMax;
//        Attitude.acc_roll*=100;//���ٶ�cm/s^2


//        Acce_Control_Filter();//���ٶ��˲�������GPS�ߵ������ٶȿ��Ʒ�����
				
//--------------------------Ϊ��̬����IMU�ļ��ٶ���׼��-------------------------------------------------

//200_30z
#ifndef Butterworth
	
	const static float b_acc[3] ={0.1311064399166,   0.2622128798333,   0.1311064399166};
	const static float a_acc[3] ={1,  -0.7477891782585,    0.272214937925};
	/* ���ٶȼ�Butterworth�˲� */
	/* ��ȡ����x(n) */
	accButter[0].input[2] =(int16_t)(Attitude.accX_origion);
    accButter[1].input[2] =(int16_t)(Attitude.accY_origion);
    accButter[2].input[2] =(int16_t)(Attitude.accZ_origion);
	/* Butterworth�˲� */
	float accelFilter[3];
	for (uint8_t axis = 0; axis < 3; axis++)
	{
		accButter[axis].output[2] =
                 (int16_t)(b_acc[0] * accButter[axis].input[2]
                  + b_acc[1] * accButter[axis].input[1]
                    + b_acc[2] * accButter[axis].input[0]
                      - a_acc[1] * accButter[axis].output[1]
                        - a_acc[2] * accButter[axis].output[0]);
		accelFilter[axis] = accButter[axis].output[2];
	}
	for (uint8_t axis = 0; axis < 3; axis++)
	{
		/* x(n) ���б��� */
		accButter[axis].input[0] = accButter[axis].input[1];
		accButter[axis].input[1] = accButter[axis].input[2];
		/* y(n) ���б��� */
		accButter[axis].output[0] = accButter[axis].output[1];
		accButter[axis].output[1] = accButter[axis].output[2];
	}
    Attitude.accX_IMU=accelFilter[0];
    Attitude.accY_IMU=accelFilter[1];
    Attitude.accZ_IMU=accelFilter[2];
	
#else	
	static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};	
	kalman_1(&ekf[0],Attitude.accX_origion);  //һά������
	kalman_1(&ekf[1],Attitude.accY_origion);  //һά������		
	kalman_1(&ekf[2],Attitude.accZ_origion);  //һά������	

	Attitude.accX_IMU=ekf[0].out;
	Attitude.accY_IMU=ekf[1].out;
	Attitude.accZ_IMU=ekf[2].out;				
	
#endif	
}

Butter_BufferData Butter_Buffer[3];
Butter_BufferData Butter_Buffer_Feedback[3];

Butter_Parameter Butter_30HZ_Parameter_Acce={
  //200hz---30hz
1,  -0.7477891782585,    0.272214937925,
0.1311064399166,   0.2622128798333,   0.1311064399166
};

Butter_Parameter Butter_5HZ_Parameter_Acce={
  //200hz---5hz
  1,   -1.778631777825,   0.8008026466657,
  0.005542717210281,  0.01108543442056, 0.005542717210281
};

#define AcceMax     4096.0f  //   4096
#define AcceGravity 9.80f

void Acce_Control_Filter(void)
{

/**********************�ߵ����ٶ�LPF_21hz**************************/ 
	//���ڿ��Ƶļ��ٶ�
   //Acce_Control[0]=LPButterworth(X_Origion,
   //                 &Butter_Buffer[0],&Butter_15HZ_Parameter_Acce);
   //Acce_Control[1]=LPButterworth(Y_Origion
   //                 ,&Butter_Buffer[1],&Butter_15HZ_Parameter_Acce);
   //Acce_Control[2]=LPButterworth(Z_Origion
   //                 ,&Butter_Buffer[2],&Butter_15HZ_Parameter_Acce);
   Attitude.accX_control=LPButterworth(Attitude.accX_origion,
                    &Butter_Buffer[0],&Butter_30HZ_Parameter_Acce);
   Attitude.accY_control=LPButterworth(Attitude.accY_origion
                    ,&Butter_Buffer[1],&Butter_30HZ_Parameter_Acce);
   Attitude.accZ_control=LPButterworth(Attitude.accZ_origion
                    ,&Butter_Buffer[2],&Butter_30HZ_Parameter_Acce);
//	//ƫ�����ϵļ��ٶȷ���
//   Attitude.acc_yaw_GPS= 
//                      -IMU.Sin_Roll* Acce_Control[0]
//                        + IMU.Sin_Pitch *IMU.Cos_Roll * Acce_Control[1]
//													+ IMU.Cos_Pitch * IMU.Cos_Roll * Acce_Control[2];
//   //�������ϵļ��ٶȷ���                         
//   Attitude.acc_pitch_GPS=
//                      IMU.Cos_Yaw* IMU.Cos_Roll * Acce_Control[0]
//                        +(IMU.Sin_Pitch*IMU.Sin_Roll*IMU.Cos_Yaw-IMU.Cos_Pitch * IMU.Sin_Yaw) *Acce_Control[1]
//                          +(IMU.Sin_Pitch * IMU.Sin_Yaw+IMU.Cos_Pitch * IMU.Sin_Roll * IMU.Cos_Yaw) * Acce_Control[2];
//   //������ϵļ��ٶȷ���
//   Attitude.acc_roll_GPS=
//                      IMU.Sin_Yaw* IMU.Cos_Roll * Acce_Control[0]
//                        +(IMU.Sin_Pitch * IMU.Sin_Roll * IMU.Sin_Yaw +IMU.Cos_Pitch * IMU.Cos_Yaw) * Acce_Control[1]
//                          + (IMU.Cos_Pitch * IMU.Sin_Roll * IMU.Sin_Yaw - IMU.Sin_Pitch * IMU.Cos_Yaw) * Acce_Control[2];
//   Attitude.acc_yaw_GPS*=AcceGravity/AcceMax;
//   Attitude.acc_yaw_GPS-=AcceGravity;
//   Attitude.acc_yaw_GPS*=100;//���ٶ�cm/s^2
//   Attitude.acc_pitch_GPS*=AcceGravity/AcceMax;
//   Attitude.acc_pitch_GPS*=100;//���ٶ�cm/s^2
//   Attitude.acc_roll_GPS*=AcceGravity/AcceMax;
//   Attitude.acc_roll_GPS*=100;//���ٶ�cm/s^2

/**********************���ٶȷ�����LPF_2hz***************************/
	float Acce_Control_Feedback[3] = {0};
	
	Acce_Control_Feedback[0] = LPButterworth(Attitude.accX_origion, &Butter_Buffer_Feedback[0], &Butter_5HZ_Parameter_Acce);
	Acce_Control_Feedback[1] = LPButterworth(Attitude.accY_origion, &Butter_Buffer_Feedback[1], &Butter_5HZ_Parameter_Acce);
	Acce_Control_Feedback[2] = LPButterworth(Attitude.accZ_origion, &Butter_Buffer_Feedback[2], &Butter_5HZ_Parameter_Acce);
	
	//��������ϵ�¼��ٶ�
	Attitude.acc_high_feedback =  -IMU.Sin_Roll* Acce_Control_Feedback[0] + IMU.Sin_Pitch *IMU.Cos_Roll * Acce_Control_Feedback[1] + IMU.Cos_Pitch * IMU.Cos_Roll * Acce_Control_Feedback[2];//��ֱ�����ϵļ��ٶ����ڸ߶ȿ���PID
	Attitude.acc_pitch_feedback = IMU.Cos_Yaw* IMU.Cos_Roll * Acce_Control_Feedback[0] +(IMU.Sin_Pitch*IMU.Sin_Roll*IMU.Cos_Yaw-IMU.Cos_Pitch * IMU.Sin_Yaw) *Acce_Control_Feedback[1]
	                              +(IMU.Sin_Pitch * IMU.Sin_Yaw+IMU.Cos_Pitch * IMU.Sin_Roll * IMU.Cos_Yaw) * Acce_Control_Feedback[2];
	Attitude.acc_roll_feedback = IMU.Sin_Yaw* IMU.Cos_Roll * Acce_Control_Feedback[0] +(IMU.Sin_Pitch * IMU.Sin_Roll * IMU.Sin_Yaw +IMU.Cos_Pitch * IMU.Cos_Yaw) * Acce_Control_Feedback[1]
	                             + (IMU.Cos_Pitch * IMU.Sin_Roll * IMU.Sin_Yaw - IMU.Sin_Pitch * IMU.Cos_Yaw) * Acce_Control_Feedback[2];
	
	//���ڵ������˶����ٶ�ת��Ϊcm/s^2
	Attitude.acc_high_feedback*=AcceGravity/AcceMax;
	Attitude.acc_high_feedback-=AcceGravity;
	Attitude.acc_high_feedback*=100;//���ٶ�cm/s^2
	
	Attitude.acc_pitch_feedback*=AcceGravity/AcceMax;
	Attitude.acc_pitch_feedback*=100;//���ٶ�cm/s^2
	
	Attitude.acc_roll_feedback*=AcceGravity/AcceMax;
	Attitude.acc_roll_feedback*=100;//���ٶ�cm/s^2
}

void  SINS_Prepare(void)
{
	Acce_Control_Filter();
      /*Z-Y-Xŷ����ת�������Ҿ���
        //Pitch Roll  Yaw �ֱ��Ӧ�� �� ��

             X����ת����
             R������
        {1      0        0    }
        {0      cos��    sin��}
        {0    -sin��    cos�� }

             Y����ת����
             R���ȣ�
        {cos��     0        -sin��}
        {0         1        0     }
        {sin��     0        cos��}

             Z����ת����
             R���ȣ�
        {cos��      sin��       0}
        {-sin��     cos��       0}
        {0          0           1 }

        ��Z-Y-X˳����:
      ��������ϵ����������ϵ����ת����R(b2n)
      R(b2n) =R(��)^T*R(��)^T*R(��)^T

      R=
        {cos��*cos��     -cos��*sin��+sin��*sin��*cos��        sin��*sin��+cos��*sin��*cos��}
        {cos��*sin��     cos��*cos�� +sin��*sin��*sin��       -cos��*sin��+cos��*sin��*sin��}
        {-sin��          cos��sin ��                          cos��cos��                   }
      */

      Attitude.acc_yaw_sensor = -IMU.Sin_Roll* Attitude.accX_control
                                + IMU.Sin_Pitch *IMU.Cos_Roll * Attitude.accY_control
                                + IMU.Cos_Pitch * IMU.Cos_Roll *Attitude.accZ_control;

      Attitude.acc_pitch_sensor = IMU.Cos_Yaw*IMU.Cos_Roll * Attitude.accX_control
                                  + (IMU.Sin_Pitch*IMU.Sin_Roll*IMU.Cos_Yaw-IMU.Cos_Pitch * IMU.Sin_Yaw) * Attitude.accY_control
                                  + (IMU.Sin_Pitch * IMU.Sin_Yaw+IMU.Cos_Pitch * IMU.Sin_Roll * IMU.Cos_Yaw)*Attitude.accZ_control;
                                

      Attitude.acc_roll_sensor = IMU.Sin_Yaw* IMU.Cos_Roll * Attitude.accX_control
                                 + (IMU.Sin_Pitch * IMU.Sin_Roll * IMU.Sin_Yaw +IMU.Cos_Pitch * IMU.Cos_Yaw) * Attitude.accY_control
                                 + (IMU.Cos_Pitch * IMU.Sin_Roll * IMU.Sin_Yaw - IMU.Sin_Pitch * IMU.Cos_Yaw)*Attitude.accZ_control;

      /*
      Origion_NamelessQuad.Acceleration[_YAW]=rMat[2][0]*Acce_Control[0]
                                              +rMat[2][1]*Acce_Control[1]
                                                +rMat[2][2]*Acce_Control[2];

      Origion_NamelessQuad.Acceleration[_PITCH]=rMat[0][0]*Acce_Control[0]
                                              +rMat[0][1]*Acce_Control[1]
                                                +rMat[0][2]*Acce_Control[2];

      Origion_NamelessQuad.Acceleration[_ROLL]=rMat[1][0]*Acce_Control[0]
                                              +rMat[1][1]*Acce_Control[1]
                                             +rMat[1][2]*Acce_Control[2]; */



      Attitude.acc_yaw_sensor*=AcceGravity/AcceMax;
      Attitude.acc_yaw_sensor-=AcceGravity;//��ȥ�������ٶ�
      Attitude.acc_yaw_sensor*=100;//���ٶ�cm/s^2
			 


      Attitude.acc_pitch_sensor*=AcceGravity/AcceMax;
      Attitude.acc_pitch_sensor*=100;//���ٶ�cm/s^2

      Attitude.acc_roll_sensor*=AcceGravity/AcceMax;
      Attitude.acc_roll_sensor*=100;//���ٶ�cm/s^2


      Attitude.Acceleration_Length=sqrt(Attitude.acc_yaw_sensor*Attitude.acc_yaw_sensor
                                   +Attitude.acc_pitch_sensor*Attitude.acc_pitch_sensor
                                   +Attitude.acc_roll_sensor*Attitude.acc_roll_sensor);

   /******************************************************************************/
   //�����˻��ڵ�������ϵ�µ���������������������˶����ٶ���ת����ǰ������˶����ٶ�:��ͷ(����)+���

      Attitude.acc_x_earth=Attitude.acc_pitch_sensor;//�ص�������ϵ�����������˶����ٶ�,��λΪCM
      Attitude.acc_y_earth=Attitude.acc_roll_sensor;//�ص�������ϵ�����������˶����ٶ�,��λΪCM


      Attitude.acc_x_body=Attitude.acc_x_earth*IMU.Cos_Yaw+Attitude.acc_y_earth*IMU.Sin_Yaw;  //��������˶����ٶ�  X������
      Attitude.acc_y_body=-Attitude.acc_x_earth*IMU.Sin_Yaw+Attitude.acc_y_earth*IMU.Cos_Yaw; //��ͷ�����˶����ٶ�  Y������

}








