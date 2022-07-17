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

//三阶巴特沃斯滤波
float GYRO_LPF(float curr_inputer,   
               Butter_BufferData *Buffer,
               Butter_Parameter *Parameter)
{
	/* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
	Buffer->Input_Butter[2]=curr_inputer;
	/* Butterworth滤波 */
	Buffer->Output_Butter[2]=
	Parameter->b[0] * Buffer->Input_Butter[2]
	+Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
	-Parameter->a[1] * Buffer->Output_Butter[1]
	-Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) 序列保存 */
	Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
	Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
	Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
	Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
	return (Buffer->Output_Butter[2]);
}

void GYRO_IMU_Filter(short gx,short gy,short gz)//角速度低通滤波后用于姿态解算
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
        /* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
        static int LPB_Cnt=0;
        Buffer->Input_Butter[2]=curr_input;
        if(LPB_Cnt>=100)
        {
	/* Butterworth滤波 */
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
	/* x(n) 序列保存 */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
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
	const float K[3]={1.0,1.0,1.0};//默认标度误差
	const float B[3]={0,0,0};//默认零位误差

//------------用于矫正加速度量,给磁力计和加速度计的校准----------------------------------------------
	Attitude.accX_correct=(int16_t)(LPButterworth(ax, &Butter_Buffer_Correct[0], &Butter_1HZ_Parameter_Acce));
	Attitude.accY_correct=(int16_t)(LPButterworth(ay, &Butter_Buffer_Correct[1], &Butter_1HZ_Parameter_Acce));											
	Attitude.accZ_correct=(int16_t)(LPButterworth(az, &Butter_Buffer_Correct[2], &Butter_1HZ_Parameter_Acce));
														
   //Acce_Correct_Update_Flag=1;
//---------------------------//经过椭球校正后的三轴加速度量--------------------------------------------
	Attitude.accX_origion=K[0]*ax-B[0]*One_G_TO_Accel;
	Attitude.accY_origion=K[1]*ay-B[1]*One_G_TO_Accel;
	Attitude.accZ_origion=K[2]*az-B[2]*One_G_TO_Accel;
//---------------------------用于观测--------------------------------------------
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
//        Attitude.acc_yaw *=100;//加速度cm/s^2
//        Attitude.acc_pitch*=AcceGravity/AcceMax;
//        Attitude.acc_pitch*=100;//加速度cm/s^2
//        Attitude.acc_roll*=AcceGravity/AcceMax;
//        Attitude.acc_roll*=100;//加速度cm/s^2


//        Acce_Control_Filter();//加速度滤波，用于GPS惯导、加速度控制反馈量
				
//--------------------------为姿态解算IMU的加速度做准备-------------------------------------------------

//200_30z
#ifndef Butterworth
	
	const static float b_acc[3] ={0.1311064399166,   0.2622128798333,   0.1311064399166};
	const static float a_acc[3] ={1,  -0.7477891782585,    0.272214937925};
	/* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
	accButter[0].input[2] =(int16_t)(Attitude.accX_origion);
    accButter[1].input[2] =(int16_t)(Attitude.accY_origion);
    accButter[2].input[2] =(int16_t)(Attitude.accZ_origion);
	/* Butterworth滤波 */
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
		/* x(n) 序列保存 */
		accButter[axis].input[0] = accButter[axis].input[1];
		accButter[axis].input[1] = accButter[axis].input[2];
		/* y(n) 序列保存 */
		accButter[axis].output[0] = accButter[axis].output[1];
		accButter[axis].output[1] = accButter[axis].output[2];
	}
    Attitude.accX_IMU=accelFilter[0];
    Attitude.accY_IMU=accelFilter[1];
    Attitude.accZ_IMU=accelFilter[2];
	
#else	
	static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};	
	kalman_1(&ekf[0],Attitude.accX_origion);  //一维卡尔曼
	kalman_1(&ekf[1],Attitude.accY_origion);  //一维卡尔曼		
	kalman_1(&ekf[2],Attitude.accZ_origion);  //一维卡尔曼	

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

/**********************惯导加速度LPF_21hz**************************/ 
	//用于控制的加速度
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
//	//偏航角上的加速度分量
//   Attitude.acc_yaw_GPS= 
//                      -IMU.Sin_Roll* Acce_Control[0]
//                        + IMU.Sin_Pitch *IMU.Cos_Roll * Acce_Control[1]
//													+ IMU.Cos_Pitch * IMU.Cos_Roll * Acce_Control[2];
//   //俯仰角上的加速度分量                         
//   Attitude.acc_pitch_GPS=
//                      IMU.Cos_Yaw* IMU.Cos_Roll * Acce_Control[0]
//                        +(IMU.Sin_Pitch*IMU.Sin_Roll*IMU.Cos_Yaw-IMU.Cos_Pitch * IMU.Sin_Yaw) *Acce_Control[1]
//                          +(IMU.Sin_Pitch * IMU.Sin_Yaw+IMU.Cos_Pitch * IMU.Sin_Roll * IMU.Cos_Yaw) * Acce_Control[2];
//   //横滚叫上的加速度分量
//   Attitude.acc_roll_GPS=
//                      IMU.Sin_Yaw* IMU.Cos_Roll * Acce_Control[0]
//                        +(IMU.Sin_Pitch * IMU.Sin_Roll * IMU.Sin_Yaw +IMU.Cos_Pitch * IMU.Cos_Yaw) * Acce_Control[1]
//                          + (IMU.Cos_Pitch * IMU.Sin_Roll * IMU.Sin_Yaw - IMU.Sin_Pitch * IMU.Cos_Yaw) * Acce_Control[2];
//   Attitude.acc_yaw_GPS*=AcceGravity/AcceMax;
//   Attitude.acc_yaw_GPS-=AcceGravity;
//   Attitude.acc_yaw_GPS*=100;//加速度cm/s^2
//   Attitude.acc_pitch_GPS*=AcceGravity/AcceMax;
//   Attitude.acc_pitch_GPS*=100;//加速度cm/s^2
//   Attitude.acc_roll_GPS*=AcceGravity/AcceMax;
//   Attitude.acc_roll_GPS*=100;//加速度cm/s^2

/**********************加速度反馈量LPF_2hz***************************/
	float Acce_Control_Feedback[3] = {0};
	
	Acce_Control_Feedback[0] = LPButterworth(Attitude.accX_origion, &Butter_Buffer_Feedback[0], &Butter_5HZ_Parameter_Acce);
	Acce_Control_Feedback[1] = LPButterworth(Attitude.accY_origion, &Butter_Buffer_Feedback[1], &Butter_5HZ_Parameter_Acce);
	Acce_Control_Feedback[2] = LPButterworth(Attitude.accZ_origion, &Butter_Buffer_Feedback[2], &Butter_5HZ_Parameter_Acce);
	
	//导航坐标系下加速度
	Attitude.acc_high_feedback =  -IMU.Sin_Roll* Acce_Control_Feedback[0] + IMU.Sin_Pitch *IMU.Cos_Roll * Acce_Control_Feedback[1] + IMU.Cos_Pitch * IMU.Cos_Roll * Acce_Control_Feedback[2];//垂直方向上的加速度用于高度控制PID
	Attitude.acc_pitch_feedback = IMU.Cos_Yaw* IMU.Cos_Roll * Acce_Control_Feedback[0] +(IMU.Sin_Pitch*IMU.Sin_Roll*IMU.Cos_Yaw-IMU.Cos_Pitch * IMU.Sin_Yaw) *Acce_Control_Feedback[1]
	                              +(IMU.Sin_Pitch * IMU.Sin_Yaw+IMU.Cos_Pitch * IMU.Sin_Roll * IMU.Cos_Yaw) * Acce_Control_Feedback[2];
	Attitude.acc_roll_feedback = IMU.Sin_Yaw* IMU.Cos_Roll * Acce_Control_Feedback[0] +(IMU.Sin_Pitch * IMU.Sin_Roll * IMU.Sin_Yaw +IMU.Cos_Pitch * IMU.Cos_Yaw) * Acce_Control_Feedback[1]
	                             + (IMU.Cos_Pitch * IMU.Sin_Roll * IMU.Sin_Yaw - IMU.Sin_Pitch * IMU.Cos_Yaw) * Acce_Control_Feedback[2];
	
	//用于导航的运动加速度转化为cm/s^2
	Attitude.acc_high_feedback*=AcceGravity/AcceMax;
	Attitude.acc_high_feedback-=AcceGravity;
	Attitude.acc_high_feedback*=100;//加速度cm/s^2
	
	Attitude.acc_pitch_feedback*=AcceGravity/AcceMax;
	Attitude.acc_pitch_feedback*=100;//加速度cm/s^2
	
	Attitude.acc_roll_feedback*=AcceGravity/AcceMax;
	Attitude.acc_roll_feedback*=100;//加速度cm/s^2
}

void  SINS_Prepare(void)
{
	Acce_Control_Filter();
      /*Z-Y-X欧拉角转方向余弦矩阵
        //Pitch Roll  Yaw 分别对应Φ θ Ψ

             X轴旋转矩阵
             R（Φ）
        {1      0        0    }
        {0      cosΦ    sinΦ}
        {0    -sinΦ    cosΦ }

             Y轴旋转矩阵
             R（θ）
        {cosθ     0        -sinθ}
        {0         1        0     }
        {sinθ     0        cosθ}

             Z轴旋转矩阵
             R（θ）
        {cosΨ      sinΨ       0}
        {-sinΨ     cosΨ       0}
        {0          0           1 }

        由Z-Y-X顺规有:
      载体坐标系到导航坐标系下旋转矩阵R(b2n)
      R(b2n) =R(Ψ)^T*R(θ)^T*R(Φ)^T

      R=
        {cosΨ*cosθ     -cosΦ*sinΨ+sinΦ*sinθ*cosΨ        sinΨ*sinΦ+cosΦ*sinθ*cosΨ}
        {cosθ*sinΨ     cosΦ*cosΨ +sinΦ*sinθ*sinΨ       -cosΨ*sinΦ+cosΦ*sinθ*sinΨ}
        {-sinθ          cosθsin Φ                          cosθcosΦ                   }
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
      Attitude.acc_yaw_sensor-=AcceGravity;//减去重力加速度
      Attitude.acc_yaw_sensor*=100;//加速度cm/s^2
			 


      Attitude.acc_pitch_sensor*=AcceGravity/AcceMax;
      Attitude.acc_pitch_sensor*=100;//加速度cm/s^2

      Attitude.acc_roll_sensor*=AcceGravity/AcceMax;
      Attitude.acc_roll_sensor*=100;//加速度cm/s^2


      Attitude.Acceleration_Length=sqrt(Attitude.acc_yaw_sensor*Attitude.acc_yaw_sensor
                                   +Attitude.acc_pitch_sensor*Attitude.acc_pitch_sensor
                                   +Attitude.acc_roll_sensor*Attitude.acc_roll_sensor);

   /******************************************************************************/
   //将无人机在导航坐标系下的沿着正东、正北方向的运动加速度旋转到当前航向的运动加速度:机头(俯仰)+横滚

      Attitude.acc_x_earth=Attitude.acc_pitch_sensor;//沿地理坐标系，正东方向运动加速度,单位为CM
      Attitude.acc_y_earth=Attitude.acc_roll_sensor;//沿地理坐标系，正北方向运动加速度,单位为CM


      Attitude.acc_x_body=Attitude.acc_x_earth*IMU.Cos_Yaw+Attitude.acc_y_earth*IMU.Sin_Yaw;  //横滚正向运动加速度  X轴正向
      Attitude.acc_y_body=-Attitude.acc_x_earth*IMU.Sin_Yaw+Attitude.acc_y_earth*IMU.Cos_Yaw; //机头正向运动加速度  Y轴正向

}








