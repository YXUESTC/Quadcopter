/**************************************************************
 * 
 * @brief
   ZIN-7�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "imu.h"
#include "myMath.h"
#include <math.h>
#include "mpu6050.h"
#include "attitude_process.h"

_st_IMU IMU;

Quaternion NumQ = {1.0f, 0.0f, 0.0f, 0.0f};

static const uint16_t Quad_Delay=10;//5
const uint8_t Quad_Num = Quad_Delay;
	
static float Quad_Buf[Quad_Num+1][4]={0}; //��ʱȡ��10���Ժ����Ԫ��

void AHRSUpdate_GraDes_Delay_Corretion(float gx, float gy, float gz, float ax, float ay, float az)
{
	const float CNTLCYCLE=0.00525f;
	float recipNorm;					// ƽ����
	float s0, s1, s2, s3;					// �ݶ��½��������������̬
	float qDot1, qDot2, qDot3, qDot4;			// ��Ԫ��΢�ַ�����õ���̬
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float delta;
	
	for(uint8_t i=Quad_Num;i>0;i--)//����Ԫ����ʷֵ��������
	{
		Quad_Buf[i][0]=Quad_Buf[i-1][0];
		Quad_Buf[i][1]=Quad_Buf[i-1][1];
		Quad_Buf[i][2]=Quad_Buf[i-1][2];
		Quad_Buf[i][3]=Quad_Buf[i-1][3];
	}
	Quad_Buf[0][0]=NumQ.q0;
	Quad_Buf[0][1]=NumQ.q1;
	Quad_Buf[0][2]=NumQ.q2;
	Quad_Buf[0][3]=NumQ.q3;
//          Gyro_History[0]=Pitch_Gyro;
//          Gyro_History[1]=Roll_Gyro;
//          Gyro_History[2]=Yaw_Gyro;
          /**************���ٶ�������ת���ɽǶ��ƣ���λ:��/��(deg/s)*************/
//          gx*=Gyro_G;
//          gz*=Gyro_G;
//          /************���ٶȸ�ֵ��������̬�����ڻ�,���ٶȷ���*************/
//          Yaw_Gyro=gz;
//          Pitch_Gyro=gx;
//          Roll_Gyro=gy;

//          Gyro_Delta[0]=Pitch_Gyro-Gyro_History[0];
//          Gyro_Delta[1]=Roll_Gyro-Gyro_History[1];
//          Gyro_Delta[2]=Yaw_Gyro-Gyro_History[2];
//          //�Ǽ��ٶ�ģ��
//          Gyro_Delta_Length=sqrt(Gyro_Delta[0]*Gyro_Delta[0]
//                                 +Gyro_Delta[1]*Gyro_Delta[1]
//                                         +Gyro_Delta[2]*Gyro_Delta[2]);
//          //���ٶ�ģ��
//          Gyro_Length=sqrt(Yaw_Gyro*Yaw_Gyro
//                                 +Pitch_Gyro*Pitch_Gyro
//                                         +Roll_Gyro*Roll_Gyro);


	/* ת��Ϊ�����ƣ�������̬����*/
	float tempx,tempy,tempz;
	tempx = gx * Gyro_Gr;
	tempy = gy * Gyro_Gr;
	tempz = gz * Gyro_Gr;
	/* ��Ԫ��΢�ַ��̼��㱾�δ�������Ԫ�� */
	qDot1 = 0.5f * (-Quad_Buf[Quad_Delay][1] * tempx - Quad_Buf[Quad_Delay][2] * tempy - Quad_Buf[Quad_Delay][3] * tempz);
	qDot2 = 0.5f * (Quad_Buf[Quad_Delay][0] * tempx + Quad_Buf[Quad_Delay][2] * tempz - Quad_Buf[Quad_Delay][3] * tempy);
	qDot3 = 0.5f * (Quad_Buf[Quad_Delay][0] * tempy - Quad_Buf[Quad_Delay][1] * tempz + Quad_Buf[Quad_Delay][3] * tempx);
	qDot4 = 0.5f * (Quad_Buf[Quad_Delay][0] * tempz + Quad_Buf[Quad_Delay][1] * tempy - Quad_Buf[Quad_Delay][2] * tempx);
	/* ���ٶȼ������Чʱ,���ü��ٶȼƲ��������� */
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
        recipNorm=Q_rsqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		/* �����ظ����� */
		_2q0 = 2.0f * NumQ.q0;
		_2q1 = 2.0f * NumQ.q1;
		_2q2 = 2.0f * NumQ.q2;
		_2q3 = 2.0f * NumQ.q3;
		_4q0 = 4.0f * NumQ.q0;
		_4q1 = 4.0f * NumQ.q1;
		_4q2 = 4.0f * NumQ.q2;
		_8q1 = 8.0f * NumQ.q1;
		_8q2 = 8.0f * NumQ.q2;
		q0q0 = NumQ.q0 * NumQ.q0;
		q1q1 = NumQ.q1 * NumQ.q1;
		q2q2 = NumQ.q2 * NumQ.q2;
		q3q3 = NumQ.q3 * NumQ.q3;

		/* �ݶ��½��㷨,�����������ݶ� */
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * NumQ.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * NumQ.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * NumQ.q3 - _2q1 * ax + 4.0f * q2q2 * NumQ.q3 - _2q2 * ay;

		/* �ݶȹ�һ�� */
		recipNorm=Q_rsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		
		const float Beta_Adjust[5]={0.03,0.03,0.025,0.02,0.01};//{0.04,0.03,0.025,0.02,0.01};//���ݼ��ٶ�ģ���ı仯����
		float BETADEF=0.04f;
		float Tmep_Acce_Length;
		Tmep_Acce_Length=LIMIT(Attitude.Acceleration_Length,0,1000);//������ͣ��500���� //�޷�
		BETADEF=Beta_Adjust[0]-0.01f*Tmep_Acce_Length/1000.0f;//��̬����
		qDot1 -= BETADEF * s0;
		qDot2 -= BETADEF * s1;
		qDot3 -= BETADEF * s2;
		qDot4 -= BETADEF * s3;
		
	}
    /* ��������Ԫ��΢�ַ����������̬��� */
	/* ����Ԫ����̬��������,�õ���ǰ��Ԫ����̬ */
	/* ���ױϿ����΢�ַ��� */
	delta = (CNTLCYCLE * tempx) * (CNTLCYCLE * tempx) + (CNTLCYCLE * tempy) * (CNTLCYCLE * tempy) + (CNTLCYCLE * tempz) * (CNTLCYCLE * tempz);
	NumQ.q0 = (1.0f - delta / 8.0f) * NumQ.q0 + qDot1 * CNTLCYCLE;
	NumQ.q1 = (1.0f - delta / 8.0f) * NumQ.q1 + qDot2 * CNTLCYCLE;
	NumQ.q2 = (1.0f - delta / 8.0f) * NumQ.q2 + qDot3 * CNTLCYCLE;
	NumQ.q3 = (1.0f - delta / 8.0f) * NumQ.q3 + qDot4 * CNTLCYCLE;
	/* ��λ����Ԫ�� */
	recipNorm=Q_rsqrt(NumQ.q0 * NumQ.q0 + NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3);
	NumQ.q0 *= recipNorm;
	NumQ.q1 *= recipNorm;
	NumQ.q2 *= recipNorm;
	NumQ.q3 *= recipNorm;
	/* ��Ԫ����ŷ����ת��,ת��˳��ΪZ-Y-X,�μ�<Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors>.pdfһ��,P24 */
	IMU.pitch= atan2(2.0f * NumQ.q2 * NumQ.q3 + 2.0f * NumQ.q0 * NumQ.q1, -2.0f * NumQ.q1 * NumQ.q1 - 2.0f * NumQ.q2* NumQ.q2 + 1.0f) * RtA;// Pitch
	IMU.roll= asin(2.0f * NumQ.q0* NumQ.q2-2.0f * NumQ.q1 * NumQ.q3) * RtA;									// Roll
	//NumQ.angle[YAW] = atan2(2.0f * NumQ.q[1] * NumQ.q[2] + 2.0f * NumQ.q[0] * NumQ.q[3], -2.0f * NumQ.q[3] * NumQ.q[3] - 2.0f * NumQ.q[2] * NumQ.q[2] + 1.0f) * RAD2DEG;// Yaw
  
          /*ƫ����һ�׻���*/       
	float yaw_G = gz * Gyro_G;
	if((yaw_G > 2.0f) || (yaw_G < -2.0f)) //����̫С������Ϊ�Ǹ��ţ�����ƫ������
	{
		IMU.yaw+=gz * Gyro_G*0.005f;
	}
}

void Yaw_Lock(short magX,short magY,short magZ) 
{
//	static unsigned char flag = 0;

//	if(flag == 1)
//	return;

	/************��������ǲ���*****************/
	float tempx, tempy;
	tempx = magX * IMU.Cos_Roll + magZ * IMU.Sin_Roll;
	tempy = magX * IMU.Sin_Pitch * IMU.Sin_Roll + magY * IMU.Cos_Pitch - magZ * IMU.Cos_Roll * IMU.Sin_Pitch;
	/***********�����еõ������ƹ۲�Ƕ�*********/
	IMU.yaw_mag = -atan2(tempx, tempy) * RtA;
	//	  float NormQuat = Q_rsqrt(squa(magX)+ squa(magY) +squa(magZ)); 
	//		IMU.yaw_mag=atan2(magX/NormQuat,magY/NormQuat)*57.296;		
	if((IMU.yaw_mag>90 && IMU.yaw<-90) || (IMU.yaw_mag<-90 && IMU.yaw>90))
		IMU.yaw = -IMU.yaw * 0.98f + IMU.yaw_mag * 0.02f;
	else 
		IMU.yaw = IMU.yaw * 0.98f + IMU.yaw_mag * 0.02f;

//	static unsigned short cnt = 0;
//	cnt++;
//	if(cnt>1000)
//	{
//		flag = 1;
//	}
}

void IMU_Calculation(void)
{
	IMU.Cos_Pitch = cos(IMU.pitch*AtR);
	IMU.Sin_Pitch = sin(IMU.pitch*AtR);
	IMU.Cos_Roll = cos(IMU.roll*AtR);
	IMU.Sin_Roll = sin(IMU.roll*AtR);
	IMU.Cos_Yaw = cos(IMU.yaw*AtR);
	IMU.Sin_Yaw = sin(IMU.yaw*AtR);
}







/***************************************************END OF FILE***************************************************/




