#ifndef _ALL_USER_DATA_H_
#define _ALL_USER_DATA_H_

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       long long int64_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       long long uint64_t;

typedef   signed          char s8;
typedef   signed short     int s16;
typedef   signed           int s32;
typedef   signed       long long s64;

    /* exact-width unsigned integer types */
typedef unsigned          char u8;
typedef unsigned short     int u16;
typedef unsigned           int u32;
typedef unsigned       long long u64;
#undef   NULL
#define NULL 0
//****************************************************************
//----------------------------�豸����
typedef struct
{
	 struct  //ң��ԭʼ���ݣ�8ͨ��
	{
			uint16_t roll;
			uint16_t pitch;
			uint16_t thr;
			uint16_t yaw;
			uint16_t AUX1;
			uint16_t AUX2;
			uint16_t AUX3;
			uint16_t AUX4;	
	}Remote;	
	 struct{ //MPU6050ԭʼ����
		int16_t accX;
		int16_t accY;
		int16_t accZ;
		int16_t gyroX;
		int16_t gyroY;
		int16_t gyroZ;
	}MPU6050;
//	 struct{ //AK8975ԭʼ����
//		int16_t magY;
//		int16_t magX;	
//		int16_t magZ;
//	}AK8975;

}_st_Device;

//----------------------------��������
 typedef struct
{	
		 struct{  //�Ƕ����� 
			float roll;
			float pitch;
			float yaw;
		}Angle;
		 struct{  //�߶�����
			float rate;
			float bara_height; 
			float ultra_height;
		}High;		 
}_st_FlightData;
//----------------------------PID����

//----------------------------�ɿؽ��յ����ⲿ������
typedef volatile struct
{
	uint8_t AccOffset :1;  //У׼����
	uint8_t GyroOffset :1;
	uint8_t MagOffset :1;
	uint8_t six_acc_offset; //����У׼
	enum{ 								 //����ģʽ�л�����
			LOCK = 0x00,				//����ģʽ
			NORMOL, 		//����ģʽ		
			HEIGHT,			//����ģʽ
			GPS_POSITION,   //GPS����ģʽ
	}FlightMode; //����ģʽ
}_st_Command;

//----------------------------���
typedef volatile struct
{
	uint8_t empty1; //Ԥ��
	
	uint8_t empty2; //Ԥ��
	uint8_t empty3; //Ԥ��
	uint8_t REMOTE :1;
	uint8_t MPU6050 :1;
	uint8_t AK8975 :1;
	uint8_t AT24C02 :1;
	uint8_t SPL06 :1;	
}_st_DeviceCheck;
	


//****************************************************************
//------------------------------------------------------------
//�豸������
extern __align(4) _st_Device Device;
//��������
extern _st_FlightData FlightData;
//�ɿ�����
extern _st_Command Command;




//--------------------------------------------------------------
#endif

