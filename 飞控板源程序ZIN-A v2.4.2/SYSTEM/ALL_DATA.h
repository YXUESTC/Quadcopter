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
//----------------------------设备数据
typedef struct
{
	 struct  //遥控原始数据，8通道
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
	 struct{ //MPU6050原始数据
		int16_t accX;
		int16_t accY;
		int16_t accZ;
		int16_t gyroX;
		int16_t gyroY;
		int16_t gyroZ;
	}MPU6050;
//	 struct{ //AK8975原始数据
//		int16_t magY;
//		int16_t magX;	
//		int16_t magZ;
//	}AK8975;

}_st_Device;

//----------------------------飞行数据
 typedef struct
{	
		 struct{  //角度数据 
			float roll;
			float pitch;
			float yaw;
		}Angle;
		 struct{  //高度数据
			float rate;
			float bara_height; 
			float ultra_height;
		}High;		 
}_st_FlightData;
//----------------------------PID数据

//----------------------------飞控接收到的外部的命令
typedef volatile struct
{
	uint8_t AccOffset :1;  //校准命令
	uint8_t GyroOffset :1;
	uint8_t MagOffset :1;
	uint8_t six_acc_offset; //六面校准
	enum{ 								 //飞行模式切换命令
			LOCK = 0x00,				//锁定模式
			NORMOL, 		//基本模式		
			HEIGHT,			//定高模式
			GPS_POSITION,   //GPS定点模式
	}FlightMode; //飞行模式
}_st_Command;

//----------------------------检查
typedef volatile struct
{
	uint8_t empty1; //预留
	
	uint8_t empty2; //预留
	uint8_t empty3; //预留
	uint8_t REMOTE :1;
	uint8_t MPU6050 :1;
	uint8_t AK8975 :1;
	uint8_t AT24C02 :1;
	uint8_t SPL06 :1;	
}_st_DeviceCheck;
	


//****************************************************************
//------------------------------------------------------------
//设备的数据
extern __align(4) _st_Device Device;
//飞行数据
extern _st_FlightData FlightData;
//飞控命令
extern _st_Command Command;




//--------------------------------------------------------------
#endif

