#ifndef __MPU6050_H
#define __MPU6050_H

#include "stdint.h"

#define	MPU_SMPLRT_DIV		0x19	//陀螺仪采样率,典型值:0x07(125Hz)
#define	MPU_CONFIG			0x1A	//低通滤波频率,典型值:0x06(5Hz)
#define	MPU_GYRO_CONFIG		0x1B	//陀螺仪自检以及测量范围,250
#define	MPU_ACCEL_CONFIG	0x1C	//加速度计自检、测量范围以及高通滤波频率,典型值:0x01(不自检,2G,5Hz)
#define	MPU_ACCEL_XOUT_H	0x3B	
#define	MPU_ACCEL_XOUT_L	0x3C
#define	MPU_ACCEL_YOUT_H	0x3D
#define	MPU_ACCEL_YOUT_L	0x3E
#define	MPU_ACCEL_ZOUT_H	0x3F
#define	MPU_ACCEL_ZOUT_L	0x40
#define	MPU_TEMP_OUT_H		0x41
#define	MPU_TEMP_OUT_L		0x42
#define	MPU_GYRO_XOUT_H		0x43
#define	MPU_GYRO_XOUT_L		0x44
#define	MPU_GYRO_YOUT_H		0x45
#define	MPU_GYRO_YOUT_L		0x46
#define	MPU_GYRO_ZOUT_H		0x47
#define	MPU_GYRO_ZOUT_L		0x48
#define MPU_INT_PIN_CFG		0x37    //设置旁路有效 打开值:0x42 AUX_DA的辅助I2C
#define MPU_USER_CTRL		0x6A    //用户配置寄存器 打开值:0x40  AUX_DA的辅助I2C
#define	MPU_PWR_MGMT_1		0x6B	//电源管理,典型值:0x00(正常启用)
#define	MPU_WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68,只读)
#define	MPU_SLAVEADRESS		0xD0	//8位格式地址
//#define	MPU_SLAVEADRESS		0x68	//7位格式地址

typedef struct
{
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyr_x;
	int16_t gyr_y;
	int16_t gyr_z;
	int16_t temp;
}mpu_6050;

uint8_t MPU6050_Init(void);
void mpu_read(mpu_6050 *pdata);
int16_t mpu_get_acc_x(void);	//获取加速度计的值
int16_t mpu_get_acc_y(void);
int16_t mpu_get_acc_z(void);
int16_t mpu_get_gyr_x(void);	//获取陀螺仪的值
int16_t mpu_get_gyr_y(void);
int16_t mpu_get_gyr_z(void);
int16_t mpu_get_temp(void);		//获取温度

#endif











