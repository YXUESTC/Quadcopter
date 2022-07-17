#ifndef __IMU_H
#define	__IMU_H


#include "stdint.h"

typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

typedef struct {
	float yaw;
	float pitch;
	float roll;
	float yaw_mag; //单独由磁力计的出来的角度
	float Cos_Roll;
	float Sin_Roll;
	float Cos_Pitch;
	float Sin_Pitch;
	float Cos_Yaw;
	float Sin_Yaw;
}_st_IMU;

extern _st_IMU IMU;

void IMU_Calculation(void);
void AHRSUpdate_GraDes_Delay_Corretion(float gx, float gy, float gz, float ax, float ay, float az);
void Yaw_Lock(short magX,short magY,short magZ);

#endif






