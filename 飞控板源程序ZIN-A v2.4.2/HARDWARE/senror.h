#ifndef __SENROR_H
#define __SENROR_H

#include "stdint.h"
#include "mpu6050.h"
#include "ak8975.h"

typedef struct{
	mpu_6050 Mpu6050;
	MAG_8975 Mag8975;
	
}Sensor;

extern Sensor Sensor_Data;

void Mpu6050_Updata(void);
void Mag8975_Updata(void);
void mpu6050_calibrate(void);
void Mag8975_calibrate(void);
int16_t sensor_cal_round(int32_t sum, int32_t num);

#endif
