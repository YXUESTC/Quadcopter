/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "stdlib.h"
#include "ALL_DATA.h"
#include "mpu6050.h"
#include "I2C.h"
#include <string.h>
#include "LED.h"
#include "kalman.h"
#include "delay.h"
#include "AT24C02.h"
#include "math.h"
#include "myMath.h"
#include "attitude_process.h"
uint8_t MPU6050_Init(void)
{
	uint8_t temp;
	
	temp = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_WHO_AM_I);
	
	if(temp == 0x68)
	{
		I2C_WriteOneByte(I2C1, MPU_SLAVEADRESS, MPU_PWR_MGMT_1, 0x00);  //电源正常
		I2C_WriteOneByte(I2C1, MPU_SLAVEADRESS, MPU_INT_PIN_CFG, 0x42); //打开旁路IIC
		I2C_WriteOneByte(I2C1, MPU_SLAVEADRESS, MPU_USER_CTRL, 0x40);   //打开旁路IIC
		I2C_WriteOneByte(I2C1, MPU_SLAVEADRESS, MPU_SMPLRT_DIV, 0x04);  //分频系数
		I2C_WriteOneByte(I2C1, MPU_SLAVEADRESS, MPU_CONFIG, 0x02);      //设置低筒滤波
		I2C_WriteOneByte(I2C1, MPU_SLAVEADRESS, MPU_GYRO_CONFIG, 0x18); //陀螺仪量程：±2000 16.4LSB/°
		I2C_WriteOneByte(I2C1, MPU_SLAVEADRESS, MPU_ACCEL_CONFIG, 0x10);//加速度计量程：±8g，4096LSB/g
		
		return 0;
	}
	
	return 1;
}

void mpu_read(mpu_6050 *pdata)
{
	uint8_t buff[14];
	
	I2C_ReadBuffer(I2C1, MPU_SLAVEADRESS, MPU_ACCEL_XOUT_H, 14, buff);
	
	pdata->acc_x = buff[0] << 8 | buff[1];
 	pdata->acc_y = buff[2] << 8 | buff[3];
	pdata->acc_z = buff[4] << 8 | buff[5];
	
	pdata->temp = buff[6] << 8 | buff[7];
	
	pdata->gyr_x = buff[8] << 8 | buff[9];
	pdata->gyr_y = buff[10] << 8 | buff[11];
	pdata->gyr_z = buff[12] << 8 | buff[13]; 
}

int16_t mpu_get_acc_x(void)
{
	int16_t AccX = 0;
	uint8_t AccXH = 0, AccXL = 0;

	AccXH = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_ACCEL_XOUT_H);
	AccXL = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_ACCEL_XOUT_L);

	AccX = (AccXH << 8) | AccXL;

	return AccX;
}

int16_t mpu_get_acc_y(void)
{
	int16_t AccY = 0;
	uint8_t AccYH = 0, AccYL = 0;

	AccYH = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_ACCEL_YOUT_H);
	AccYL = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_ACCEL_YOUT_L);

	AccY = (AccYH << 8) | AccYL;

	return AccY;
}

int16_t mpu_get_acc_z(void)
{
	int16_t AccZ = 0;
	uint8_t AccZH = 0, AccZL = 0;

	AccZH = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_ACCEL_ZOUT_H);
	AccZL = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_ACCEL_ZOUT_L);

	AccZ = (AccZH << 8) | AccZL;

	return AccZ;
}

int16_t mpu_get_gyr_x(void)
{
	int16_t GyroX = 0;
	uint8_t GyroXH = 0, GyroXL = 0;

	GyroXH = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_GYRO_XOUT_H);
	GyroXL = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_GYRO_XOUT_L);

	GyroX = (GyroXH << 8) | GyroXL;

	return GyroX;
}

int16_t mpu_get_gyr_y(void)
{
	int16_t GyroY = 0;
	uint8_t GyroYH = 0, GyroYL = 0;

	GyroYH = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_GYRO_YOUT_H);
	GyroYL = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_GYRO_YOUT_L);

	GyroY = (GyroYH << 8) | GyroYL;

	return GyroY;
}

int16_t mpu_get_gyr_z(void)
{
	int16_t GyroZ = 0;
	uint8_t GyroZH = 0, GyroZL = 0;

	GyroZH = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_GYRO_ZOUT_H);
	GyroZL = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_GYRO_ZOUT_L);

	GyroZ = (GyroZH << 8) | GyroZL;

	return GyroZ;
}

int16_t mpu_get_temp(void)
{
	int16_t temperature = 0;
	uint8_t temperatureH = 0, temperatureL = 0;

	temperatureH = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_TEMP_OUT_H);
	temperatureL = I2C_ReadOneByte(I2C1, MPU_SLAVEADRESS, MPU_TEMP_OUT_L);

	temperature = (temperatureH << 8) | temperatureL;	//t=36.53 + temperature / 340

	return temperature;
}



/**************************************END OF FILE*************************************/

