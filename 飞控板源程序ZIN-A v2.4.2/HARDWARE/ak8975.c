/*******************************************************************
 *MPU6050
 *@brief 
 *@brief 
 *@time  2016.1.8
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 *非授权使用人员，禁止使用。禁止传阅，违者一经发现，侵权处理。
 ******************************************************************/
#include "AK8975.h"
#include <string.h>
#include "myMath.h"
#include "I2C.h"
#include "usart.h"
#include "delay.h"
#include "kalman.h"
#include "ALL_DATA.h"
#include "AT24C02.h"
#include "imu.h"
#include "attitude_process.h"
#include <stdio.h>

uint8_t AK8975_Init(void)
{
	uint8_t temp;
	
	temp = AK8975_Read(MAG_ADDR, MAG_WIA);
	
	if(temp == 0x48)
	{
		AK8975_Write(MAG_ADDR, MAG_CNTL, 0x11);
		delay_ms(100);
		
		return 0;
	}
	return 1;
}

void mag_read(MAG_8975 *pdata)
{
	uint8_t buff[6];
	
	buff[0] = AK8975_Read(MAG_ADDR, MAG_XOUT_L);
	buff[1] = AK8975_Read(MAG_ADDR, MAG_XOUT_H);
	buff[2] = AK8975_Read(MAG_ADDR, MAG_YOUT_L);
	buff[3] = AK8975_Read(MAG_ADDR, MAG_YOUT_H);
	buff[4] = AK8975_Read(MAG_ADDR, MAG_ZOUT_L);
	buff[5] = AK8975_Read(MAG_ADDR, MAG_ZOUT_H);
	
	pdata->MAG_x = (int16_t)(buff[1] << 8 | buff[0]);
	pdata->MAG_y = (int16_t)(buff[3] << 8 | buff[2]);
	pdata->MAG_z = (int16_t)(buff[5] << 8 | buff[4]);
}

void AK8975_Write(uint8_t IIC_Addr, uint8_t REG_Addr, uint8_t val)
{
	I2C_WriteOneByte(I2C1, IIC_Addr, REG_Addr, val);
}

uint8_t AK8975_Read(uint8_t IIC_Addr, uint8_t REG_Addr)
{
	uint8_t read_data;
	
	read_data = I2C_ReadOneByte(I2C1, IIC_Addr, REG_Addr);
	
	return read_data;
}






/**********************END OF FILE *******************************************************************/





