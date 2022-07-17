#include "senror.h"
#include "myMath.h"
#include "delay.h"
#include "LED.h"
#include "filter.h"
#include "attitude_process.h"
#include "Remote.h"
#include "ALL_DATA.h"
#include "AT24C02.h"

#define SENSOR_CAL_NUM 500

mpu_6050 Mpu_Data;
mpu_6050 Mpu_Offset;

MAG_8975 Mag_Data;
MAG_8975 Mag_Offset;

Sensor Sensor_Data;

void Mpu6050_Updata(void)
{
	mpu_read(&Mpu_Data);
	
	Sensor_Data.Mpu6050.acc_x = Mpu_Data.acc_x - Mpu_Offset.acc_x;
	Sensor_Data.Mpu6050.acc_y = Mpu_Data.acc_y - Mpu_Offset.acc_y;
	Sensor_Data.Mpu6050.acc_z = Mpu_Data.acc_z - Mpu_Offset.acc_z;
	ACC_IMU_Filter(Sensor_Data.Mpu6050.acc_x, Sensor_Data.Mpu6050.acc_y, Sensor_Data.Mpu6050.acc_z);
	
	Sensor_Data.Mpu6050.gyr_x = Mpu_Data.gyr_x - Mpu_Offset.gyr_x;
	Sensor_Data.Mpu6050.gyr_y = Mpu_Data.gyr_y - Mpu_Offset.gyr_y;
	Sensor_Data.Mpu6050.gyr_z = Mpu_Data.gyr_z - Mpu_Offset.gyr_z;
	GYRO_IMU_Filter(Sensor_Data.Mpu6050.gyr_x, Sensor_Data.Mpu6050.gyr_y, Sensor_Data.Mpu6050.gyr_z);
}

void Mag8975_Updata(void)
{
	mag_read(&Mag_Data);
	
	Sensor_Data.Mag8975.MAG_x = Mag_Data.MAG_x - Mag_Offset.MAG_x; //这里是否有问题
	Sensor_Data.Mag8975.MAG_y = -Mag_Data.MAG_y - Mag_Offset.MAG_y;
	Sensor_Data.Mag8975.MAG_z = -Mag_Data.MAG_z - Mag_Offset.MAG_z;
	
	static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.6},{0.02,0,0,0,0.001,0.6},{0.02,0,0,0,0.001,0.6}};
	
	kalman_1(&ekf[0], (float)Sensor_Data.Mag8975.MAG_x);
	Sensor_Data.Mag8975.MAG_x = (int16_t)ekf[0].out;
	kalman_1(&ekf[1], (float)Sensor_Data.Mag8975.MAG_y);
	Sensor_Data.Mag8975.MAG_y = (int16_t)ekf[1].out;
	kalman_1(&ekf[2], (float)Sensor_Data.Mag8975.MAG_z);
	Sensor_Data.Mag8975.MAG_z = (int16_t)ekf[2].out;
		
	AK8975_Write(MAG_ADDR, MAG_CNTL, 0x11);
}

void mpu6050_calibrate(void)
{
	uint16_t time;
	
 	int32_t acc_x = 0;
 	int32_t acc_y = 0;
 	int32_t acc_z = 0;
	
	int32_t gyr_x = 0;
	int32_t gyr_y = 0;
	int32_t gyr_z = 0;
	
	for(time=0; time<SENSOR_CAL_NUM; time++)
	{
		mpu_read(&Mpu_Data);
		
		gyr_x += Mpu_Data.gyr_x;
		gyr_y += Mpu_Data.gyr_y;
		gyr_z += Mpu_Data.gyr_z;
		
		acc_x += Mpu_Data.acc_x;
		acc_y += Mpu_Data.acc_y;
		acc_z += Mpu_Data.acc_z;
	}
	
	Mpu_Offset.gyr_x = sensor_cal_round(gyr_x, SENSOR_CAL_NUM);
	Mpu_Offset.gyr_y = sensor_cal_round(gyr_y, SENSOR_CAL_NUM);
	Mpu_Offset.gyr_z = sensor_cal_round(gyr_z, SENSOR_CAL_NUM);
	
	Mpu_Offset.acc_x = sensor_cal_round(acc_x, SENSOR_CAL_NUM);
	Mpu_Offset.acc_y = sensor_cal_round(acc_y, SENSOR_CAL_NUM);
	Mpu_Offset.acc_z = sensor_cal_round(acc_z, SENSOR_CAL_NUM) - 4096;
	
}	

//void Mag8975_calibrate(void)
//{
//	uint16_t cng;
//	
//	int16_t tempx, tempy, tempz;
//	int16_t gx_last,gy_last,gz_last;
//	
//	
//	//检测陀螺仪是否静止下来
//	do{
//		cng = 30;
//		tempx = 0; tempy = 0; tempz = 0;
//		
//		while(cng--)
//		{
//			mpu_read(&Mpu_Data);
//			
//			tempx += absu16(Mpu_Data.gyr_x - gx_last);
//			tempy += absu16(Mpu_Data.gyr_y - gy_last);
//			tempz += absu16(Mpu_Data.gyr_z - gz_last);
//			
//			gx_last = Mpu_Data.gyr_x;
//			gy_last = Mpu_Data.gyr_y;
//			gz_last = Mpu_Data.gyr_z;
//			
//			__nop();__nop();__nop();__nop();__nop();
//		}
//	}while(tempx >= 100 || tempy >= 100 || tempz >= 100); //如果>=100陀螺仪没有静止
//	
//	
//	float Pitch = 0, Roll = 0;
//	int16_t Zmax=-2000, Zmin = 2000;
//	
//	//RGB_Flash(Blue, ON); //RGB_Flash(Green, ON);
//	while(Pitch >-500 && Roll < 500 && Pitch < 500 && Roll > -500)
//	{
//		mag_read(&Mag_Data);
//		delay_ms(10);
//		
//		Mag_Data.MAG_z = -Mag_Data.MAG_z;
//		if(Mag_Data.MAG_z >= Zmax) Zmax = Mag_Data.MAG_z;
//		if(Mag_Data.MAG_z <= Zmin) Zmin = Mag_Data.MAG_z;
//		
//		mpu_read(&Mpu_Data);
//		Pitch += ((float)Mpu_Data.gyr_x) * Gyro_G * 0.01f;
//		Roll  += ((float)Mpu_Data.gyr_y) * Gyro_G * 0.01f;
//	}
//	Mag_Offset.MAG_z = (Zmax + Zmin) *0.5f;
//	//RGB_Flash(Blue, OFF); //RGB_Flash(Green, OFF);
//	
//	float Yaw = 0;
//	int16_t Xmax=-2000, Xmin = 2000, Ymax=-2000, Ymin = 2000;
//	
//	//RGB_Flash(Red, ON); //RGB_Flash(Green, ON);
//	while(Yaw >-500 && Yaw < 500)
//	{
//		mag_read(&Mag_Data);
//		delay_ms(10);
//		
//		if(Mag_Data.MAG_x >= Xmax) Xmax = Mag_Data.MAG_x;
//		if(Mag_Data.MAG_x <= Xmin) Xmin = Mag_Data.MAG_x;
//		
//		if(Mag_Data.MAG_y >= Ymax) Ymax = Mag_Data.MAG_y;
//		if(Mag_Data.MAG_y <= Ymin) Ymin = Mag_Data.MAG_y;
//		
//		mpu_read(&Mpu_Data);
//		Yaw += ((float)Mpu_Data.gyr_z) * Gyro_G * 0.01f;
//	}
//	Mag_Offset.MAG_x = (Xmax + Xmin) * 0.5f;
//	Mag_Offset.MAG_y = (Ymax + Ymin) * 0.5f;
//	//RGB_Flash(Blue, OFF); //RGB_Flash(Green, OFF);
//}

int16_t sensor_cal_round(int32_t sum, int32_t num)	
{
	if(sum >= 0)
	{
		return (int16_t)((sum + num / 2) / num);
	}
	else
	{
		return (int16_t)((sum - num / 2) / num);
	}
}

int16_t x_max ;
int16_t x_min ;
int16_t y_max ;
int16_t y_min ;
int16_t z_max ;
int16_t z_min ;

 float Y_up_angle; //Y轴朝上的角度
 float Z_up_angle; //Z轴朝上的角度
void Mag8975_calibrate(void)	
{
	static uint8_t status = 0;
	
	if(Command.FlightMode != LOCK) // 只有在锁定状态才能校准，解锁后不能校准。
	{
		status = 255;
	}	
	
  switch(status)
	{
		case 0:
					status = 1;	
Command.MagOffset=0;		
			break;
			//-------------------------------------------------
		case 1:
				if(Command.MagOffset)//进入磁力计校准模式
				{
						Mag_Offset.MAG_x= Mag_Offset.MAG_y = Mag_Offset.MAG_z = 0;//校准值清0
						 x_max = -32767;
						 x_min = 32767;
						 y_max = -32767;
						 y_min = 32767;
						 z_max = -32767;
						 z_min = 32767;
						LED.color = PINK	;
						status = 2;	
						Z_up_angle = 0;
				}
			break;
		case 2:		//第一次点校准，进入第一面校准
				LED.color = PINK	;
				Command.MagOffset = 1;
				if(Attitude.accZ_origion>=2500)//Z轴基本竖直
				{
						Z_up_angle += Attitude.gyroZ_IMU*0.005F* Gyro_G;

							if(Sensor_Data.Mag8975.MAG_x >= x_max)   x_max = (int16_t)(Sensor_Data.Mag8975.MAG_x);
							if(Sensor_Data.Mag8975.MAG_x <  x_min)   x_min = (int16_t)(Sensor_Data.Mag8975.MAG_x);
							if(Sensor_Data.Mag8975.MAG_y >= y_max)   y_max = (int16_t)(Sensor_Data.Mag8975.MAG_y);
							if(Sensor_Data.Mag8975.MAG_y <  y_min)   y_min = (int16_t)(Sensor_Data.Mag8975.MAG_y);
							if(Sensor_Data.Mag8975.MAG_z >= z_max)   z_max = (int16_t)(Sensor_Data.Mag8975.MAG_z);
							if(Sensor_Data.Mag8975.MAG_z <  z_min)   z_min = (int16_t)(Sensor_Data.Mag8975.MAG_z);
						
							if(Z_up_angle>450 || Z_up_angle<-450)
						{
								Command.MagOffset = 0; //清除上位机发来的校准标志位  //进入下一面校准
								status = 3;
						}	
				}
			break;
		case 3:				
		 			if(Command.MagOffset)//再点一次磁力计校准，进入第二面校准
					{
							LED.color = PINK	;
							status = 4;	
							Y_up_angle = 0;
					}
			break;
		case 4:
				LED.color = PINK	; //粉色
				LED.FlashTime  = 50; //第二次校准快闪
				Command.MagOffset = 1;//保持正在校准状态
		
				if(Attitude.accY_origion>=2500)//Z轴基本竖直
				{
						Y_up_angle += Attitude.gyroY_IMU*0.005F* Gyro_G;
					
						
							if(Sensor_Data.Mag8975.MAG_x >= x_max)   x_max = (int16_t)(Sensor_Data.Mag8975.MAG_x);
							if(Sensor_Data.Mag8975.MAG_x <  x_min)   x_min = (int16_t)(Sensor_Data.Mag8975.MAG_x);
							if(Sensor_Data.Mag8975.MAG_y >= y_max)   y_max = (int16_t)(Sensor_Data.Mag8975.MAG_y);
							if(Sensor_Data.Mag8975.MAG_y <  y_min)   y_min = (int16_t)(Sensor_Data.Mag8975.MAG_y);
							if(Sensor_Data.Mag8975.MAG_z >= z_max)   z_max = (int16_t)(Sensor_Data.Mag8975.MAG_z);
							if(Sensor_Data.Mag8975.MAG_z <  z_min)   z_min = (int16_t)(Sensor_Data.Mag8975.MAG_z);
						
						if(Y_up_angle>600 || Y_up_angle<-600)
						{
								Command.MagOffset = 0; //清除上位机发来的校准标志位  //进入下一面校准
								LED.FlashTime = 300;//正常闪烁
								status = 254;
						}	
				}											
			break;				
		case 254:
						Mag_Offset.MAG_x =(x_min+x_max)/2.0;
						Mag_Offset.MAG_y =(y_min+y_max)/2.0;
						Mag_Offset.MAG_z=(z_min+z_max)/2.0;	
//						Write_AT24C02(13,(uint8_t)Mag_Offset,6);//保存校准数据
		case 255: //全都校准完成
					status = 0;
			break;		
		default:
			break;	
	}
}

