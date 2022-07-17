#include "ALL_DATA.h"
#include "spl06.h"
#include "i2c.h"
#include "kalman.h"
#include "filter.h"

spl0601_t p_spl0601;
Spl06_High High;

uint8_t SPL06_001_Init(void)
{
	uint8_t temp;
	
	temp = spl06_read(HW_ADR, ID);
	
	if(temp == 0x10)
	{
		spl06_001_get_calib_param();
	
		spl06_001_set(PRESSURE_SENSOR, 128, 32);
		spl06_001_set(TEMPERATURE_SENSOR, 32, 8);
		
		spl06_001_start_continuous(CONTINUOUS_P_AND_T);
		
		return 0;
	}
	
	return 1;
}

void spl06_001_set(uint8_t xSensor, uint8_t SmplRate, uint8_t OverSmpl)
{
	uint8_t REG=0;
	int32_t i32kPkT=0;
	
	switch(SmplRate)
	{
		case 2:
			REG |= (1<<4);
			break;
		 
		case 4:
			REG |= (2<<4);
			break;
		
		case 8:
			REG |= (3<<4);
			break;
		 
		case 16:
			REG |= (4<<4);
			break;
		 
		case 32:
			REG |= (5<<4);
			break;
		 
		case 64:
			REG |= (6<<4);
			break;
		 
		case 128:
			REG |= (7<<4);
			break;
		 
		case 1:
		default: break;
	}
	
	switch(OverSmpl)
	{
		case 2:
			REG |= 1;
			i32kPkT = 1572864;
			break;
		 
		case 4:
			REG |= 2;
			i32kPkT = 3670016;
			break;
		 
		case 8:
			REG |= 3;
			i32kPkT = 7864320;
			break;
		 
		case 16:
			REG |= 4;
			i32kPkT = 253952;
			break;
		 
		case 32:
			REG |= 5;
			i32kPkT = 516096;
			break;
		 
		case 64:
			REG |= 6;
			i32kPkT = 1040384;
			break;
		 
		case 128:
			REG |= 7;
			i32kPkT = 2088960;
			break;
		  
		case 1:
		default:
			i32kPkT = 524288;
			break;	
	}
	
	if(xSensor == 0)
	{
		p_spl0601.i32kP = i32kPkT;
		spl06_write(HW_ADR, PSR_CFG, REG);
		if(OverSmpl > 8)
		{
			REG = spl06_read(HW_ADR, CFG_REG);
			spl06_write(HW_ADR, CFG_REG, REG|0x04);
		}
	}
	if(xSensor == 1)
	{
		p_spl0601.i32kT = i32kPkT;
		spl06_write(HW_ADR, TMP_CFG, REG|0x80);
		if(OverSmpl > 8)
		{
			REG = spl06_read(HW_ADR, CFG_REG);
			spl06_write(HW_ADR, CFG_REG, REG|0x08);
		}
	}
}

void spl06_001_get_calib_param(void)
{
    uint32_t h;
    uint32_t m;
    uint32_t l;
	
    h = spl06_read(HW_ADR, 0x10);
    l = spl06_read(HW_ADR, 0x11);
    p_spl0601.calib_param.c0 = (int16_t)h<<4 | l>>4;
    p_spl0601.calib_param.c0 = (p_spl0601.calib_param.c0&0x0800)?(0xF000|p_spl0601.calib_param.c0):p_spl0601.calib_param.c0;
    h = spl06_read(HW_ADR, 0x11);
    l = spl06_read(HW_ADR, 0x12);
    p_spl0601.calib_param.c1 = (int16_t)(h&0x0F)<<8 | l;
    p_spl0601.calib_param.c1 = (p_spl0601.calib_param.c1&0x0800)?(0xF000|p_spl0601.calib_param.c1):p_spl0601.calib_param.c1;
    h = spl06_read(HW_ADR, 0x13);
    m = spl06_read(HW_ADR, 0x14);
    l = spl06_read(HW_ADR, 0x15);
    p_spl0601.calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    p_spl0601.calib_param.c00 = (p_spl0601.calib_param.c00&0x080000)?(0xFFF00000|p_spl0601.calib_param.c00):p_spl0601.calib_param.c00;
    h = spl06_read(HW_ADR, 0x15);
    m = spl06_read(HW_ADR, 0x16);
    l = spl06_read(HW_ADR, 0x17);
    p_spl0601.calib_param.c10 = (int32_t)h<<16 | (int32_t)m<<8 | l;
    p_spl0601.calib_param.c10 = (p_spl0601.calib_param.c10&0x080000)?(0xFFF00000|p_spl0601.calib_param.c10):p_spl0601.calib_param.c10;
    h = spl06_read(HW_ADR, 0x18);
    l = spl06_read(HW_ADR, 0x19);
    p_spl0601.calib_param.c01 = (int16_t)h<<8 | l;
		p_spl0601.calib_param.c01 = (p_spl0601.calib_param.c01&0x08000)?(0xF0000|p_spl0601.calib_param.c01):p_spl0601.calib_param.c01;
    h = spl06_read(HW_ADR, 0x1A);
    l = spl06_read(HW_ADR, 0x1B);
    p_spl0601.calib_param.c11 = (int16_t)h<<8 | l;
		p_spl0601.calib_param.c11 = (p_spl0601.calib_param.c11&0x08000)?(0xF0000|p_spl0601.calib_param.c11):p_spl0601.calib_param.c11;
    h = spl06_read(HW_ADR, 0x1C);
    l = spl06_read(HW_ADR, 0x1D);
    p_spl0601.calib_param.c20 = (int16_t)h<<8 | l;
		p_spl0601.calib_param.c20 = (p_spl0601.calib_param.c20&0x08000)?(0xF0000|p_spl0601.calib_param.c20):p_spl0601.calib_param.c20;
    h = spl06_read(HW_ADR, 0x1E);
    l = spl06_read(HW_ADR, 0x1F);
    p_spl0601.calib_param.c21 = (int16_t)h<<8 | l;
		p_spl0601.calib_param.c21 = (p_spl0601.calib_param.c21&0x08000)?(0xF0000|p_spl0601.calib_param.c21):p_spl0601.calib_param.c21;
    h = spl06_read(HW_ADR, 0x20);
    l = spl06_read(HW_ADR, 0x21);
    p_spl0601.calib_param.c30 = (int16_t)h<<8 | l;
		p_spl0601.calib_param.c30 = (p_spl0601.calib_param.c30&0x08000)?(0xF0000|p_spl0601.calib_param.c30):p_spl0601.calib_param.c30;
}

void spl06_001_start_continuous(uint8_t mode)
{
    spl06_write(HW_ADR, MEAS_CFG, mode+4);
}

void spl06_001_get_raw_pressure(void)
{
	uint8_t m[3];
	
	m[2] = spl06_read(HW_ADR, PSR_B2);
	m[1] = spl06_read(HW_ADR, PSR_B1);
	m[0] = spl06_read(HW_ADR, PSR_B0);
	
	p_spl0601.i32rawPressure = (int32_t)(m[2]<<16 | m[1]<<8 | m[0]);
	p_spl0601.i32rawPressure = (p_spl0601.i32rawPressure&0x800000) ? (0xFF000000|p_spl0601.i32rawPressure) : p_spl0601.i32rawPressure;
}

void spl06_001_get_raw_temp(void)
{
	uint8_t m[3];
	
	m[2] = spl06_read(HW_ADR, TMP_B2);
	m[1] = spl06_read(HW_ADR, TMP_B2);
	m[0] = spl06_read(HW_ADR, TMP_B2);
	
	p_spl0601.i32rawTemperature = (int32_t)(m[2]<<16 | m[1]<<8 | m[0]);
	p_spl0601.i32rawTemperature = (p_spl0601.i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601.i32rawTemperature) : p_spl0601.i32rawTemperature;
}

void spl06_001_get_pressure(void)
{
	float fTsc, fPsc;
	
	fTsc = p_spl0601.i32rawTemperature / (float)p_spl0601.i32kT;
	fPsc = p_spl0601.i32rawPressure / (float)p_spl0601.i32kP;
	
	p_spl0601.Pressure = p_spl0601.calib_param.c00 + fPsc * (p_spl0601.calib_param.c10 + fPsc * (p_spl0601.calib_param.c20 + fPsc * p_spl0601.calib_param.c30))
	                    + fTsc * p_spl0601.calib_param.c01 + fPsc * fTsc * (p_spl0601.calib_param.c11 + fPsc * p_spl0601.calib_param.c21);
}

void spl06_001_get_temperature(void)
{
	float fTsc;
	
	fTsc = p_spl0601.i32rawTemperature / (float)p_spl0601.i32kT;
	p_spl0601.Temperature = p_spl0601.calib_param.c0 * 0.5 + p_spl0601.calib_param.c1 * fTsc;
}

void spl06_001_get_data(void)
{
	spl06_001_get_raw_pressure();
	spl06_001_get_raw_temp();
	//spl06_001_get_pressure();
	//spl06_001_get_temperature();
}
float presure_offset;
void spl06_001_HeightHighProcess(void)  
{
	static uint16_t delay_cnt1;
	float presure;
	static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};
	
	spl06_001_get_pressure();
	
	presure = p_spl0601.Pressure;
//	if(delay_cnt1<500)//刚刚上电时 对气压计地面补偿
//		{
//				presure_offset = presure;
//				delay_cnt1++;
//				return;
//		}
	
	High.bara_height =  (float)((102000.0f	- presure) * 7.8740f); 
	
	kalman_1(&ekf[2],High.bara_height);  //一维卡尔曼
	
	High.bara_height = 	ekf[2].out;		//解算后的高度值滤波	
}	

void spl06_write(uint8_t IIC_Addr, uint8_t REG_Addr, uint8_t val)
{
	I2C_WriteOneByte(I2C1, IIC_Addr, REG_Addr, val);
}

uint8_t spl06_read(uint8_t IIC_Addr, uint8_t REG_Addr)
{
	uint8_t read_data;
	
	read_data = I2C_ReadOneByte(I2C1, IIC_Addr, REG_Addr);
	
	return read_data;
}






