#ifndef _SPL06_001_H
#define _SPL06_001_H
#include "ALL_DEFINE.h"

#define PSR_B2 0x00  //压力值最高位 24位
#define PSR_B1 0x01  //中
#define PSR_B0 0x02  //低

#define TMP_B2 0x03  //温度值最高位 24位
#define TMP_B1 0x04  //中
#define TMP_B0 0x05  //低

#define PSR_CFG 0x06 //压力配置寄存器  4~6位压力测量速率 0~3位压力过采样率
#define TMP_CFG 0x07 //温度配置寄存器  7位温度传感器选择 4~6位温度测量速率 0~2位温度过采样率
#define MEAS_CFG 0x08//传感器操作模式及状态寄存器 7位：0-系数不可用 1-系数可用  6位：0-未完成初始化 1-完成     0~2位设置测量模式和类型
#define CFG_REG 0x09 //中断和FIFO配置寄存器

#define INT_STS 0x0A //中断状态寄存器
#define FIFO_STS 0x0B//FIFO状态寄存器

#define RESET_CFG 0x0C   //复位寄存器
#define ID 0x0D      //产品与版本ID寄存器

#define COEF_SRCE 0X28//系数来源寄存器 7位：0-内部

//#define HW_ADR (0x77) //SDO HIGH OR NC
//#define HW_ADR (0x76) //SDO LOW
//#define HW_ADR 0xEC
#define HW_ADR 0xEE

#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3

#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1

typedef struct{  //高度数据
			float rate;
			float bara_height; 
			float ultra_height;
}Spl06_High;	

struct spl0601_calib_param_t {	
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
};

typedef struct{	
    struct spl0601_calib_param_t calib_param;
    uint8_t chip_id;	
    int32_t i32rawPressure;
    int32_t i32rawTemperature;
    int32_t i32kP;    
    int32_t i32kT;
	float Pressure;
	float Temperature;
}spl0601_t;

extern Spl06_High High;
extern spl0601_t p_spl0601;

uint8_t SPL06_001_Init(void);
void spl06_write(uint8_t IIC_Addr, uint8_t REG_Addr, uint8_t val);
uint8_t spl06_read(uint8_t IIC_Addr, uint8_t REG_Addr);
void spl06_001_set(uint8_t xSensor, uint8_t SmplRate, uint8_t OverSmpl);
void spl06_001_get_calib_param(void);
void spl06_001_start_continuous(uint8_t mode);
void spl06_001_get_raw_pressure(void);
void spl06_001_get_raw_temp(void);

void spl06_001_get_pressure(void);
void spl06_001_get_temperature(void);

void spl06_001_get_data(void);
void spl06_001_HeightHighProcess(void);

#endif





