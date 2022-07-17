#ifndef _SPL06_001_H
#define _SPL06_001_H
#include "ALL_DEFINE.h"

#define PSR_B2 0x00  //ѹ��ֵ���λ 24λ
#define PSR_B1 0x01  //��
#define PSR_B0 0x02  //��

#define TMP_B2 0x03  //�¶�ֵ���λ 24λ
#define TMP_B1 0x04  //��
#define TMP_B0 0x05  //��

#define PSR_CFG 0x06 //ѹ�����üĴ���  4~6λѹ���������� 0~3λѹ����������
#define TMP_CFG 0x07 //�¶����üĴ���  7λ�¶ȴ�����ѡ�� 4~6λ�¶Ȳ������� 0~2λ�¶ȹ�������
#define MEAS_CFG 0x08//����������ģʽ��״̬�Ĵ��� 7λ��0-ϵ�������� 1-ϵ������  6λ��0-δ��ɳ�ʼ�� 1-���     0~2λ���ò���ģʽ������
#define CFG_REG 0x09 //�жϺ�FIFO���üĴ���

#define INT_STS 0x0A //�ж�״̬�Ĵ���
#define FIFO_STS 0x0B//FIFO״̬�Ĵ���

#define RESET_CFG 0x0C   //��λ�Ĵ���
#define ID 0x0D      //��Ʒ��汾ID�Ĵ���

#define COEF_SRCE 0X28//ϵ����Դ�Ĵ��� 7λ��0-�ڲ�

//#define HW_ADR (0x77) //SDO HIGH OR NC
//#define HW_ADR (0x76) //SDO LOW
//#define HW_ADR 0xEC
#define HW_ADR 0xEE

#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3

#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1

typedef struct{  //�߶�����
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





