#ifndef _OSC_H_
#define _OSC_H_
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_gpio.h"
extern uint16_t  OutData[4]; 
extern int dataduty[256];
extern int16_t Sensor[6];
//volatile int  OutData[4];  //ʾ����
//--------����ʾ����1���ݷ�װ����------//
extern unsigned char databuf[10];
//void SciFrameTx(long  Data) ;
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
//--------����ʾ����2���ݷ�װ����------//

extern void OutPut_Data(void);
extern void Ano_Send(void);
#endif



