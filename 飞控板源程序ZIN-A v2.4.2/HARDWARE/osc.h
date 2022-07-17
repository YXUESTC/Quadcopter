#ifndef _OSC_H_
#define _OSC_H_
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_gpio.h"
extern uint16_t  OutData[4]; 
extern int dataduty[256];
extern int16_t Sensor[6];
//volatile int  OutData[4];  //示波器
//--------串口示波器1数据封装发送------//
extern unsigned char databuf[10];
//void SciFrameTx(long  Data) ;
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
//--------串口示波器2数据封装发送------//

extern void OutPut_Data(void);
extern void Ano_Send(void);
#endif



