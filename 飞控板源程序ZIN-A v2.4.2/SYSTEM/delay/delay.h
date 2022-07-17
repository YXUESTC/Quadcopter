#ifndef __DELAY_H
#define __DELAY_H 			   
	  
#include "stm32f4xx.h"

void delay_ms(u16 nms);
void delay_us(u32 nus);
void SysTick_Configuration(void);
	void cycleCounterInit(void);
#endif





























