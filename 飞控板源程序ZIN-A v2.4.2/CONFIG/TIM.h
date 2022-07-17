#ifndef ____TIM____H_____
#define ____TIM____H_____

#include "stm32f4xx.h"



void TIM2_config(void);
void TIM1_TIM4_PWM_IN_Init(void);
u8 TIM3_TIM5_TIM8_PWM_Out_Init(uint16_t hz,u8 init_duty);//400hz

#endif


















