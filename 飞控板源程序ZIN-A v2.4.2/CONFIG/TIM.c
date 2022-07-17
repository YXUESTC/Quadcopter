#include "stm32f4xx.h"
#include "TIM.h"



void TIM2_config(void)   //APB1  84M
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;		
    NVIC_InitTypeDef NVIC_InitStructure; 
    
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
	
		/* 使能时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

		TIM_DeInit(TIM2);
	
	/* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Period=5000;
	
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
	  /* 时钟预分频数为72 */
    TIM_TimeBaseStructure.TIM_Prescaler= 84 - 1;
	
		/* 对外部时钟进行采样的时钟分频,这里没有用到 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   //向上计数
	
		TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

		TIM_ClearFlag(TIM2,TIM_FLAG_Update);

		TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
		
		
    TIM_Cmd(TIM2, ENABLE);																		
		
}


//void TIM2_IRQHandler(void)  
//{  
//    if (TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)      
//			{     
//        TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);        
//    }  
//}  



