#include "delay.h"
#include "ALL_DEFINE.h"


#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)


//void cycleCounterInit(void)
//{
//    RCC_ClocksTypeDef clocks;
//    RCC_GetClocksFreq(&clocks);
//    usTicks = clocks.SYSCLK_Frequency / 1000000;
//}

void  SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t         cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;
	
	SysTick_Config(cnts);
	NVIC_SetPriority (SysTick_IRQn, 0);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

uint32_t GetSysTime_us(void) 
{
	register uint32_t ms;
	u32 value;
	ms = SysTick_count;
	value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return value;
}

void delay_us(uint32_t us)
{
    uint32_t now = GetSysTime_us();
    while (GetSysTime_us() - now < us);
}

void delay_ms(uint16_t ms)
{
    while (ms--)
        delay_us(1000);
}


























