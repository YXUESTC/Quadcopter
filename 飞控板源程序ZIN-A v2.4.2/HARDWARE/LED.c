/*******************************************************************
 *@title LED system
 *@brief flight light
 *@brief 
 *@time  2016.10.19
 *@editor–°ƒœ&zin
 *∑…øÿ∞Æ∫√QQ»∫551883670,” œ‰759421287@qq.com
 ******************************************************************/
#include "stm32f4xx.h"
#include "LED.h"
#include "ALL_DEFINE.h"


//led÷∏ æµ∆–≈œ¢
 _st_LED LED;
//∫Ï…´			 
#define RED_H PEout(2) = 1                 //∞µ
#define RED_L  PEout(2) = 0                //¡¡
#define RED_Toggle  PEout(2) ^= 1          //…¡	
//¬Ã…´
#define GREE_H PEout(1) = 1                //∞µ
#define GREE_L  PEout(1) = 0               //¡¡
#define GREE_Toggle  PEout(1) ^= 1         //…¡	
//¿∂…´
#define BLUE_H PEout(0) = 1                //∞µ
#define BLUE_L  PEout(0) = 0               //¡¡
#define BLUE_Toggle  PEout(0) ^= 1         //…¡	
//«‡…´
#define CYAN_H  GPIOE->ODR |= 0x03         //∞µ
#define CYAN_L  GPIOE->ODR &= ~0x03		     //¡¡							 
#define CYAN_Toggle GPIOE->ODR ^= 0x03     //…¡	
//∑€…´										
#define PINK_H  GPIOE->ODR |= 0x05         //∞µ
#define PINK_L  GPIOE->ODR &= ~0x05		     //¡¡								
#define PINK_Toggle GPIOE->ODR ^= 0x05	   //…¡	
//ª∆…´										
#define YELLOW_H GPIOE->ODR |= 0x06	       //∞µ
#define YELLOW_L GPIOE->ODR &= ~0x06	     //¡¡
#define YELLOW_Toggle GPIOE->ODR ^= 0x06	 //…¡																	
//∞◊…´										
#define WHITE_H  GPIOE->ODR |= 0x07        //∞µ
#define WHITE_L  GPIOE->ODR &= ~0x07		   //¡¡							
#define WHITE_Toggle GPIOE->ODR ^= 0x07	   //…¡																	
/*******************************************************
 *  LED Init
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void LEDInit(void)	
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_StructInit(&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2| GPIO_Pin_1| GPIO_Pin_0;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOE, GPIO_Pin_2);		
	GPIO_SetBits(GPIOE, GPIO_Pin_1);		
	GPIO_SetBits(GPIOE, GPIO_Pin_0);			
	
	LED.FlashTime = 150; //10MS ∏¸–¬“ª¥Œ

	LED.status = AlwaysOff;
}
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void LED_display(void) //flash 300MS interval
{
	static uint8_t status = 0;
	static uint8_t last_LEDstatus = AlwaysOff;
	static uint8_t last_LEDcorlor;
	
	if(LED.status != last_LEDstatus)      //◊¥Ã¨«–ªª
	{
		WHITE_H;
		status = 0;
		last_LEDstatus = LED.status;
	}
	else if(LED.color != last_LEDcorlor)  //—’…´«–ªª
	{
		WHITE_H;
		last_LEDcorlor = LED.color;
	}
	
	switch(status)
	{
//------------------------------------------------------------------				
		case 0: //≈–∂œ…¡À∏◊¥Ã¨
										switch(LED.status)
										{
												case AlwaysOff: 
															status = 1;
													break;						
												case Flash:
															status = 3;
													break;	
												case AlwaysOn: 
															status = 4;
													break;
												default:
													status = 3;
													break;
										}
			break;
//------------------------------------------------------------------												
		case 1:	//≥£∞µ	
				WHITE_H; //π≤“ıº´£¨∏ﬂ‘Ú∞µ
			break;
//------------------------------------------------------------------				
//------------------------------------------------------------------												
		case 3: //…¡À∏
										{
													static uint32_t FlashTime = 0;
											
													if(SysTick_count-FlashTime < LED.FlashTime)
													{
																			return;
													}	
													FlashTime = SysTick_count;			
										}
										switch(LED.color)
										{
												case RED:  //∫Ï…¡
													RED_Toggle;
													break;		
												case YELLOW:  //ª∆…¡
													YELLOW_Toggle;			
													break;	
												case GREE:  //¬Ã…¡
													GREE_Toggle;
													break;	
												case BLUE:  //¿∂…¡
													BLUE_Toggle;
													break;	
												case CYAN:  //«‡…¡
													CYAN_Toggle;
													break;	
												case PINK:  //∑€…¡
													PINK_Toggle;
													break;	
												case WHITE:  //∞◊…¡
													WHITE_Toggle;
													break;
												case RANDOM:	
													{
															static uint8_t random; 
															random += TIM2->CNT;
															random++;
															PEout(0) ^= random;
															PEout(1) ^= random>>1;
															PEout(2) ^= random>>2;													
													}	
													break;
												default:
														LED.color = RED;
												break;						
										}	
				break;							
//------------------------------------------------------------------												
		case 4:		//≥£¡¡					
												switch(LED.color)
												{
														case RED:  //∫Ï¡¡
															RED_L;
															break;		
														case YELLOW:  //ª∆¡¡
															YELLOW_L;			
															break;	
														case GREE:  //¬Ã¡¡
															GREE_L;
															break;	
														case BLUE:  //¿∂¡¡
															BLUE_L;
															break;	
														case CYAN:  //«‡¡¡
															CYAN_L;
															break;	
														case PINK:  //∑€¡¡
															PINK_L;
															break;	
														case WHITE:  //∞◊¡¡
															WHITE_L;
															break;	
														default:
																LED.color = RED;
														break;						
												}	
												status = 	1;	
			break;
//------------------------------------------------------------------											
		default:
				status = 0;
			break;		
	}
}

/**************************END OF FILE*********************************/



