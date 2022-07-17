/*******************************************************************
 *@title LED system
 *@brief flight light
 *@brief 
 *@time  2016.10.19
 *@editorС��&zin
 *�ɿذ���QQȺ551883670,����759421287@qq.com
 ******************************************************************/
#include "stm32f4xx.h"
#include "LED.h"
#include "ALL_DEFINE.h"


//ledָʾ����Ϣ
 _st_LED LED;
//��ɫ			 
#define RED_H PEout(2) = 1                 //��
#define RED_L  PEout(2) = 0                //��
#define RED_Toggle  PEout(2) ^= 1          //��	
//��ɫ
#define GREE_H PEout(1) = 1                //��
#define GREE_L  PEout(1) = 0               //��
#define GREE_Toggle  PEout(1) ^= 1         //��	
//��ɫ
#define BLUE_H PEout(0) = 1                //��
#define BLUE_L  PEout(0) = 0               //��
#define BLUE_Toggle  PEout(0) ^= 1         //��	
//��ɫ
#define CYAN_H  GPIOE->ODR |= 0x03         //��
#define CYAN_L  GPIOE->ODR &= ~0x03		     //��							 
#define CYAN_Toggle GPIOE->ODR ^= 0x03     //��	
//��ɫ										
#define PINK_H  GPIOE->ODR |= 0x05         //��
#define PINK_L  GPIOE->ODR &= ~0x05		     //��								
#define PINK_Toggle GPIOE->ODR ^= 0x05	   //��	
//��ɫ										
#define YELLOW_H GPIOE->ODR |= 0x06	       //��
#define YELLOW_L GPIOE->ODR &= ~0x06	     //��
#define YELLOW_Toggle GPIOE->ODR ^= 0x06	 //��																	
//��ɫ										
#define WHITE_H  GPIOE->ODR |= 0x07        //��
#define WHITE_L  GPIOE->ODR &= ~0x07		   //��							
#define WHITE_Toggle GPIOE->ODR ^= 0x07	   //��																	
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
	
	LED.FlashTime = 150; //10MS ����һ��

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
	
	if(LED.status != last_LEDstatus)      //״̬�л�
	{
		WHITE_H;
		status = 0;
		last_LEDstatus = LED.status;
	}
	else if(LED.color != last_LEDcorlor)  //��ɫ�л�
	{
		WHITE_H;
		last_LEDcorlor = LED.color;
	}
	
	switch(status)
	{
//------------------------------------------------------------------				
		case 0: //�ж���˸״̬
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
		case 1:	//����	
				WHITE_H; //������������
			break;
//------------------------------------------------------------------				
//------------------------------------------------------------------												
		case 3: //��˸
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
												case RED:  //����
													RED_Toggle;
													break;		
												case YELLOW:  //����
													YELLOW_Toggle;			
													break;	
												case GREE:  //����
													GREE_Toggle;
													break;	
												case BLUE:  //����
													BLUE_Toggle;
													break;	
												case CYAN:  //����
													CYAN_Toggle;
													break;	
												case PINK:  //����
													PINK_Toggle;
													break;	
												case WHITE:  //����
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
		case 4:		//����					
												switch(LED.color)
												{
														case RED:  //����
															RED_L;
															break;		
														case YELLOW:  //����
															YELLOW_L;			
															break;	
														case GREE:  //����
															GREE_L;
															break;	
														case BLUE:  //����
															BLUE_L;
															break;	
														case CYAN:  //����
															CYAN_L;
															break;	
														case PINK:  //����
															PINK_L;
															break;	
														case WHITE:  //����
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



