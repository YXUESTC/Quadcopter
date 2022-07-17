#ifndef __LED_H__
#define __LED_H__




//----------------------------LED
typedef struct
{
	short FlashTime; //LED更新时间
//	unsigned char Flash_num;//闪烁次数
	enum
	{
		AlwaysOn,   //常亮 
		AlwaysOff,  //常暗
		Flash,  //闪烁一次
	}status; 
	enum      //颜色选择
	{
		RED, 
		GREE,
		BLUE,
		CYAN,
		PINK,
		YELLOW,
		WHITE,
		RANDOM
	}color; 
}_st_LED;

extern _st_LED LED;

void LEDInit(void);	
void LED_display(void); //flash 300MS interval


#define SYSTEM_ERROR  LED.FlashTime = 150;\
												LED.status = Flash;\
												LED.color = RED
#define SYSTEM_NORMOL			  LED.FlashTime = 300;\
												LED.status = Flash;\
												LED.color = CYAN
#define SYSTEM_WARNING			  LED.FlashTime = 300;\
												LED.status = Flash;\
												LED.color = YELLOW											
																																
															
#endif 


