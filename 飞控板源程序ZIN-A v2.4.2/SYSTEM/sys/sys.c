/**************************************************************
 * @autor ZIN
 * @brief
   ZIN-A套件
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "stm32f4xx.h"  
#include "sys.h"  
#include "ALL_DEFINE.h"
#include "LED.h"

/**************************************************************
 * 滴答时钟ms中断
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void SysTick_Handler(void)
{
		SysTick_count++; //ms计数
		LED_display();  	//LED闪烁	
}
/**************************************************************
 *返回系统当前的运行时间  单位ms
 * @param[in] 
 * @param[out] 
 * @return  ms   
 ***************************************************************/	
float micros(void) //返回系统当前时间
{
	 //SysTick_count 滴答时钟ms计数
	 //SysTick->VAL 滴答时钟定时器计数器counter 总计数满168000.0f/8.0f为1个ms，并发生滴答时钟ms中断
	 //168MHZ/1000 = 168000每个ms需要的晶振频率输了
	 //滴答时钟八分频
    return SysTick_count + (SysTick->LOAD - SysTick->VAL)/(168000.0f/8.0f);//当前系统时间 单位ms  
}












































