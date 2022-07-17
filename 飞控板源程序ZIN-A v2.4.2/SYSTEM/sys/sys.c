/**************************************************************
 * @autor ZIN
 * @brief
   ZIN-A�׼�
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/
#include "stm32f4xx.h"  
#include "sys.h"  
#include "ALL_DEFINE.h"
#include "LED.h"

/**************************************************************
 * �δ�ʱ��ms�ж�
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void SysTick_Handler(void)
{
		SysTick_count++; //ms����
		LED_display();  	//LED��˸	
}
/**************************************************************
 *����ϵͳ��ǰ������ʱ��  ��λms
 * @param[in] 
 * @param[out] 
 * @return  ms   
 ***************************************************************/	
float micros(void) //����ϵͳ��ǰʱ��
{
	 //SysTick_count �δ�ʱ��ms����
	 //SysTick->VAL �δ�ʱ�Ӷ�ʱ��������counter �ܼ�����168000.0f/8.0fΪ1��ms���������δ�ʱ��ms�ж�
	 //168MHZ/1000 = 168000ÿ��ms��Ҫ�ľ���Ƶ������
	 //�δ�ʱ�Ӱ˷�Ƶ
    return SysTick_count + (SysTick->LOAD - SysTick->VAL)/(168000.0f/8.0f);//��ǰϵͳʱ�� ��λms  
}












































