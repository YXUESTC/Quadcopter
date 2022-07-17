
#include "TIM.h"
#include "stm32f4xx.h"
#include "ALL_DATA.h"
#include "remote.h"

void TIM1_TIM4_PWM_IN_Init(void) //���岶��//���֧��8ͨ��
{
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOD, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	 
	
//////////////////////////////////////////////////////////////////////////////////////////////
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOE, &GPIO_InitStructure);


  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	


	TIM1->PSC = (168)-1;//���÷�Ƶϵ��  ��Ƶ��ÿ����ʱclkΪ1us
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM1, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
/////////////////////////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
  
	TIM4->PSC = (168/2)-1; //���÷�Ƶϵ��  ��Ƶ��ÿ����ʱclkΪ1us
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM4, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
}
//˫���ز����ж�
void TIM1_CC_IRQHandler(void)	
{
	static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
	
//	Feed_Rc_Dog(1);//RC���Ź����㣬��ʾң���ź�����
	
	if(TIM1->SR & TIM_IT_CC1)  //ͨ��8
	{
		TIM1->SR = ~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM1->SR = ~TIM_FLAG_CC1OF;
		if(GPIOE->IDR & GPIO_Pin_9)
		{
			temp_cnt1 = TIM_GetCapture1(TIM1);
		}
		else
		{
			temp_cnt1_2 = TIM_GetCapture1(TIM1);
			if(temp_cnt1_2>=temp_cnt1)
				Device.Remote.AUX4 = temp_cnt1_2-temp_cnt1;//������-�½��ؼ�Ϊ�ߵ�ƽ��ʱ��
			else
				Device.Remote.AUX4 = 0xffff-temp_cnt1+temp_cnt1_2+1;//����ǿ䶨ʱ�����ڵ�
			if(Device.Remote.AUX4<500 || Device.Remote.AUX4>2500)
			{
				Device.Remote.AUX4 = 1000;
			}		
		}
	}
	if(TIM1->SR & TIM_IT_CC2)  //ͨ��7
	{
		TIM1->SR = ~TIM_IT_CC2;
		TIM1->SR = ~TIM_FLAG_CC2OF;
		if(GPIOE->IDR & GPIO_Pin_11)
		{
			temp_cnt2 = TIM_GetCapture2(TIM1);
		}
		else
		{
			temp_cnt2_2 = TIM_GetCapture2(TIM1);
			if(temp_cnt2_2>=temp_cnt2)
			{
				Device.Remote.AUX3 = temp_cnt2_2-temp_cnt2;
			}
			else
			{
				Device.Remote.AUX3 = 0xffff-temp_cnt2+temp_cnt2_2+1;
			}
			if(Device.Remote.AUX3<500 || Device.Remote.AUX3>2500)
			{
				Device.Remote.AUX3 = 1000;
			}		
		}		
	}
	if(TIM1->SR & TIM_IT_CC3) //ͨ��6 
	{
		TIM1->SR = ~TIM_IT_CC3;
		TIM1->SR = ~TIM_FLAG_CC3OF;
		if(GPIOE->IDR & GPIO_Pin_13)
		{
			temp_cnt3 = TIM_GetCapture3(TIM1);
		}
		else
		{
			temp_cnt3_2 = TIM_GetCapture3(TIM1);
			if(temp_cnt3_2>=temp_cnt3)
				Device.Remote.AUX2 = temp_cnt3_2-temp_cnt3;
			else
				Device.Remote.AUX2 = 0xffff-temp_cnt3+temp_cnt3_2+1;
			if(Device.Remote.AUX2<500 || Device.Remote.AUX2>2500)
			{
				Device.Remote.AUX2 = 1000;
			}		
		}
	}
	if(TIM1->SR & TIM_IT_CC4)  //ͨ��5
	{
		TIM1->SR = ~TIM_IT_CC4;
		TIM1->SR = ~TIM_FLAG_CC4OF;
		if(GPIOE->IDR & GPIO_Pin_14)
		{
			temp_cnt4 = TIM_GetCapture4(TIM1);
		}
		else
		{
			temp_cnt4_2 = TIM_GetCapture4(TIM1);
			if(temp_cnt4_2>=temp_cnt4)
				Device.Remote.AUX1 = temp_cnt4_2-temp_cnt4;
			else
				Device.Remote.AUX1 = 0xffff-temp_cnt4+temp_cnt4_2+1;
			if(Device.Remote.AUX1<500 || Device.Remote.AUX1>2500)
			{
				Device.Remote.AUX1 = 1000;
			}		
		}
	}
}

void TIM4_IRQHandler(void)		
{
	static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
	
	Feed_Rc_Dog();//RC���Ź����㣬��ʾң���ź�����
	
	if(TIM4->SR & TIM_IT_CC1)   //ͨ��1
	{
		TIM4->SR = ~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM4->SR = ~TIM_FLAG_CC1OF;
		if(GPIOD->IDR & GPIO_Pin_12)  //������ڸߵ�ƽ����Ϊ������
		{
			temp_cnt1 = TIM_GetCapture1(TIM4);
		}
		else //����Ϊ�½���
		{
			temp_cnt1_2 = TIM_GetCapture1(TIM4);
			//�ߵ�ƽ��Ⱦ��Ǹ�ͨ����ֵ,��λus������1500us���ͨ����ֵΪ1500
			if(temp_cnt1_2>=temp_cnt1)  //�����һ������������ 0XFFFF
				Device.Remote.roll = temp_cnt1_2-temp_cnt1;
			else //����ߵ�ƽ��Խ������������
				Device.Remote.roll = 0xffff-temp_cnt1+temp_cnt1_2+1;
			if(Device.Remote.roll<500 || Device.Remote.roll>2500)
			{
				Device.Remote.roll = 1500;
			}				
		}
	}
	if(TIM4->SR & TIM_IT_CC2)   //ͨ��2
	{
		TIM4->SR = ~TIM_IT_CC2;
		TIM4->SR = ~TIM_FLAG_CC2OF;
		if(GPIOD->IDR & GPIO_Pin_13)
		{
			temp_cnt2 = TIM_GetCapture2(TIM4);
		}
		else
		{
			temp_cnt2_2 = TIM_GetCapture2(TIM4);
			if(temp_cnt2_2>=temp_cnt2)
				Device.Remote.pitch = temp_cnt2_2-temp_cnt2;
			else
				Device.Remote.pitch = 0xffff-temp_cnt2+temp_cnt2_2+1;
			if(Device.Remote.pitch<500 || Device.Remote.pitch>2500)
			{
				Device.Remote.pitch = 1500;
			}				
		}
	}
	if(TIM4->SR & TIM_IT_CC3)   //ͨ��3
	{
		TIM4->SR = ~TIM_IT_CC3;
		TIM4->SR = ~TIM_FLAG_CC3OF;
		if(GPIOD->IDR & GPIO_Pin_14)
		{
			temp_cnt3 = TIM_GetCapture3(TIM4);
		}
		else
		{
			temp_cnt3_2 = TIM_GetCapture3(TIM4);
			if(temp_cnt3_2>=temp_cnt3)
				Device.Remote.thr = temp_cnt3_2-temp_cnt3;
			else
				Device.Remote.thr = 0xffff-temp_cnt3+temp_cnt3_2+1;
			if(Device.Remote.thr<500 || Device.Remote.thr>2500)
			{
				Device.Remote.thr = 1000;
			}				
		}
	}
	if(TIM4->SR & TIM_IT_CC4)   //ͨ��4
	{
		TIM4->SR = ~TIM_IT_CC4;
		TIM4->SR = ~TIM_FLAG_CC4OF;
		if(GPIOD->IDR & GPIO_Pin_15)
		{
			temp_cnt4 = TIM_GetCapture4(TIM4);
		}
		else
		{
			temp_cnt4_2 = TIM_GetCapture4(TIM4);
			if(temp_cnt4_2>=temp_cnt4)
				Device.Remote.yaw = temp_cnt4_2-temp_cnt4;
			else
				Device.Remote.yaw = 0xffff-temp_cnt4+temp_cnt4_2+1;
			if(Device.Remote.yaw<500 || Device.Remote.yaw>2500)
			{
				Device.Remote.yaw = 1500;
			}					
		}
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

