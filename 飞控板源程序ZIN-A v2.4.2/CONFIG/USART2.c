#include "stm32f4xx.h"
#include "usart.h"
#include "USART.h"


void USART2_init(u32 bound)  
{  
    //�����жϽṹ��  
    NVIC_InitTypeDef NVIC_InitStructure ;  
    //����IO��ʼ���ṹ��  
    GPIO_InitTypeDef GPIO_InitStructure;  
    //���崮�ڽṹ��    
    USART_InitTypeDef USART_InitStructure;  
    //����DMA�ṹ��  
    DMA_InitTypeDef DMA_InitStructure;  
  
    //�򿪴��ڶ�Ӧ������ʱ��    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  
       
    //���ڷ�DMA����    
    //����DMAʱ��  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  
    //DMA�����ж�����  
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);  
    //DMAͨ������  
    DMA_DeInit(DMA1_Stream6);  
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
    //�����ַ  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
    //�ڴ��ַ  	
    DMA_InitStructure.DMA_Memory0BaseAddr = 0; 
    //dma���䷽��  
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
    //����DMA�ڴ���ʱ�������ĳ���  
    DMA_InitStructure.DMA_BufferSize = 0;  
    //����DMA���������ģʽ��һ������  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    //����DMA���ڴ����ģʽ  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    //���������ֳ�  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    //�ڴ������ֳ�  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
    //����DMA�Ĵ���ģʽ  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
    //����DMA�����ȼ���  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
      
    //ָ�����FIFOģʽ��ֱ��ģʽ������ָ������ �� ��ʹ��FIFOģʽ    
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
    //ָ����FIFO��ֵˮƽ  
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
    //ָ����Burstת�������ڴ洫��   
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
    //ָ����Burstת��������Χת�� */    
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
  
    //����DMA1��ͨ��           
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);    
    //ʹ���ж�  
    DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);  
		
   //��ʼ�����ڲ���    
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
    USART_InitStructure.USART_StopBits = USART_StopBits_1;    
    USART_InitStructure.USART_Parity = USART_Parity_No;    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      
    USART_InitStructure.USART_BaudRate = bound;   
    //��ʼ������   
    USART_Init(USART2,&USART_InitStructure);    
      
    //�ж�����  
    USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);  
    USART_ITConfig(USART2,USART_IT_IDLE,DISABLE);    
  
    //�����ж�    
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);  
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;               //ͨ������Ϊ�����ж�    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //�ж�ռ�ȵȼ�  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //�ж���Ӧ���ȼ�   
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //���ж�    
    NVIC_Init(&NVIC_InitStructure);     
          
    //����DMA��ʽ����  
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  

    //��������    
    USART_Cmd(USART2, ENABLE);      
  
    //����IO��ʱ��        
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);   
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
  
		//����PD5��ΪUSART2��Tx
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
		GPIO_Init(GPIOD, &GPIO_InitStructure); 
		//����PD6��ΪUSART2��Rx
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
		GPIO_Init(GPIOD, &GPIO_InitStructure); 
	     
}  


void USART2_Send(const u8 *data,u8 len)
{
  u8 i;
	for(i=0;i<len;i++)
	{
				while(!(USART2->SR & 0x80));
		//		USART_SendData(USART2, *(data+i));         //�򴮿�1��������
			USART2->DR = *(data+i);
	}
}


//void USART2_IRQHandler(void)                	//����1�жϷ������
//{
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
//	{

//		  		 
//  } 

//} 


void DMA1_Stream6_IRQHandler(void)
{
	if(DMA1->HISR & DMA_IT_TCIF6)
  {
     DMA1->HIFCR = DMA_IT_TCIF6; //����жϱ�־λ

		 DMA1_Stream6->CR  &= ~ DMA_SxCR_EN; //ʧ��DMA
  }
}
