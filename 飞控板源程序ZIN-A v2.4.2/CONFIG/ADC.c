#include "stm32f4xx.h"
#include "ADC.h"

uint16_t ADC_Converted_Value[2];

/********************************************************************* 
*                           ��ʼADC 
**********************************************************************/  
  
void ADC3_Init(void)  
{  
    GPIO_InitTypeDef GPIO_InitStructure;  
    DMA_InitTypeDef DMA_InitStructure;    
    ADC_InitTypeDef ADC_InitStructure;    
    ADC_CommonInitTypeDef ADC_CommonInitStructure;  
  
    //������IO������     
    //��ʼ��ʱ��  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
    //�ܽ�ģʽ:�����  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;      
    //������������  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      
    //�ܽ�ָ��  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  
    //��ʼ��  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
  
    //DMA����  
    //����DMAʱ��    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
    //DMAͨ������  
    DMA_DeInit(DMA2_Stream0);  
    DMA_InitStructure.DMA_Channel = DMA_Channel_2;   
    //�����ַ  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC3->DR);  
    //�ڴ��ַ  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC_Converted_Value;  
    //dma���䷽��  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  
    //����DMA�ڴ���ʱ�������ĳ���  
    DMA_InitStructure.DMA_BufferSize = 2;  
    //����DMA���������ģʽ��һ������  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    //����DMA���ڴ����ģʽ  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    //���������ֳ�  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  
    //�ڴ������ֳ�  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  
    //����DMA�Ĵ���ģʽ  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  
    //����DMA�����ȼ���  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
      
    //ָ�����FIFOģʽ��ֱ��ģʽ������ָ������:��ʹ��FIFOģʽ    
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
    //ָ����FIFO��ֵˮƽ  
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
    //ָ����Burstת�������ڴ洫��   
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
    //ָ����Burstת��������Χת�� */    
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
      
    //����DMA��ͨ��           
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);    
    //ʹ��ͨ��  
    DMA_Cmd(DMA2_Stream0, ENABLE);        
  
    //ADC����    
    //����ADCʱ��    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);  
    //��ʼ��ADC  
    ADC_DeInit();  
    
    //ADCͨ������  
    //�ر�DMA��ȡ  
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;    
    //��������ģʽ  
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;  
    //��Ƶϵ��:8,10.5M  
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;  
    //ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;  
    //���β������  
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;  
    //ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;  
    ADC_CommonInit(&ADC_CommonInitStructure);     
  
    //����Ϊ12λ  
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;  
    //ɨ�跽ʽ    
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;     
    //ADC_InitStructure.ADC_ScanConvMode = DISABLE;  
    //����ת��    
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;    
    //�ⲿ������ֹ    
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;    
    //�����Ҷ���    
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;    
    //����ת����ͨ����    
    ADC_InitStructure.ADC_NbrOfConversion = 1;    
    ADC_Init(ADC3, &ADC_InitStructure);    
        
    //����ģʽͨ������        
    ADC_RegularChannelConfig(ADC3, ADC_Channel_9, 1, ADC_SampleTime_144Cycles);  
    //ADC_RegularChannelConfig(ADC3, ADC_Channel_9, 1, ADC_SampleTime_15Cycles);  
  
    //ʹ��ADC��DMA    
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);  
    ADC_DMACmd(ADC3, ENABLE);   
      
    //ʹ��ADC1   
    ADC_Cmd(ADC3, ENABLE);    
        
    //����ADC1�����ת��  
    ADC_SoftwareStartConv(ADC3);       
}





