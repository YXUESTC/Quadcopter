#include "ZIN-module.h"
#include "I2C.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1




uint8_t ZIN_34_Barometer_Altitue(uint32_t *pHigh) //ZIN-34 ��ѹ���θ߶�ģ��
{	//��ȡ��ǰ�߶�����  ZIN-34  SPL06-001ģ��

	#define ZIN_34_ADDRESS 0xC0 //ģ���ַ ���λ��д��־λ 0��д  1����
	#define HEIGHT_READ 0X33 //�߶����ݼĴ�����ַ
	uint8_t data[3]; 
	
	I2C_FastMode =  0;
	if(IIC_read_Bytes(ZIN_34_ADDRESS,HEIGHT_READ,3,data) == FAILED)
	{
		return FAILED;//�����֧�ֶ��ߣ�������ѹ���д�����ֱ�ӷ��ء�
	}			
	*pHigh = ((u32)data[0]<<16) | ((u16)data[1]<<8) | data[2];   //���θ߶�
	return SUCCESS;	
}	




uint8_t ZIN_32_Barometer_Altitue(uint32_t *pHigh)//ZIN-32 ��ѹ���θ߶�ģ��
{



	return SUCCESS;
}




















