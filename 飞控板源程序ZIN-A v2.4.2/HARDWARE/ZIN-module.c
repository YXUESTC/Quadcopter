#include "ZIN-module.h"
#include "I2C.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1




uint8_t ZIN_34_Barometer_Altitue(uint32_t *pHigh) //ZIN-34 气压海拔高度模组
{	//获取当前高度数据  ZIN-34  SPL06-001模块

	#define ZIN_34_ADDRESS 0xC0 //模块地址 最低位读写标志位 0：写  1：读
	#define HEIGHT_READ 0X33 //高度数据寄存器地址
	uint8_t data[3]; 
	
	I2C_FastMode =  0;
	if(IIC_read_Bytes(ZIN_34_ADDRESS,HEIGHT_READ,3,data) == FAILED)
	{
		return FAILED;//如果不支持定高，或者气压计有错误则直接返回。
	}			
	*pHigh = ((u32)data[0]<<16) | ((u16)data[1]<<8) | data[2];   //海拔高度
	return SUCCESS;	
}	




uint8_t ZIN_32_Barometer_Altitue(uint32_t *pHigh)//ZIN-32 气压海拔高度模组
{



	return SUCCESS;
}




















