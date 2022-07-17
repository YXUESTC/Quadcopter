#include "stm32f4xx.h"
#include "I2C.h"

uint32_t ulTimeOut_Time = 0;

void IICx_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructe;
	I2C_InitTypeDef I2C_InitStructe;
	RCC_ClocksTypeDef RCC_ClocksStructe;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	GPIO_InitStructe.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructe.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructe.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructe.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructe.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructe);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
	
	I2C_InitStructe.I2C_ClockSpeed = 400000;
	I2C_InitStructe.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructe.I2C_DutyCycle = I2C_DutyCycle_16_9;
	I2C_InitStructe.I2C_OwnAddress1 = 0x00;
	I2C_InitStructe.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructe.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStructe);
	
	I2C_Cmd(I2C1, ENABLE);
	
	RCC_GetClocksFreq(&RCC_ClocksStructe);
	ulTimeOut_Time = RCC_ClocksStructe.SYSCLK_Frequency / 10000;
}

uint8_t I2C_Err;
uint8_t I2C_ReadOneByte(I2C_TypeDef* I2Cx, uint8_t I2C_Addr, uint8_t Reg_Addr)
{
	uint8_t read_data;
	uint32_t tim;
	
	tim = ulTimeOut_Time;
	while((--tim) && I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	if(tim == 0) I2C_Err = 1;
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_Send7bitAddress(I2Cx, (I2C_Addr), I2C_Direction_Transmitter);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_SendData(I2Cx, Reg_Addr);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_Send7bitAddress(I2Cx, (I2C_Addr), I2C_Direction_Receiver);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)));
	if(tim == 0) I2C_Err = 1;
	
	read_data = I2C_ReceiveData(I2Cx);
	
	return read_data;
}

void I2C_WriteOneByte(I2C_TypeDef* I2Cx, uint8_t I2C_Addr, uint8_t Reg_Addr, uint8_t val)
{
	uint32_t tim;
	
	tim = ulTimeOut_Time;
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	while((--tim) && I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	if(tim == 0) I2C_Err = 1;
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_SendData(I2Cx, Reg_Addr);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_SendData(I2Cx, val);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_ReadBuffer(I2C_TypeDef* I2Cx, uint8_t I2C_Addr, uint8_t Reg_Addr, uint8_t len, uint8_t *buf)
{
	uint32_t tim;
	
	tim = ulTimeOut_Time;
	while((--tim) && I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	if(tim == 0) I2C_Err = 1;
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_SendData(I2Cx, Reg_Addr);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_GenerateSTART(I2Cx, ENABLE);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)));
	if(tim == 0) I2C_Err = 1;
	
	I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Receiver);
	tim = ulTimeOut_Time;
	while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
	if(tim == 0) I2C_Err = 1;
	
	while(len)
	{
		if(len == 1)
		{
			I2C_AcknowledgeConfig(I2Cx, DISABLE);
			tim = ulTimeOut_Time;
			while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)));
			if(tim == 0) I2C_Err = 1;
			
			*buf = I2C_ReceiveData(I2Cx);
		}
		else
		{
			I2C_AcknowledgeConfig(I2Cx, ENABLE);
			tim = ulTimeOut_Time;
			while((--tim) && (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)));
			if(tim == 0) I2C_Err = 1;
			
			*buf = I2C_ReceiveData(I2Cx);
		}
		
		len--;
		buf++;
	}
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
}


 
 
 
 
 
 
 
 
