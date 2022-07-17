#include "AT24C02.h"
#include "delay.h"
#include "i2c.h"


#define AT24C02_ADDR 0xA0

#define START_ADDR 0  //256byte
#define  STOP_ADDR 0XFF

void Write_AT24C02(uint8_t addr, uint8_t *data, uint8_t len)
{
	if(addr>0x00 && addr<=0xff) 
	{
		while(len--)
		{
			delay_ms(100);
			I2C_WriteOneByte(I2C1, AT24C02_ADDR, addr++, *data);
			data++;
		}
	}
	else
		return;
}

void Read_AT24C02(uint8_t addr, uint8_t *data, uint8_t len)
{
	if(addr>0x00 && addr<=0xff) 
	{
		while(len--)
		{
			*data = I2C_ReadOneByte(I2C1, AT24C02_ADDR, addr++);
			data++;
		}
	}
	else
		return;
	
}

void Reset_AT24C02(void)
{
	for(int16_t i=0; i<=0xff; i++)
	{
		I2C_WriteOneByte(I2C1, AT24C02_ADDR, i, 0);
	}
}

uint8_t test;

void AT24C02_Test(void)
{
	I2C_WriteOneByte(I2C1, AT24C02_ADDR, 0x55, 0xaa);
	delay_ms(1000);
	test = I2C_ReadOneByte(I2C1, AT24C02_ADDR, 0x55);
}






















