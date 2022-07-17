#ifndef _AT24C02_H
#define _AT24C02_H

#include "stdint.h"

void Write_AT24C02(uint8_t addr, uint8_t *data, uint8_t len);
void Read_AT24C02(uint8_t addr, uint8_t *data, uint8_t len);
void Reset_AT24C02(void);
void AT24C02_Test(void);
#endif



