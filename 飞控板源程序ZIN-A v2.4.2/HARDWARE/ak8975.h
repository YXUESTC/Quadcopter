#ifndef _MAGNETIC_H
#define _MAGNETIC_H 


#include "stdint.h"

#define	MAG_ADDR 0x18

#define MAG_WIA 0x00   //�豸ID��0x48
#define MAG_INFO 0x01  //�豸��Ϣ

#define MAG_ST1 0x02 //��0λΪ1 ����׼������

#define MAG_XOUT_L 0x03
#define MAG_XOUT_H 0x04
#define MAG_YOUT_L 0x05
#define MAG_YOUT_H 0x06
#define MAG_ZOUT_L 0x07
#define MAG_ZOUT_H 0x08

#define MAG_ST2 0x09    
#define MAG_CNTL 0x0A   //0x00 ����ģʽ   0x01  ��һ����ģʽ  0x08  �Լ�ģʽ  0x0F   Fuse ROM����ģʽ
#define MAG_RSV 0x0B    //����
#define MAG_ASTC 0x0C   //ֻ��д��6λ

//������������
#define MAG_TS1 0x0D
#define MAG_TS2 0x0E
#define MAG_I2CDIS 0x0F

//�����ȵ���
#define MAG_ASAX 0x10
#define MAG_ASAY 0x11
#define MAG_ASAZ 0x12

typedef struct{
	int16_t MAG_x;
	int16_t MAG_y;
	int16_t MAG_z;
}MAG_8975;

uint8_t AK8975_Init(void);
void mag_read(MAG_8975 *pdata);

void AK8975_Write(uint8_t IIC_Addr, uint8_t REG_Addr, uint8_t val);
uint8_t AK8975_Read(uint8_t IIC_Addr, uint8_t REG_Addr);

#endif







