#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H


#include "ALL_DEFINE.h"


enum ANTO_SEND{
	ANTO_VER      = 0x00,
	ANTO_STATUS    = 0x01,
	ANTO_MPU_MAGIC   = 0X02,
	ANTO_RCDATA   = 0x03,
	ANTO_GPSDATA  = 0x04,
	ANTO_POWER    = 0x05,
	ANTO_MOTOR    = 0x06,
	ANTO_SENSER2  = 0x07,	
	ANTO_RESERD1  = 0x08,
	ANTO_RESERD2  = 0x09,	
	ANTO_FLY_MODE = 0x0A,	
	ANTO_SPEED = 0x0B,	
	ANTO_PID1       = 0x10,	
	ANTO_PID2      = 0x11,
	ANTO_PID3  = 0x12,		
	ANTO_PID4   = 0x13,	
	ANTO_PID5    = 0x14,	
	ANTO_PID6   = 0x15, 	
	ANTO_CHECK	 = 0xEF
};

extern int16_t checkPID; //����PIDУ��
extern void ANO_Recive(int8_t *pt); //�յ���λ��������
extern void ANTO_polling(void);//����״̬��ѯ����
extern void ANTO_Send(const enum ANTO_SEND FUNCTION);//�������ݸ���λ��
#endif







