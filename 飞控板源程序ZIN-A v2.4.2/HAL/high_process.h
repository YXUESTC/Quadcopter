#ifndef __HIGH_PROCESS__H
#define __HIGH_PROCESS__H

















typedef struct{
	float High;//�����ĸ߶�
	float Speed;//�߶�
	float Acc_speed;//���ٶ�
}_st_Height;

extern _st_Height Height;
extern float Altitude_Estimate;
void Strapdown_INS_High(float high);
void Strapdown_INS_High_US100(float high);
void High_US100(float high);
#endif



