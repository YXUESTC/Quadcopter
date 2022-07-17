#ifndef _FILTER_H_
#define _FILTER_H_


#include "stdint.h"

typedef struct
{
	float rslt;
	float k;
}lpf_t;

struct _1_ekf_filter
{
	float LastP;
	float Now_P;
	float out;
	float Kg;
	float Q;
	float R;	
};

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
 const float a[3];
 const float b[3];
}Butter_Parameter;

void fliter_lowpass_init(lpf_t *p, float val, float k);
float fliter_lowpass(lpf_t *p, float data);

double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
void kalman_1(struct _1_ekf_filter *ekf, float input);  //Ò»Î¬¿¨¶ûÂü

uint16_t limit_date(uint16_t dat, uint16_t min, uint16_t max);

#endif





