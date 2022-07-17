#include "ALL_DEFINE.h"
#include "filter.h"



//低通滤波器初始化
void fliter_lowpass_init(lpf_t *p, float val, float k)
{
	p->rslt = val;
	p->k = k;
}

//低通滤波器
float fliter_lowpass(lpf_t *p, float data)
{
	p->rslt = p->rslt * p->k + data * (1 - p->k);
	
	return p->rslt;
}

double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }

double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double y_last;
   double y_mid = y_last;
   double y_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   y_mid=y_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   y_now=y_mid+kg*(ResrcData-y_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   y_last = y_now; //更新系统状态值
   return y_now;                
 }

double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double z_last;
   double z_mid = z_last;
   double z_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   z_mid=z_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   z_now=z_mid+kg*(ResrcData-z_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   z_last = z_now; //更新系统状态值
   return z_now;                
 }

 uint16_t limit_date(uint16_t dat, uint16_t min, uint16_t max)
 {
	 dat = (dat>max)?dat=max:dat;
	 dat = (dat<=min)?dat=min:dat;
	 return dat;
 }
 
 void kalman_1(struct _1_ekf_filter *ekf,float input)  //一维卡尔曼
{
	ekf->Now_P = ekf->LastP + ekf->Q;
	ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
	ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
	ekf->LastP = (1-ekf->Kg) * ekf->Now_P ;
}




