#ifndef __MY_MATH_H
#define	__MY_MATH_H

#define RtA 	57.2957795f	//			
#define AtR    	0.01745329f	//			
#define Acc_G 	0.00239261f	//+-4g,1 / (65536 / (16 * g))			
#define Gyro_G 	0.06103608f	//+-2000,1/(65535 / 4000)			
#define Gyro_Gr	0.00106422f  //Gyro_G * AtR


#define absu16( Math_X )  ((Math_X)<0? -(Math_X):(Math_X))
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

float Q_rsqrt(float number);

#endif









