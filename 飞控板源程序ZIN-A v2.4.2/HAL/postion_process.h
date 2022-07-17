#ifndef _POSITION_H_
#define _POSITION_H_


//��������ϵ��
//Longitude   W_E��������
//Latitude    N_S�ϱ�γ��
typedef struct
{	
	float Longitude_W_E_Origin; //ԭʼ�ó���HOME��ĵľ���
	float Latitude_N_S_Origin; 	
	float Longitude_W_E_Distance; //�ںϺ� ����HOME��ľ�������PID����
	float Latitude_N_S_Distance; 
	float Longitude_W_E_Speed;    //�ںϺ� ����ǰ��γ���ϵ��ٶ�����PID����
	float Latitude_N_S_Speed;
	uint8_t Filter_Defeated_Flag; //�ں�ʧ�ܱ�־
}_st_Earth_Position;


extern _st_Earth_Position Earth_Position;

void Filter_Horizontal(void);
void Strapdown_INS_Horizontal(void);
#endif

