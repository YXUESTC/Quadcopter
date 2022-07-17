#include "ALL_DEFINE.h"
#include "PID.h"





//V1.7
/*
1ƫ���޷���־��  2�����޷���־��3���ַ����־��   4������
5����            6ƫ�        7�ϴ�ƫ�       8ƫ���޷�ֵ��
9���ַ���ƫ��ֵ��10����ֵ       11�����޷�ֵ��    12���Ʋ���Kp��
13���Ʋ���Ki��   14���Ʋ���Kd�� 15�������������  16�ϴο����������
17������޷���
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,35  ,0  ,0 , 20,  1.2    ,0.000    ,0.00     ,0  ,0 , 250},//Pitch_Angle;ƫ���Ƕ�
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 ,150,  1.0     ,0.006     ,0.5     ,0  ,0 ,500},//Pitch_Gyro;ƫ�����ٶ�*/
const float Control_Unit[15][17]=
{
/*                                         Kp        Ki        Kd            */
 /*1  2  3  4  5  6   7  8   9   10  11    12        13     14  15  16  17*/
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,35,0  ,0 , 20,    5.6   ,0.0000  ,0.00 ,0  ,0 , 250},//Pitch_Angle;ƫ���Ƕ�
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 , 200,  0.65   ,0.0055  ,3.0  ,0  ,0 ,500},//Pitch_Gyro;ƫ�����ٶ�
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,35  ,0  ,0 , 20,  5.6   ,0.0000  ,0.00 ,0  ,0 , 250},//Roll_Angle;�����
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 , 200,  0.65   ,0.0055  ,3.0  ,0  ,0 ,500},//Roll_Gyro;������ٶ�
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,35  ,0  ,0 , 0 ,   7.0   ,0       ,0.00  ,0  ,0 ,100},//Yaw_Angle;ƫ����
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100 ,0  ,0 , 30 ,  2.50   ,0.002   ,4.5   ,0  ,0 ,120},//Yaw_Gyro;ƫ�����ٶ�
 //���߲���
 //�߶ȵ���������ƣ���ƫ���޷����������Ϊ����������½��ٶ�45cm/s
 //Z���ٶȱ���+���ֿ��ƣ���ƫ���޷�
#if (YAW_Pos_Control_Accel_Disable==1)
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 ,50 ,   1.0     ,0.000   ,0    ,0   ,0 ,400},//High_Position;���θ߶�λ��
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,400 ,0  ,0 ,400 ,  2.0     ,0.05   ,0.15  ,0  ,0 ,600},//High_Speed;���������ٶ�
#else
 //{1  ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 ,100 ,  1.5     ,0.000   ,0    ,0  ,0 ,400},//High_Position;���θ߶�λ��
 //{1  ,1 ,0 ,0 ,0 ,0 , 0 ,400 ,0  ,0 ,500 ,  3.0     ,0.000   ,0    ,0  ,0 ,700},//High_Speed;���������ٶ�
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,200 ,0  ,0 ,100 ,  2.3     ,0.0023   ,0.2    ,0  ,0 ,400},//High_Position;���θ߶�λ��
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,400 ,0  ,0 ,500 ,  0.4     ,0.000   ,0.1    ,0  ,0 ,700},//High_Speed;���������ٶ�
#endif
 /*
1ƫ���޷���־��  2�����޷���־��3���ַ����־��   4������
5����            6ƫ�        7�ϴ�ƫ�       8ƫ���޷�ֵ��
9���ַ���ƫ��ֵ��10����ֵ       11�����޷�ֵ��    12���Ʋ���Kp��
13���Ʋ���Ki��   14���Ʋ���Kd�� 15�������������  16�ϴο����������
17������޷���
*/
/*                                       Kp        Ki        Kd            */
 /*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17*/
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0 ,0 ,8,   0.10    ,0.000    ,0    ,0    ,0 ,150},//Longitude_Position;ˮƽ����λ��
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100 ,0  ,0 ,15,  0.20    ,0.0001   ,0    ,0    ,0 ,25},//Longitude_Speed;ˮƽ�����ٶ�
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100 ,0  ,0 ,8,   0.10    ,0.000    ,0    ,0    ,0 ,150},//Latitude_Position;ˮƽγ��λ��
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100 ,0  ,0 ,15,  0.20    ,0.0001   ,0    ,0    ,0 ,25},//Latitude_Speed;ˮƽγ���ٶ�
 //{1  ,1 ,0 ,0 ,0 ,0 , 0 ,100 ,0  ,0 ,8,   0.15    ,0.000    ,0    ,0    ,0 ,150},//Latitude_Position;ˮƽγ��λ��
 //{1  ,1 ,0 ,0 ,0 ,0 , 0 ,100 ,0  ,0 ,15,  0.30    ,0.0005   ,2.50 ,0    ,0 ,25},//Latitude_Speed;ˮƽγ���ٶ�
  /*************���ٶȿ�����****************/
 //�����ٶ�200cm/s^2
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,500  ,0  ,0 ,300,   0.30     ,0.0035     ,0 ,0   ,0 ,500},//��ֱ���ٶȿ�����
 //{1  ,1 ,0 ,0 ,0 ,0 , 0 ,500  ,0  ,0 ,300, 0.32     ,0.005     ,0 ,0   ,0 ,500},//��ֱ���ٶȿ�����
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0  ,0 ,3,     0.32    ,0.0001    ,0    ,0   ,0 ,150},//ˮƽ���ȷ�����ٶȿ�����
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,100  ,0  ,0 ,15,    0.45    ,0.0015    ,0.3  ,0   ,0 ,25},//ˮƽά�ȷ�����ٶȿ�����
};


_st_pidData pidData;//ϵͳ�ܿ�����

typedef enum
{
     Pitch_Angle_Controler=0,
     Pitch_Gyro_Controler=1,
     Roll_Angle_Controler=2,
     Roll_Gyro_Controler=3,
     Yaw_Angle_Controler=4,
     Yaw_Gyro_Controler=5,
     High_Position_Controler=6,
     High_Speed_Controler=7,
     Longitude_Position_Controler=8,
     Longitude_Speed_Controler=9,
     Latitude_Position_Controler=10,
     Latitude_Speed_Controler=11,
     High_Acce_Controler=12,
     Longitude_Acce_Controler=13,
     Latitude_Acce_Controler=14

}Controler_Label;

void PID_Init(PidObject *Controler,Controler_Label Label)
{
  Controler->Err_Limit_Flag=(uint8)(Control_Unit[Label][0]);//1ƫ���޷���־
  Controler->Integrate_Limit_Flag=(uint8)(Control_Unit[Label][1]);//2�����޷���־
  Controler->Integrate_Separation_Flag=(uint8)(Control_Unit[Label][2]);//3���ַ����־
  Controler->Expect=Control_Unit[Label][3];//4����
  Controler->FeedBack=Control_Unit[Label][4];//5����ֵ
  Controler->Err=Control_Unit[Label][5];//6ƫ��
  Controler->Last_Err=Control_Unit[Label][6];//7�ϴ�ƫ��
  Controler->Err_Max=Control_Unit[Label][7];//8ƫ���޷�ֵ
  Controler->Integrate_Separation_Err=Control_Unit[Label][8];//9���ַ���ƫ��ֵ
  Controler->Integrate=Control_Unit[Label][9];//10����ֵ
  Controler->Integrate_Max=Control_Unit[Label][10];//11�����޷�ֵ
  Controler->Kp=Control_Unit[Label][11];//12���Ʋ���Kp
  Controler->Ki=Control_Unit[Label][12];//13���Ʋ���Ki
  Controler->Kd=Control_Unit[Label][13];//14���Ʋ���Ki
  Controler->Control_OutPut=Control_Unit[Label][14];//15�����������
  Controler->Last_Control_OutPut=Control_Unit[Label][15];//16�ϴο����������
  Controler->Control_OutPut_Limit=Control_Unit[Label][16];//16�ϴο����������
}





void Total_PID_Init(void)
{
 PID_Init(&pidData.Pitch,Pitch_Angle_Controler);
 PID_Init(&pidData.RateX,Pitch_Gyro_Controler);
 PID_Init(&pidData.Roll,Roll_Angle_Controler);
 PID_Init(&pidData.RateY,Roll_Gyro_Controler);
 PID_Init(&pidData.Yaw,Yaw_Angle_Controler);
 PID_Init(&pidData.RateZ,Yaw_Gyro_Controler);
 PID_Init(&pidData.HeightHigh,High_Position_Controler);
 PID_Init(&pidData.HeightRate,High_Speed_Controler);
 PID_Init(&pidData.GPS_Longitude_position,Longitude_Position_Controler);
 PID_Init(&pidData.GPS_Longitude_rate,Longitude_Speed_Controler);
 PID_Init(&pidData.GPS_Latitude_position,Latitude_Position_Controler);
 PID_Init(&pidData.GPS_Latitude_rate,Latitude_Speed_Controler);

 PID_Init(&pidData.HeightAccel,High_Acce_Controler);
 PID_Init(&pidData.GPS_Longitude_Acce,Longitude_Acce_Controler);
 PID_Init(&pidData.GPS_Latitude_Acce,Latitude_Acce_Controler);
}

float PID_Control(PidObject *Controler)
{
/*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect  + Controler->Offset -Controler->FeedBack;//������ȥ�����õ�ƫ��
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
                         +Controler->Integrate//����
                         +Controler->Kd*(Controler->Err-Controler->Last_Err);//΢��
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Yaw(PidObject *Controler)
{
/*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect  + Controler->Offset -Controler->FeedBack;//������ȥ�����õ�ƫ��
/***********************ƫ����ƫ���+-180����*****************************/
  if(Controler->Err<-180)  Controler->Err=Controler->Err+360;
  if(Controler->Err>180)  Controler->Err=Controler->Err-360;

  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
                         +Controler->Integrate//����
                         +Controler->Kd*(Controler->Err-Controler->Last_Err);//΢��
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}



float PID_Control_High(PidObject *Controler)
{
/*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect  + Controler->Offset -Controler->FeedBack;//������ȥ�����õ�ƫ��
  //Controler->Err=LPButter_Vel_Error(Controler->Err);
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
                         +Controler->Integrate//����
                         +Controler->Kd*(Controler->Err-Controler->Last_Err);//΢��
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}

float Control_Device_LPF(float curr_inputer,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
        /* ���ٶȼ�Butterworth�˲� */
	/* ��ȡ����x(n) */
        Buffer->Input_Butter[2]=curr_inputer;
	/* Butterworth�˲� */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) ���б��� */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) ���б��� */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
        return (Buffer->Output_Butter[2]);
}



Butter_Parameter Control_Device_Div_LPF_Parameter={
 //200---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};

Butter_Parameter Control_Device_Err_LPF_Parameter={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};


float PID_Control_Div_LPF(PidObject *Controler)
{
  int16  i=0;
/*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect + Controler->Offset - Controler->FeedBack;//������ȥ�����õ�ƫ��
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//ԭʼ΢��
  for(i=4;i>0;i--)//���ֵ�ͨ��΢�����
  {
  Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Div_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz

  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
  if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
                         +Controler->Integrate//����
//                         +Controler->Kd*Controler->Dis_Err;//΢��
                         +Controler->Kd*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}


float PID_Control_Err_LPF(PidObject *Controler)
{
/*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect + Controler->Offset - Controler->FeedBack;//������ȥ�����õ�ƫ��
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//ԭʼ΢��

  Controler->Last_Err_LPF=Controler->Err_LPF;
  Controler->Err_LPF=Control_Device_LPF(Controler->Err,
                                  &Controler->Control_Device_LPF_Buffer,
                                  &Control_Device_Err_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz

  Controler->Dis_Err_LPF=Controler->Err_LPF-Controler->Last_Err_LPF;//ƫ�����ͨ���΢����

  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
  if(Controler->Err_LPF>=Controler->Err_Max)   Controler->Err_LPF= Controler->Err_Max;
  if(Controler->Err_LPF<=-Controler->Err_Max)  Controler->Err_LPF=-Controler->Err_Max;
  }
/*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err_LPF)<=Controler->Integrate_Separation_Err)
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF;
  }
/*******�����޷�*********************/
 if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
 {
  if(Controler->Integrate>=Controler->Integrate_Max)
    Controler->Integrate=Controler->Integrate_Max;
  if(Controler->Integrate<=-Controler->Integrate_Max)
    Controler->Integrate=-Controler->Integrate_Max ;
 }
/*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err_LPF//����
                         +Controler->Integrate//����
                          +Controler->Kd*Controler->Dis_Err_LPF;//�Ѷ�ƫ���ͨ���˴����ٶ�΢�������ͨ
/*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
  Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
/*******���������*********************/
  return Controler->Control_OutPut;
}


void pidRest(PidObject *Controler,uint8_t len)
{
	while(len)
	{
		Controler-> Expect = 0;
		Controler-> Offset = 0;
		Controler-> FeedBack = 0;
		Controler-> Err = 0;
		Controler-> Last_Err = 0;
		Controler-> Integrate = 0;
		Controler-> Control_OutPut = 0;
		Controler-> Last_Control_OutPut = 0;
		Controler-> Last_FeedBack = 0;
		Controler-> Dis_Err = 0;
		Controler-> Err_LPF = 0;
		Controler-> Last_Err_LPF = 0;
		Controler-> Dis_Err_LPF = 0;
		Controler-> Dis_Error_History[0] = 0;
		Controler-> Dis_Error_History[1] = 0;
		Controler-> Dis_Error_History[2] = 0;
		Controler-> Dis_Error_History[3] = 0;	
		Controler-> Dis_Error_History[4] = 0;	
		len--;
		Controler++;
	}

}


