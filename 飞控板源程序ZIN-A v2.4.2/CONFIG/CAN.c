#include "stm32f4xx.h"
#include "CAN.h"


//	//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������500Kbps    

////////////////////////////////////////////////////////////////////////////////// 	 

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((6+7+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 

uint8_t CAN1_buf[8];
uint8_t CAN2_buf[8];
uint8_t CAN_BUF_1[600][8];
uint8_t CAN_BUF_2[600][8];
int buf1 = 0,buf2 = 0;

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
		u16 std_id_1 =0x123;  
	
		u16 std_id_2 =0x123;  
	
  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	   
	
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12
		
//		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
//    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA12,PA13
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
//		GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11����ΪCAN2
//	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12����ΪCAN2
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
//		CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
//  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
//  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
//  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
//  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
//  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
//  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
//  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
//  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
//  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
//  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
//  	CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN2
		//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //����ģʽ
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)std_id_1<<21)&0xFFFF0000)>>16;;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=(((u32)std_id_1<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF; 	

  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
/*---------------------------------------------------------------------------------------------------------------------------------*/		
		
//		CAN_FilterInitStructure.CAN_FilterNumber=14; //������14
//  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //����ģʽ
//  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
//  	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)std_id_2<<21)&0xFFFF0000)>>16;;////32λID
//  	CAN_FilterInitStructure.CAN_FilterIdLow=(((u32)std_id_2<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF; 
//		
//  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;//32λMASK
//  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
//   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
//  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
//		CAN_FilterInit(&CAN_FilterInitStructure);  //CAN2
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif

//#if CAN2_RX0_INT_ENABLE
//	
//	  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
//  
//  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  	NVIC_Init(&NVIC_InitStructure);
//#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
		for(i=0;i<8;i++)
	  {CAN_BUF_1[buf1][i] = RxMessage.Data[i];
		}
		buf1++;
//	printf("CAN1�жϽ���:");
//	for(i=0;i<8;i++)
//	printf("%d,",RxMessage.Data[i]);
//	printf("\r\n");
}
#endif

//#if CAN2_RX0_INT_ENABLE	//ʹ��RX0�ж�
////�жϷ�����			    
//void CAN2_RX0_IRQHandler(void)
//{
//  	CanRxMsg RxMessage;
//	int i=0;
//    CAN_Receive(CAN2, 0, &RxMessage);
//	for(i=0;i<8;i++)
//	     CAN_BUF_2[buf2][i] = RxMessage.Data[i];
//	buf2++;
////	printf("%d,",RxMessage.Data[i]);
////	printf("\r\n");
//}
//#endif


//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		

}

//u8 CAN2_Send_Msg(u8* msg,u8 len)
//{	
//  u8 mbox;
//  u16 i=0;
//  CanTxMsg TxMessage;
//  TxMessage.StdId=0x15;	 // ��׼��ʶ��Ϊ0
//  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
//  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
//  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
//  TxMessage.DLC=len;							 // ������֡��Ϣ
//  for(i=0;i<len;i++)
//  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
//  mbox= CAN_Transmit(CAN2, &TxMessage);   
//  i=0;
//  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
//  if(i>=0XFFF)return 1;
//  return 0;		

//}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

//u8 CAN2_Receive_Msg(u8 *buf)
//{
// 	u32 i;
//	CanRxMsg RxMessage;
//    if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
//    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//��ȡ����	
//    for(i=0;i<RxMessage.DLC;i++)
//    buf[i]=RxMessage.Data[i];  
//	return RxMessage.DLC;	
//}







