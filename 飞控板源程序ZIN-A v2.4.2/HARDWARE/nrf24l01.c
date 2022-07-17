/**************************************************************
 * 
 * @brief
   ZIN-7�׼�
	 �ɿذ���Ⱥ551883670
	 �Ա���ַ��https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/

#include "nrf24l01.h"
#include <string.h>
#include <stdlib.h>
uint8_t SPI_ReadWriteByte(uint8_t TxData);
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01������   
#define NRF24L01_CSN_H    (GPIOB->BSRRL = GPIO_Pin_12) // PB12
#define NRF24L01_CSN_L    (GPIOB->BSRRH = GPIO_Pin_12)    // PB12
#define NRF24L01_CE_H     (GPIOD->BSRRL = GPIO_Pin_9)    // PD9
#define NRF24L01_CE_L     (GPIOD->BSRRH = GPIO_Pin_9)    // PD9
#define NRF24L01_IRQ_IN   (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8))//IRQ������������ PD8
#define NRF24L01_IRQ_H    (GPIOD->MODER&=~(3<<(8*2)),GPIOD->MODER|=1<<(8*2),GPIOD->BSRRL = GPIO_Pin_8)
#define NRF24L01_IRQ_L    (GPIOD->MODER&=~(3<<(8*2)),GPIOD->MODER|=1<<(8*2),GPIOD->BSRRH = GPIO_Pin_8)
#define NRF24L01_MOSI_H   (GPIOB->BSRRL = GPIO_Pin_15)
#define NRF24L01_MOSI_L   (GPIOB->BSRRH = GPIO_Pin_15)
#define NRF24L01_MISO_IN  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)
#define NRF24L01_MISO_H   (GPIOB->MODER&=~(3<<(14*2)),GPIOB->MODER|=1<<(14*2),GPIOB->BSRRL = GPIO_Pin_14)
#define NRF24L01_SCK_H    (GPIOB->BSRRL = GPIO_Pin_13)
#define NRF24L01_SCK_L    (GPIOB->BSRRH = GPIO_Pin_13)


//��ʼ��24L01��IO��
void NRF24L01_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);    //ʹ��GPIO��ʱ��  CE  PD9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          //NRF24L01  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    //�������
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //ʹ��GPIO��ʱ�� CSN    PB12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    //�������
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

    //����NRF2401��IRQ  PD8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;    //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//MISO  PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;    //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//MOSI  PB15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    //�������
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SCK  PB13
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    //�������
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//��ַ�������ʾ,���ֽ���ǰ,���ڲ鿴.
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ

//��ʼ��24L01��IO��
void My_NRF24L01_Init(void)
{
	NRF24L01_CE_H;
	NRF24L01_CSN_H;
	NRF24L01_SCK_H;
	NRF24L01_MOSI_H;
	NRF24L01_MISO_H;
	NRF24L01_IRQ_H;
  
	NRF24L01_CE_L;
	NRF24L01_CSN_H;	 	
	NRF24L01_SCK_L;	
}

//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	 	 

//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
	NRF24L01_CSN_L;                 //ʹ��SPI����
	status =SPI_ReadWriteByte(reg);//���ͼĴ����� 
	SPI_ReadWriteByte(value);      //д��Ĵ�����ֵ
	NRF24L01_CSN_H;                 //��ֹSPI����	   
	return(status);       			//����״ֵ̬
}

//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_CSN_L;          //ʹ��SPI����		
	SPI_ReadWriteByte(reg);   //���ͼĴ�����
	reg_val=SPI_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
	NRF24L01_CSN_H;          //��ֹSPI����		    
	return(reg_val);           //����״ֵ̬
}	

//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,u8_ctr;	       
	NRF24L01_CSN_L;           //ʹ��SPI����
	status=SPI_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI_ReadWriteByte(0XFF);//��������
	NRF24L01_CSN_H;       //�ر�SPI����
	return status;        //���ض�����״ֵ̬
}

//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,u8_ctr;	    
 	NRF24L01_CSN_L;          //ʹ��SPI����
	status = SPI_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI_ReadWriteByte(*pBuf++); //д������	 
	NRF24L01_CSN_H;       //�ر�SPI����
	return status;          //���ض�����״ֵ̬
}				   

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
	NRF24L01_CE_L;
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	NRF24L01_CE_H;//��������	
  GPIOD->MODER&=~(3<<(8*2));
	GPIOD->MODER|=0<<(8*2);
	while((NRF24L01_IRQ_IN)!=0);//�ȴ��������
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
}

//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE_L;  
	NRF24L01_SCK_L;
	
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
	NRF24L01_Write_Reg(WRITE_REG_NRF+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,40);       //����RFͨ��Ϊ40
	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
	NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG_2401,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	
	NRF24L01_CE_H;//CEΪ��,10us����������
}		  



uint8_t SPI_ReadWriteByte(uint8_t TxData)
{
	uint8_t bit_ctr;
	uint8_t bit_temp;
	for(bit_ctr=0; bit_ctr < 8; bit_ctr++) 	// output 8-bit
	{
		bit_temp = TxData & 0x80;
		
		if(bit_temp)													// output 'TxData', MSB to MOSI
			NRF24L01_MOSI_H;
		else
			NRF24L01_MOSI_L;
		
		TxData = (TxData << 1);           		// shift next bit into MSB..
		
		NRF24L01_SCK_H;                   // Set SCK high..
		GPIOB->MODER&=~(3<<(14*2));
		GPIOB->MODER|=0<<(14*2);
		bit_temp = (uint8_t)NRF24L01_MISO_IN;
		
		if(bit_temp)
			TxData |= 1;           							// capture current MISO bit
		
		NRF24L01_SCK_L;                		// ..then set SCK low again
	}
	return(TxData);               					// return read uchar
}

/*********************END OF FILE******************************************************/





















/*********************END OF FILE******************************************************/
















