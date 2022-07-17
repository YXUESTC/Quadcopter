/**************************************************************
 * 
 * @brief
   ZIN-7套件
	 飞控爱好群551883670
	 淘宝地址：https://shop297229812.taobao.com/shop/view_shop.htm?mytmenu=mdianpu&user_number_id=2419305772
 ***************************************************************/

#include "nrf24l01.h"
#include <string.h>
#include <stdlib.h>
uint8_t SPI_ReadWriteByte(uint8_t TxData);
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01操作线   
#define NRF24L01_CSN_H    (GPIOB->BSRRL = GPIO_Pin_12) // PB12
#define NRF24L01_CSN_L    (GPIOB->BSRRH = GPIO_Pin_12)    // PB12
#define NRF24L01_CE_H     (GPIOD->BSRRL = GPIO_Pin_9)    // PD9
#define NRF24L01_CE_L     (GPIOD->BSRRH = GPIO_Pin_9)    // PD9
#define NRF24L01_IRQ_IN   (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8))//IRQ主机数据输入 PD8
#define NRF24L01_IRQ_H    (GPIOD->MODER&=~(3<<(8*2)),GPIOD->MODER|=1<<(8*2),GPIOD->BSRRL = GPIO_Pin_8)
#define NRF24L01_IRQ_L    (GPIOD->MODER&=~(3<<(8*2)),GPIOD->MODER|=1<<(8*2),GPIOD->BSRRH = GPIO_Pin_8)
#define NRF24L01_MOSI_H   (GPIOB->BSRRL = GPIO_Pin_15)
#define NRF24L01_MOSI_L   (GPIOB->BSRRH = GPIO_Pin_15)
#define NRF24L01_MISO_IN  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)
#define NRF24L01_MISO_H   (GPIOB->MODER&=~(3<<(14*2)),GPIOB->MODER|=1<<(14*2),GPIOB->BSRRL = GPIO_Pin_14)
#define NRF24L01_SCK_H    (GPIOB->BSRRL = GPIO_Pin_13)
#define NRF24L01_SCK_L    (GPIOB->BSRRH = GPIO_Pin_13)


//初始化24L01的IO口
void NRF24L01_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);    //使能GPIO的时钟  CE  PD9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          //NRF24L01  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    //推挽输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //使能GPIO的时钟 CSN    PB12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    //推挽输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

    //配置NRF2401的IRQ  PD8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;    //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//MISO  PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;    //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//MOSI  PB15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    //推挽输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SCK  PB13
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    //推挽输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//地址用数组表示,低字节在前,便于查看.
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址

//初始化24L01的IO口
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

//检测24L01是否存在
//返回值:0，成功;1，失败	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}	 	 

//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
	NRF24L01_CSN_L;                 //使能SPI传输
	status =SPI_ReadWriteByte(reg);//发送寄存器号 
	SPI_ReadWriteByte(value);      //写入寄存器的值
	NRF24L01_CSN_H;                 //禁止SPI传输	   
	return(status);       			//返回状态值
}

//读取SPI寄存器值
//reg:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_CSN_L;          //使能SPI传输		
	SPI_ReadWriteByte(reg);   //发送寄存器号
	reg_val=SPI_ReadWriteByte(0XFF);//读取寄存器内容
	NRF24L01_CSN_H;          //禁止SPI传输		    
	return(reg_val);           //返回状态值
}	

//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,u8_ctr;	       
	NRF24L01_CSN_L;           //使能SPI传输
	status=SPI_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI_ReadWriteByte(0XFF);//读出数据
	NRF24L01_CSN_H;       //关闭SPI传输
	return status;        //返回读到的状态值
}

//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,u8_ctr;	    
 	NRF24L01_CSN_L;          //使能SPI传输
	status = SPI_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI_ReadWriteByte(*pBuf++); //写入数据	 
	NRF24L01_CSN_H;       //关闭SPI传输
	return status;          //返回读到的状态值
}				   

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
	NRF24L01_CE_L;
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	NRF24L01_CE_H;//启动发送	
  GPIOD->MODER&=~(3<<(8*2));
	GPIOD->MODER|=0<<(8*2);
	while((NRF24L01_IRQ_IN)!=0);//等待发送完成
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	   
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}

//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE_L;  
	NRF24L01_SCK_L;
	
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x01);     //使能通道0的自动应答    
	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01); //使能通道0的接收地址  
	NRF24L01_Write_Reg(WRITE_REG_NRF+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,40);       //设置RF通道为40
	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG_2401,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	
	NRF24L01_CE_H;//CE为高,10us后启动发送
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
















