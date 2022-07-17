#include "osc.h"
  //上位机示波器
uint16_t  OutData[4] = {0}; 
//=====================================================================================
//函数名:      SciFrameTx1
//功  能:      示波器1封装   xikeda开发软件
//说  明：
//修改注释：   
//=====================================================================================

/*
void SciFrameTx(long  Data) 
{
  unsigned char ch_Temp = 0;
	unsigned char Temp_A_L;				//必须为Unsignged						
	signed char Temp_A_H;
	signed char Temp_B_L;
	signed char Temp_B_H;
	Temp_A_L = (Data >>0)&0x000000ff; 
	Temp_A_H = (Data >>8)&0x000000ff; 
	Temp_B_L = (Data >>16)&0x000000ff;
	Temp_B_H = (Data >>24)&0x000000ff;
     
        LPLD_UART_PutChar(UART0,0xaa);
        LPLD_UART_PutChar(UART0,0x09);
        LPLD_UART_PutChar(UART0,0x01);

	ch_Temp ^= 0x09;
	ch_Temp ^= 0x01;
	ch_Temp ^= Temp_B_H;/////按位异或
	ch_Temp ^= Temp_B_L;/////相同为0不同位1
	ch_Temp ^= Temp_A_H;
	ch_Temp ^= Temp_A_L;
LPLD_UART_PutChar(UART0,Temp_A_L);
LPLD_UART_PutChar(UART0,Temp_A_H);
LPLD_UART_PutChar(UART0,Temp_B_L);
LPLD_UART_PutChar(UART0,Temp_B_H);
LPLD_UART_PutChar(UART0,ch_Temp);
LPLD_UART_PutChar(UART0,0x55);
}*/ 

//=====================================================================================
//函数名:     1: CRC_CHECK   2: OutPut_Data  
//功  能:      示波器2封装
//说  明：
//修改注释：   
//=====================================================================================
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

unsigned char databuf[10];
/////////////////////////////////////////////////////////

void OutPut_Data(void)
{
  unsigned int temp;
  unsigned int i;
  unsigned short CRC16;
	
  for(i=0;i<4;i++)
   {
    temp =(uint32_t)(OutData[i]);
    databuf[i*2]   = (unsigned char)(temp%256);
    databuf[i*2+1] = (unsigned char)(temp/256);
   }
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
 
//  for(i=0;i<10;i++)
//	 {
//		UART_PutChar(UART2,databuf[i]);
//	 }
}




