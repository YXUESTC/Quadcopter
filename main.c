#include<reg52.h> 
#include<intrins.h>
sbit trig=P3^2;   //外部中断INT0 杨旭不过
sbit echo=P3^3;	  //外部中断INT1
sbit dula=P2^6;
sbit wela=P2^7;
int time0,time1,time,flag=0,distant;
char table[]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
void delay(int x)
{
	  int i,j;
	  for(i=x;i>0;i--)
	  for(j=101;j>0;j--);
}
void delay_10us()
{
  
  _nop_();
  _nop_();
  _nop_();
  _nop_();
  _nop_();
  _nop_();
  _nop_();
  _nop_();
  _nop_();
  _nop_();	 	 	 
}
void display()
{
 	 int ge,shi,bai;
		  bai=distant/100;
		  shi=(distant%100)/10;
		  ge=distant%10; 
		  dula=1;
		  P0=table[bai];
		  dula=0;
		  P0=0xff;
		  wela=1;
		  P0=0xfe;
		  wela=0;
		  delay(1);
		
		  dula=1;
		  P0=table[shi];
		  dula=0;
		  P0=0xff;
		  wela=1;
		  P0=0xfd;
		  wela=0;
		  delay(1);
		  
		  dula=1;
		  P0=table[ge];
		  dula=0;
		  P0=0xff;
		  wela=1;
		  P0=0xfb;
		  wela=0;
}
void main()
{
   trig=0;
   echo=0;
   TMOD=0x10;
   TH1=0;
   TL1=0;
  while(1)
  {
	  trig=1;
	  delay_10us();
	  trig=0;
	  while(echo==0);
	  EA=1;
	  EX1=1;
	  ET1=1;
	  TR1=1;
	  delay(20);
	  
  if(flag==1)
  {
      flag=0;
	  
	  TH1=0;
	  TL1=0;
  }
  else
  {
	    distant=0;
		display();
  }
 
  }
}
void sert() interrupt 2
{
      TR1=0;
	  EA=0;
	  time1=TH1;
	  time0=TL1;
	  time=time1*256+time0;
	  distant=time*0.017;    //算出来的厘米
	  display();
 	  flag=1;
	  
	  
}
void ser()interrupt 3
{
	  TH1=0;
	  TL1=0;
}