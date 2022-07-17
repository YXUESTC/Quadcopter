/*----------------------------------------------------------------------------
 *      RL-ARM - USB
 *----------------------------------------------------------------------------
 *      Name:    usbd_user_hid.c
 *      Purpose: Human Interface Device Class User module
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "RTL.h"
#include <rl_usb.h>
#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */
#define __NO_USB_LIB_C
#include "usb_config.c"
#include "usbd_user_hid.h"
#include "ANO_DT.h"

u8 HID_SEND_TIMEOUT = 5;			//hid发送不足一帧时，等待HID_SEND_TIMEOUT周期进行发送
u8 hid_datatemp[256];					//hid环形缓冲区
u8 hid_datatemp_begin = 0;		//环形缓冲区数据指针，指向应当发送的数据
u8 hid_datatemp_end = 0;			//环形缓冲区数据结尾
u8 hid_data2send[64];					//hid发送缓存，一个hid数据帧64字节，第一字节表示有效数据字节数，0-63，后面是数据，最多63字节
/****************************************************************************************
HID初始化
*@brief    
*@brief   
*@param[in]
*****************************************************************************************/
void Usb_Hid_Init (void) 
{
  usbd_init();
	usbd_connect(__TRUE);
}
/****************************************************************************************
HID发送数据给上位机
*@brief    
*@brief   
*@param[in]
*****************************************************************************************/
void Usb_Hid_Adddata(u8 *dataToSend , u8 length)
{
	u8  i;
	for(i=0; i<length; i++)
	{
		hid_datatemp[hid_datatemp_end++] = dataToSend[i];
	}
}
void Usb_Hid_Send (void)
{
	static u8 notfull_timeout=0;
	
	if(hid_datatemp_end > hid_datatemp_begin)
	{
		if((hid_datatemp_end - hid_datatemp_begin) >= 63)
		{
			u8 i;
			notfull_timeout = 0;
			hid_data2send[0] = 63;
			for(i=0; i<63; i++)
			{
				hid_data2send[i+1] = hid_datatemp[hid_datatemp_begin++];
			}
			usbd_hid_get_report_trigger(0, hid_data2send, 64);		//send
		}
		else
		{
			notfull_timeout++;
			if(notfull_timeout == HID_SEND_TIMEOUT)
			{
				notfull_timeout = 0;
				hid_data2send[0] = hid_datatemp_end - hid_datatemp_begin;
				for(u8 i=0; i<63; i++)
				{
					if(i<hid_datatemp_end - hid_datatemp_begin)
						hid_data2send[i+1] = hid_datatemp[hid_datatemp_begin+i];
					else
						hid_data2send[i+1] = 0;
				}
				hid_datatemp_begin = hid_datatemp_end;
				usbd_hid_get_report_trigger(0, hid_data2send, 64);		//send
			}
		}
	}
	else if(hid_datatemp_end < hid_datatemp_begin)
	{
		if((256 - hid_datatemp_begin + hid_datatemp_end) >= 63)
		{
			u8 i;
			notfull_timeout = 0;
			hid_data2send[0] = 63;
			for(i=0; i<63; i++)
			{
				hid_data2send[i+1] = hid_datatemp[hid_datatemp_begin++];
			}
			usbd_hid_get_report_trigger(0, hid_data2send, 64);		//send
		}
		else
		{
			notfull_timeout++;
			if(notfull_timeout == HID_SEND_TIMEOUT)
			{
				u8 i;
				notfull_timeout = 0;
				hid_data2send[0] = 256 - hid_datatemp_begin + hid_datatemp_end;
				for(i=0; i<63; i++)
				{
					if(i<256 - hid_datatemp_begin + hid_datatemp_end)
						hid_data2send[i+1] = hid_datatemp[(u8)(hid_datatemp_begin+i)];
					else
						hid_data2send[i+1] = 0;
				}
				hid_datatemp_begin = hid_datatemp_end;
				usbd_hid_get_report_trigger(0, hid_data2send, 64);		//dend
			}
		}
	}
}

int usbd_hid_get_report (U8 rtype, U8 rid, U8 *buf, U8 req) 
{
  switch (rtype) 
	{
    case HID_REPORT_INPUT:
      switch (rid) 
			{
         case 0:
          switch (req) 
					{
            case USBD_HID_REQ_EP_CTRL:
            case USBD_HID_REQ_PERIOD_UPDATE:
							return 64;
            case USBD_HID_REQ_EP_INT:
              break;
          }
           break;
      }
      break;
    case HID_REPORT_FEATURE:
      return (1);
  }
  return (0);
}


/****************************************************************************************
HID接收到上位机发来的数据
*@brief    
*@brief   
*@param[in]
*****************************************************************************************/
void usbd_hid_set_report (U8 rtype, U8 rid, U8 *buf, int len, U8 req) {  

  switch (rtype) {
    case HID_REPORT_OUTPUT:
		   if(buf[1] == 0xaa && buf[2] == 0xaF ) //帧头正确	
			 {    
				      u8 i;
							u8 check_sum = 0;
							for(i=1;i<buf[0];i++)  //buf[0] 为收到PC来的数据总长度
							{
								check_sum+= buf[i];
							}
							if(check_sum == buf[buf[0]]) //buf[0] 为收到PC来的数据总长度 最后一个buf为PC发来的本次数据校验和，与本次收到计算出的校验和做比较
							{
									if(buf[4] >= 0x10 && buf[5]<=0x15) //如果收到的是PID数据则要返回校验值
									{	
											checkPID = buf[4]<<8 | check_sum;  //返回PID校验数据
										  ANTO_Send(ANTO_CHECK); //收到HID发来的PID则要马上返回校验值给上位机
									}	
									ANO_Recive((int8_t*)(buf+1));		//hid接收到数据会调用此函数	
							}
							buf[1] = 0;
			 }					
      break;
    case HID_REPORT_FEATURE:
      //feat = buf[0];
      break;
  }
}
