#include "../inc/sys_debug.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_uart.h"

#define SYS_GLASS_DEBUG 		1

void uart_printHex(u8 *buf,u8 len)
{
#if (SYS_GLASS_DEBUG != 0)
	u8 cbuf[256];
	u8 cmid;
	int i;
	if(len > 126)
	{
		len = 126;
	}
	memset(cbuf,0,256);
	for(i = 0;i<len;i++)
	{
		cmid = buf[i];
		if(((buf[i] >> 4) & 0x0f) > 9)
		{
			cbuf[2*i] = ((buf[i] >> 4) & 0x0f) + 0x37;
		}
		else
		{
			cbuf[2*i] = ((buf[i] >> 4) & 0x0f) + 0x30;
		}

		if((cmid & 0x0f) > 9)
		{
			cbuf[(2*i)+1] = (cmid & 0x0f) + 0x37;
		}
		else
		{
			cbuf[(2*i)+1] = (cmid & 0x0f) + 0x30;
		}
	}
	cbuf[2*i] = '\n';
	//cbuf[(2*i) + 1] = '\n';
	NRF_LOG(cbuf);
#endif	
}


void uart_NRF_LOG(u8 type,u8 *buf)
{
	NRF_LOG(buf);
}

	


