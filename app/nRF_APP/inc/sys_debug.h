#ifndef SYS_DEBUG_H
#define SYS_DEBUG_H
#include "stdio.h"
#include "string.h"

typedef unsigned char   u8;
typedef unsigned short  u16;










void uart_printHex(u8 *buf,u8 len);
void uart_NRF_LOG(u8 type,u8 *buf);

#endif


