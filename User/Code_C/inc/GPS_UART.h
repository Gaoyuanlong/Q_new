#ifndef __GPS_UART_H__
#define __GPS_UART_H__
#include "stm32f4xx.h"
#include "Queue.h"
#include "Common.h"

//#define ENABLE_GPS_PC //GPS占用串口1进行调试  需要统一波特率为GPS波特率 9600


extern struct GPS_UART_
{
	void (*Init)(u32 Bound);
	BOOL (*send)(u8 *data, u16 num);
	BOOL (*receive)(u8 *data, u16 num);
	void (*GPS_Cof)(void);
}GPS_UART;

#endif

