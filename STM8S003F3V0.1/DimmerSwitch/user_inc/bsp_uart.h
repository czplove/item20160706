/*
*********************************************************************************************************
*
*	模块名称 : 串口驱动模块
*	文件名称 : bsp_uart.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2014-2015, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_UART_H
#define __BSP_UART_H
#include "stm8s.h"

#define RX_MAX_RTU  64

typedef struct
{
	uint8_t RxBuf[RX_MAX_RTU];
	uint8_t RxCount;
	uint8_t RxStatus;
	uint8_t RxNewFlag;
	
	uint16_t Baud;
}UART_COMM_T;

void bsp_InitUart(uint32_t _baud);
uint8_t bsp_ReadUart(uint8_t *_data);
uint8_t rx_pro(void);

extern UART_COMM_T g_tComm;

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
