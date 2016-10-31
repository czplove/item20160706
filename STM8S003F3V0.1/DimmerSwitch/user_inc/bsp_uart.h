/*
*********************************************************************************************************
*
*	ģ������ : ��������ģ��
*	�ļ����� : bsp_uart.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2014-2015, ���������� www.armfly.com
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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
