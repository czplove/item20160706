/*
*********************************************************************************************************
*
*	模块名称 : 串口驱动模块
*	文件名称 : bsp_uart.c
*	版    本 : V1.0
*	说    明 : 配置UART工作模式
*********************************************************************************************************
*/

//#include "bsp.h"
#include "bsp_uart.h"

UART_COMM_T g_tComm;

/*
*********************************************************************************************************
*	函 数 名: bsp_InitUart
*	功能说明: 初始化CPU的USART1串口硬件设备。未启用中断。
*	形    参: _baud 波特率
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitUart(uint32_t _baud)
{
	g_tComm.Baud = _baud;
	
#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)	  
	UART2_DeInit();

	/* 配置 UART1
	    - BaudRate = 115200 baud
	    - Word Length = 8 Bits
	    - One Stop Bit
	    - No parity
	    - Receive and transmit enabled
	    - UART1 Clock disabled
	*/
	UART2_Init((uint32_t)_baud, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO,
	          UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
#endif


#if defined(STM8S208) ||defined(STM8S207) || defined(STM8S007) ||defined(STM8S103) ||\
    defined(STM8S003) || defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)		
	
	#if 0	
		UART1_DeInit();
	
		/* 配置 UART1
			- BaudRate = 115200 baud
			- Word Length = 8 Bits
		 	- One Stop Bit
			- No parity
			- Receive and transmit enabled
			- UART1 Clock disabled
		*/
		UART1_Init((uint32_t)_baud, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
				  UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);	
	#else
		/* 208 */
		UART3_DeInit();
	
		/* 配置 UART3
			- BaudRate = 9600 baud
			- Word Length = 8 Bits
			- One Stop Bit
			- No parity
			- Receive and transmit enabled
			- UART1 Clock disabled
		*/
		UART3_Init((uint32_t)_baud, UART3_WORDLENGTH_8D, UART3_STOPBITS_1, UART3_PARITY_NO,
				  UART3_MODE_TXRX_ENABLE);	

        UART3_ITConfig(UART3_IT_RXNE_OR, ENABLE);
	#endif
#endif	

#ifdef RS485_EN	
	/* 配置PD4 = 0 */
	GPIO_Init(GPIOD, (GPIO_Pin_TypeDef)GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_SLOW);
#endif		
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SendUart
*	功能说明: 向串口发送一组数据
*	形    参：_pBuf 数据缓冲区
*			 _ucLen 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SendUart(uint8_t *_pBuf, uint8_t _ucLen)
{
#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)	   
	while (_ucLen--)
	{
  		UART2_SendData8(*_pBuf++);
  		while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);	/* 等待发送寄存器为空 */
  	}
  	while (UART2_GetFlagStatus(UART2_FLAG_TC) == RESET);	/* 必须等待最后1个字节发送完毕 */
#endif

#if defined(STM8S208) ||defined(STM8S207) || defined(STM8S007) ||defined(STM8S103) ||\
    defined(STM8S003) || defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)	

	#if 0
		while (_ucLen--)
		{
			UART1_SendData8(*_pBuf++);
			while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	/* 等待发送寄存器为空 */
		}
		while (UART1_GetFlagStatus(UART1_FLAG_TC) == RESET);	/* 必须等待最后1个字节发送完毕 */
	#else	/*　208 有两个串口 */
		while (_ucLen--)
		{
			UART3_SendData8(*_pBuf++);
			while (UART3_GetFlagStatus(UART3_FLAG_TXE) == RESET);	/* 等待发送寄存器为空 */
		}
		while (UART3_GetFlagStatus(UART3_FLAG_TC) == RESET);	/* 必须等待最后1个字节发送完毕 */	
	#endif
#endif
}

/*
*********************************************************************************************************
*	函 数 名: bsp_UartRxNew
*	功能说明: 判断UART是否收到新数据
*	形    参: 无
*	返 回 值: 1表示有， 0表示无
*********************************************************************************************************
*/
static uint8_t bsp_UartRxNew(void)
{
#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)	   
	if (UART2_GetFlagStatus(UART2_FLAG_RXNE) != RESET)
	{
		return 1;
	}
	else
	{
	  	return 0;
	}
#endif

#if defined(STM8S208) ||defined(STM8S207) || defined(STM8S007) ||defined(STM8S103) ||\
    defined(STM8S003) || defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)	
	
	#if 0
		if (UART1_GetFlagStatus(UART1_FLAG_RXNE) != RESET)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	#else
		if (UART3_GetFlagStatus(UART3_FLAG_RXNE) != RESET)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	#endif
#endif
}

/*
*********************************************************************************************************
*	函 数 名: bsp_ReadUartData
*	功能说明: 读串口接收数据寄存器； 需要先判断是否有数据
*	形    参: 无
*	返 回 值: 字节数据
*********************************************************************************************************
*/
static uint8_t bsp_ReadUartData(void)
{
  	uint8_t rxdata;
	
#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)	   
	rxdata = UART2_ReceiveData8();
#endif

#if defined(STM8S208) ||defined(STM8S207) || defined(STM8S007) ||defined(STM8S103) ||\
    defined(STM8S003) || defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)	
	
	#if 0
		rxdata = UART1_ReceiveData8();
	#else
		rxdata = UART3_ReceiveData8();
	#endif
#endif
	return rxdata;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_ReadUartData
*	功能说明: 读串口接收数据寄存器； 需要先判断是否有数据
*	形    参: _data 数据存放地址
*	返 回 值: 0表示无数据，1表示有数据。 数据存放在_data
*********************************************************************************************************
*/
uint8_t bsp_ReadUart(uint8_t *_data)
{
	if (bsp_UartRxNew())
	{
		*_data = bsp_ReadUartData();
		return 1;
	}
	else
	{
		return 0;
	}
}
/*
*********************************************************************************************************
*	函 数 名: uart_rx_isr
*	功能说明: 串口接收中断服务程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void uart_rx_isr(void)
{
	uint8_t rxdata;

	rxdata = bsp_ReadUartData();

	/*
		3.5个字符的时间间隔，只是用在RTU模式下面，因为RTU模式没有开始符和结束符，
		两个数据包之间只能靠时间间隔来区分，Modbus定义在不同的波特率下，间隔时间是不一样的，
		所以就是3.5个字符的时间，波特率高，这个时间间隔就小，波特率低，这个时间间隔相应就大
	*/
	bsp_StartTimer(1, 4);		/* 设置4ms 超时 (9600) */

	if (g_tComm.RxCount < RX_MAX_RTU)
	{			
		g_tComm.RxBuf[g_tComm.RxCount++] = rxdata;
	}
}

/*
*********************************************************************************************************
*	函 数 名: uart_rx_isr
*	功能说明: 串口接收中断服务程序。 需要插入主程序执行。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t rx_pro(void)
{
	/* 收到命令 
		05 06 00 88 04 57 3B70 (8 字节) 
			05    :  数码管屏的号站，
			06    :  指令
			00 88 :  数码管屏的显示寄存器
			04 57 :  数据,,,转换成 10 进制是 1111.高位在前, 
			3B70  :  二个字节 CRC 码	从05到 57的校验	
	*/
	if (bsp_CheckTimer(1))
	{
        g_tComm.RxNewFlag = 1;
        
        return 1;
    }
    return 0;
}    

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
