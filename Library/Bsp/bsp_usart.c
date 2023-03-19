/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-19 21:03:53
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: debug 初始化结构体
 * @detail: 
 * @param {ULONG} ulBound
 * @return {*}
 * @author: lkc
 */
STATIC VOID Bsp_Debug_InitType(ULONG ulBound)
{
	DEBUG_USART_CLK_EN();
	DEBUG_USART_TX_CLK_EN();
	DEBUG_USART_RX_CLK_EN();

	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = ulBound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
	USART_Init(DEBUG_USART, &USART_InitStructure); 

	USART_Cmd(DEBUG_USART, ENABLE);

	USART_ClearFlag(DEBUG_USART, USART_FLAG_TC);

	/* 清空发送缓冲区 */
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET)
	{
		USART_ReceiveData(DEBUG_USART);
	}

	return;
}

/**
 * @description: debug 中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
STATIC VOID Bsp_Debug_Nvic(VOID)
{
	/* 空闲中断使用命令行 */
	NVIC_InitTypeDef NVIC_InitStre;
	NVIC_InitStre.NVIC_IRQChannel = DEBUG_USART_IRQn;
    NVIC_InitStre.NVIC_IRQChannelPreemptionPriority = IRQ_DEBUG_IDE;
    NVIC_InitStre.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStre.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStre);

	/* 空闲中断,接收到一连串数据触发一次 */
	USART_ITConfig(DEBUG_USART, USART_IT_IDLE, ENABLE);
	USART_ClearITPendingBit(DEBUG_USART, USART_IT_IDLE);

	return;
}

/**
 * @Description: 串口初始化
 * @author: lkc
 * @Date: 2023-02-17 23:06:43
 * @param {int} baud
 * @return {*}
 */
VOID Bsp_Usart_DebugInit(ULONG ulBound)
{
	Bsp_Debug_InitType(ulBound);
	Bsp_Debug_Nvic();


    return;
}

/**
 * @Description: 串口输出字符串
 * @author: lkc
 * @Date: 2023-02-17 23:01:32
 * @return {*}
 */
void Bsp_Usart_PutString(const CHAR *cStr)
{
	/* 直到字符串结束 */
	while (*cStr != '\0')
	{ 
		/* 换行 */
/*
#ifdef ENTER_PRI
		if (*cStr == '\n')	
		{
			USART_SendData(DEBUG_USART, ENTER_PRI);
			while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
		}
#endif
*/
		USART_SendData(DEBUG_USART, *cStr++);
		IS_TIME_OUT(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);

	}		

    return;
}

/**
 * @Description: 串口输出N个字符
 * @author: lkc
 * @Date: 2023-02-17 23:01:32
 * @return {*}
 */
VOID Bsp_Usart_PutBuf(const UCHAR *cStr, ULONG ulLen)
{
	while (ulLen--)
	{
		/* 换行 */
#ifdef ENTER_PRI
		if (*cStr == '\n')	
		{
			USART_SendData(DEBUG_USART, ENTER_PRI);
			IS_TIME_OUT(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
		}
#endif
		USART_SendData(DEBUG_USART, *cStr++);
		IS_TIME_OUT(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
	}	

    return;
}


/**
  * @brief  输出到终端一个字符
  * @param  cKey: 输出字符
  * @retval None
  */
void Bsp_Usart_PutChar(UCHAR cKey)
{
	USART_SendData(DEBUG_USART, cKey);
	IS_TIME_OUT(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);

	return;
}

/**
  * @brief  Test to see if a ucKey has been pressed on the HyperTerminal
  * @param  ucKey: The ucKey pressed
  * @retval 1: Correct
  *         0: Error
  */
ULONG Bsp_Usart_GetChar(UCHAR *ucKey)
{
	/* 当存在数据的时候 */
	if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) != RESET)
	{
		/* 取出数据 */
		*ucKey = (uint8_t)DEBUG_USART->DR;
		return SUCCESS;
	}

	return ERROR;
}

/**
 * @description: 
 * @detail: 
 * @param {UCHAR} *ucBuf
 * @return {*}
 * @author: lkc
 */
ULONG Bsp_Usart_GetBuf(UCHAR *ucBuf, ULONG ulLen)
{
	ULONG i = 0;

	for (i = 0; i < ulLen; i++)
	{
		/* 当存在数据的时候 */
		if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) != RESET)
		{
			/* 取出数据 */
			ucBuf[i] = (uint8_t)DEBUG_USART->DR;
			return SUCCESS;
		}
	}
	
	return ulLen;
}

/**
 * @Description: 重定向fputc
 * @author: lkc
 * @Date: 2023-02-18 09:44:40
 * @param {int} ch
 * @param {FILE} *f
 * @return {*}
 */
int fputc(int ch, FILE *f)
{
#ifdef ENTER_PRI
	/* 如果 */
	if ('\n' == ch)
	{
		USART_SendData(DEBUG_USART, (u8)ENTER_PRI);
		IS_TIME_OUT(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET);
	}
#endif
	USART_SendData(DEBUG_USART, (u8)ch);
	IS_TIME_OUT(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET);

    return ch;
}

/**
 * @description: 串口3中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
extern QueueHandle_t queueCmdHandle;
ULONG ulRxLen = 0;
VOID Bsp_Usart_Handler(VOID)
{
    if (RESET != USART_GetITStatus(DEBUG_USART, USART_IT_IDLE))
    {
		/* 个人认为是清空缓冲,取出数据 */
		DEBUG_USART->SR;
		DEBUG_USART->DR;

		/* 清空空闲中断 */
		USART_ClearITPendingBit(DEBUG_USART, USART_IT_IDLE);
		/* 关闭dma */
		DMA_Cmd(DMA1_Stream1, DISABLE);
		/* 本帧数据长度=DMA准备的接收数据长度-DMA已接收数据长度 */
		ulRxLen = USART_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1); 

		/* 此时知道接收的长度以及Buf,直接传一个队列 */
		osMessageQueuePut(queueCmdHandle, &ulRxLen, NULL, 0);
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF4 | DMA_FLAG_FEIF4 | DMA_FLAG_DMEIF4 |
				DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4);//清除DMA2_Steam7传输完成标志
		IS_TIME_OUT(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
		
		DMA_SetCurrDataCounter(DMA1_Stream1, USART_RX_BUF_SIZE);
		DMA_Cmd(DMA1_Stream1, ENABLE);
	}
	
	return;
}

#ifdef __cplusplus
}
#endif
