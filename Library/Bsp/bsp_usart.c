/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-09 23:53:26
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if 0
#define DEBUG_USART                        USART1
#define DEBUG_USART_TX_AF                  GPIO_AF_USART1
#define DEBUG_USART_RX_AF                  GPIO_AF_USART1
#define DEBUG_USART_CLK_EN()               RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)
#define DEBUG_USART_TX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)		
#define DEBUG_USART_RX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define DEBUG_USART_TX_PIN                 GPIO_Pin_9
#define DEBUG_USART_TX_GPIO_PORT           GPIOA
#define DEBUG_USART_TX_SOURCE              GPIO_PinSource9
#define DEBUG_USART_RX_PIN                 GPIO_Pin_10
#define DEBUG_USART_RX_GPIO_PORT           GPIOA
#define DEBUG_USART_RX_SOURCE              GPIO_PinSource10
#define DEBUG_USART_IRQn                   USART1_IRQn
#endif

#if 1
#define DEBUG_USART                        USART3
#define DEBUG_USART_TX_AF                  GPIO_AF_USART3
#define DEBUG_USART_RX_AF                  GPIO_AF_USART3
#define DEBUG_USART_CLK_EN()               RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE)
#define DEBUG_USART_TX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)		
#define DEBUG_USART_RX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define DEBUG_USART_TX_PIN                 GPIO_Pin_10
#define DEBUG_USART_TX_SOURCE              GPIO_PinSource10
#define DEBUG_USART_TX_GPIO_PORT           GPIOB
#define DEBUG_USART_RX_PIN                 GPIO_Pin_11
#define DEBUG_USART_RX_SOURCE              GPIO_PinSource11
#define DEBUG_USART_RX_GPIO_PORT           GPIOB
#define DEBUG_USART_IRQn                   USART3_IRQn
#endif

#if 0
#define DEBUG_USART                        UART4
#define DEBUG_USART_TX_AF                  GPIO_AF_UART4
#define DEBUG_USART_RX_AF                  GPIO_AF_UART4
#define DEBUG_USART_CLK_EN()               RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE)
#define DEBUG_USART_TX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)		
#define DEBUG_USART_RX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define DEBUG_USART_TX_PIN                 GPIO_Pin_10
#define DEBUG_USART_TX_SOURCE              GPIO_PinSource10
#define DEBUG_USART_TX_GPIO_PORT           GPIOC
#define DEBUG_USART_RX_PIN                 GPIO_Pin_11
#define DEBUG_USART_RX_SOURCE              GPIO_PinSource11
#define DEBUG_USART_RX_GPIO_PORT           GPIOC
#define DEBUG_USART_IRQn                   UART4_IRQn
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @Description: 串口初始化
 * @author: lkc
 * @Date: 2023-02-17 23:06:43
 * @param {int} baud
 * @return {*}
 */
void Bsp_Usart_DebugInit(uint32_t ulBound)
{
	DEBUG_USART_CLK_EN();
	DEBUG_USART_TX_CLK_EN();
	DEBUG_USART_RX_CLK_EN();

	GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_SOURCE, DEBUG_USART_TX_AF);
	GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_SOURCE, DEBUG_USART_RX_AF);

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

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN | DEBUG_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	// /* 中断使用命令行 */
	NVIC_InitTypeDef NVIC_InitStre;
	NVIC_InitStre.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStre.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStre.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStre.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStre);//初始化

	/* 空闲中断,接收到一连串数据触发一次 */
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	USART_ClearITPendingBit(USART3, USART_IT_IDLE);

	// todo 注意 如果是使能dma一定要加这个
	USART_DMACmd(DEBUG_USART, USART_DMAReq_Rx, ENABLE);

    return;
}


/**
 * @Description: 串口获取
 * @author: lkc
 * @Date: 2023-02-17 23:01:32
 * @return {*}
 */
// uint16_t Bsp_Usart_GetChar(void)
// {
//     static char ch = 0;

// 	//ch = (char)USART_ReceiveData(DEBUG_USART);
// 	if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) != RESET)
// 	{
// 		ch = (char)USART_ReceiveData(DEBUG_USART);
// 	}
// 	else
// 	{
// 		if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_ORE) != RESET)
// 		{
// 			USART_ClearFlag(DEBUG_USART, USART_FLAG_TC);
// 		}
// 		ch = 0;
// 	}

//     return ch;
// }

/**
 * @Description: 串口获取
 * @author: lkc
 * @Date: 2023-02-17 23:01:32
 * @return {*}
 */
void Bsp_Usart_PutString(const char *str)
{
	/* 直到字符串结束 */
	while (*str != '\0')
	{ 
		// 换行
		if (*str == '\n')	
		{
			USART_SendData(DEBUG_USART, '\r');
			while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
		}
		USART_SendData(DEBUG_USART, *str++);
	  	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);	
	}		
    return;
}

/**
  * @brief  输出到终端一个字符
  * @param  c: 输出字符
  * @retval None
  */
void Bsp_Usart_PutChar(uint8_t c)
{
	USART_SendData(DEBUG_USART, c);
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET)
	{

	}

	return;
}

/**
  * @brief  Test to see if a key has been pressed on the HyperTerminal
  * @param  key: The key pressed
  * @retval 1: Correct
  *         0: Error
  */
uint32_t Bsp_Usart_GetChar(uint8_t* key)
{
	/* 当存在数据的时候 */
	if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) != RESET)
	{
		/* 取出数据 */
		*key = (uint8_t)DEBUG_USART->DR;
		//*key = (uint8_t)USART_ReceiveData(DEBUG_USART);
		return 1;
	}
	else
	{
		return 0;
	}
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
	USART_SendData(DEBUG_USART, (u8)ch);
	while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET);

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
void Bsp_Usart_U3Handler(void)
{
    uint8_t data = 0;
    if (RESET != USART_GetITStatus(USART3, USART_IT_IDLE))
    {
		/* 个人认为是清空缓冲,取出数据 */
		USART3->SR;
		USART3->DR;

		/* 清空空闲中断 */
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);
		/* 关闭dma */
		DMA_Cmd(DMA1_Stream1, DISABLE);
		//本帧数据长度=DMA准备的接收数据长度-DMA已接收数据长度
		ulRxLen = USART3_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1); 

		/* 此时知道接收的长度以及Buf,直接传一个队列 */
		osMessageQueuePut(queueCmdHandle, &ulRxLen, NULL, 0);
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF4 | DMA_FLAG_FEIF4 | DMA_FLAG_DMEIF4 |
				DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4);//清除DMA2_Steam7传输完成标志
		while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE){}		
		DMA_SetCurrDataCounter(DMA1_Stream1, USART3_RX_BUF_SIZE);
		DMA_Cmd(DMA1_Stream1, ENABLE);
	}
}

// xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
// {
// 	xComPortHandle xReturn;

// 	/* Create the queues used to hold Rx/Tx characters. */
// 	xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
// 	xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	
// 	/* If the queue/semaphore was created correctly then setup the serial port
// 	hardware. */
// 	if( ( xRxedChars != serINVALID_QUEUE ) && ( xCharsForTx != serINVALID_QUEUE ) )
// 	{
// 		Bsp_Usart_DebugInit(ulWantedBaud);
// 		Bsp_Dma_Usart3Rx();
// 	}
// 	else
// 	{
// 		xReturn = ( xComPortHandle ) 0;
// 	}

// 	/* This demo file only supports a single port but we have to return
// 	something to comply with the standard demo header file. */
// 	return xReturn;
// }

#ifdef __cplusplus
}
#endif
