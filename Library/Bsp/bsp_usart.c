/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-09-23 20:03:13
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART_TX_BUF_MASK                 (USART_TX_BUF_SIZE - 1U)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
 * 调试串口发送环形缓冲区。
 *
 * head 指向“下一次写入”的位置，tail 指向“下一次发送”的位置。
 * head == tail 表示缓冲区为空；为了区分“空”和“满”，始终保留一个空字节，
 * 所以实际最多存放 USART_TX_BUF_SIZE - 1 个字节。
 */
STATIC UCHAR sgucUsartTxBuf[USART_TX_BUF_SIZE];
STATIC volatile ULONG sgulUsartTxHead = 0U;
STATIC volatile ULONG sgulUsartTxTail = 0U;

/* 当前 DMA 正在发送的连续字节数，完成中断中用它推进 tail。 */
STATIC volatile ULONG sgulUsartTxDmaLen = 0U;
/* true 表示 DMA 正在使用 sgucUsartTxBuf 中的一段数据。 */
STATIC volatile bool sgbUsartTxDmaBusy = false;
/* DMA 初始化前，启动日志仍使用轮询发送；初始化后自动切换为异步 DMA。 */
STATIC volatile bool sgbUsartTxDmaReady = false;
/* Private function prototypes -----------------------------------------------*/
STATIC VOID Bsp_Usart_PollingPutChar(UCHAR ucData);
STATIC VOID Bsp_Usart_TxDmaStartLocked(VOID);
STATIC bool Bsp_Usart_TxQueueByte(UCHAR ucData);
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  使用轮询方式发送一个字节。
 * @note   只在发送 DMA 尚未初始化时使用，例如 Bsp_Init() 的早期启动日志。
 *         DMA 就绪后，普通 printf 不再走这里。
 */
STATIC VOID Bsp_Usart_PollingPutChar(UCHAR ucData)
{
	USART_SendData(DEBUG_USART, ucData);
	IS_TIME_OUT(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);
}

/**
 * @brief  启动环形缓冲区中的下一段 DMA 发送。
 * @note   调用本函数前必须先关闭全局中断，防止 head/tail/busy 被并发修改。
 *
 * 环形缓冲区在物理内存末尾可能发生回绕，而一次 DMA 只能发送一段连续内存：
 *
 *   情况 1：tail < head，发送 [tail, head)
 *   情况 2：tail > head，先发送 [tail, 缓冲区末尾)，完成后再发送 [0, head)
 */
STATIC VOID Bsp_Usart_TxDmaStartLocked(VOID)
{
	ULONG ulContinuousLen;

	if (!sgbUsartTxDmaReady || sgbUsartTxDmaBusy ||
		(sgulUsartTxHead == sgulUsartTxTail)) {
		return;
	}

	/* 正常模式 DMA 在计数归零后会自动关闭；只有关闭状态才能改地址和长度。 */
	if (DMA_GetCmdStatus(DEBUG_USART_TX_DMA_STREAM) != DISABLE) {
		return;
	}

	if (sgulUsartTxHead > sgulUsartTxTail) {
		ulContinuousLen = sgulUsartTxHead - sgulUsartTxTail;
	} else {
		ulContinuousLen = USART_TX_BUF_SIZE - sgulUsartTxTail;
	}

	sgulUsartTxDmaLen = ulContinuousLen;
	sgbUsartTxDmaBusy = true;

	/* 清除上一次发送标志，再设置本次连续内存块的首地址和长度。 */
	DMA_ClearFlag(DEBUG_USART_TX_DMA_STREAM, DEBUG_USART_TX_DMA_FLAGS);
	DEBUG_USART_TX_DMA_STREAM->M0AR =
		(uint32_t)&sgucUsartTxBuf[sgulUsartTxTail];
	DMA_SetCurrDataCounter(DEBUG_USART_TX_DMA_STREAM, ulContinuousLen);

	/* 保证缓冲区内容和 DMA 寄存器写入完成后，才真正使能 DMA。 */
	__DMB();
	DMA_Cmd(DEBUG_USART_TX_DMA_STREAM, ENABLE);
}

/**
 * @brief  将一个字节放入发送环形缓冲区。
 * @retval true  入队成功。
 * @retval false 当前在中断/临界区中且缓冲区已满，为避免死锁而放弃该字节。
 *
 * 普通任务遇到缓冲区满时可以等待，因为 DMA 和 DMA 完成中断仍会继续运行；
 * 中断上下文或已经关闭中断的临界区不能等待，否则 DMA 完成中断无法推进 tail，
 * 会形成永久死锁。因此这些特殊上下文在满队列时直接返回 false。
 */
STATIC bool Bsp_Usart_TxQueueByte(UCHAR ucData)
{
	ULONG ulNextHead;
	uint32_t ulPrimask;

	while (true) {
		ulPrimask = __get_PRIMASK();
		__disable_irq();

		ulNextHead = (sgulUsartTxHead + 1U) & USART_TX_BUF_MASK;
		if (ulNextHead != sgulUsartTxTail) {
			sgucUsartTxBuf[sgulUsartTxHead] = ucData;
			sgulUsartTxHead = ulNextHead;
			Bsp_Usart_TxDmaStartLocked();

			if (0U == ulPrimask) {
				__enable_irq();
			}
			return true;
		}

		if (0U == ulPrimask) {
			__enable_irq();
		}

		/* 中断或外部临界区中绝不等待，避免 DMA 完成中断无法执行。 */
		if ((__get_IPSR() != 0U) || (ulPrimask != 0U)) {
			return false;
		}

		/* 普通任务仅在超过 2KB 的连续输出时短暂等待 DMA 腾出一个字节。 */
		__NOP();
	}
}
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
	PRINTF("Bsp Uart Init Success\r\n");

    return;
}

/**
 * @brief  初始化调试串口发送 DMA。
 * @detail UART4_TX 使用 DMA1 Stream4 / Channel4。DMA 采用普通模式，每次只发送
 *         环形缓冲区中的一段连续数据；一段完成后由中断自动接着发送下一段。
 *
 * 初始化发生在 Bsp_Usart_DebugInit() 之后，因此更早的启动日志先用轮询方式输出。
 * 切换到 DMA 前等待最后一个轮询字符完全离开 TX 引脚，避免两种发送方式重叠。
 */
VOID Bsp_Usart_TxDmaInit(VOID)
{
	DMA_InitTypeDef stDmaInit;
	NVIC_InitTypeDef stNvicInit;
	volatile uint32_t ulTimeout = 100000U;
	uint32_t ulPrimask;

	if (sgbUsartTxDmaReady) {
		return;
	}

	IS_TIME_OUT(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET);
	DEBUG_USART_TX_DMA_CLK_EN();

	DMA_Cmd(DEBUG_USART_TX_DMA_STREAM, DISABLE);
	while ((DMA_GetCmdStatus(DEBUG_USART_TX_DMA_STREAM) != DISABLE) &&
		   (ulTimeout > 0U)) {
		ulTimeout--;
		/* 只有流关闭后，下面的 DMA 配置寄存器才允许修改。 */
	}
	if (DMA_GetCmdStatus(DEBUG_USART_TX_DMA_STREAM) != DISABLE) {
		/*
		 * 关闭失败时保持 sgbUsartTxDmaReady=false，后续输出会继续使用安全的
		 * 轮询方式；绝不能在 DMA 仍运行时强行改写它的配置寄存器。
		 */
		return;
	}

	DMA_DeInit(DEBUG_USART_TX_DMA_STREAM);
	DMA_StructInit(&stDmaInit);
	stDmaInit.DMA_Channel = DEBUG_USART_TX_DMA_CHANNEL;
	stDmaInit.DMA_PeripheralBaseAddr = (uint32_t)&DEBUG_USART->DR;
	stDmaInit.DMA_Memory0BaseAddr = (uint32_t)sgucUsartTxBuf;
	stDmaInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	/* 初始化时给一个合法占位值，真正发送前会改成连续数据段的长度。 */
	stDmaInit.DMA_BufferSize = 1U;
	stDmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	stDmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	stDmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	stDmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	stDmaInit.DMA_Mode = DMA_Mode_Normal;
	/* UART DMA 与 ADC FOC DMA 分属 DMA1/DMA2；中等优先级已足够且不会抢占接收。 */
	stDmaInit.DMA_Priority = DMA_Priority_Medium;
	stDmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
	stDmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	stDmaInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	stDmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DEBUG_USART_TX_DMA_STREAM, &stDmaInit);

	DMA_ClearFlag(DEBUG_USART_TX_DMA_STREAM, DEBUG_USART_TX_DMA_FLAGS);
	DMA_ITConfig(DEBUG_USART_TX_DMA_STREAM, DMA_IT_TC, ENABLE);

	/* 发送 DMA 中断只管理缓冲区，不调用 RTOS API，优先级低于 FOC DMA 中断。 */
	stNvicInit.NVIC_IRQChannel = DEBUG_USART_TX_DMA_IRQn;
	stNvicInit.NVIC_IRQChannelPreemptionPriority = IRQ_DEBUG_IDE;
	stNvicInit.NVIC_IRQChannelSubPriority = 0U;
	stNvicInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&stNvicInit);

	ulPrimask = __get_PRIMASK();
	__disable_irq();
	sgulUsartTxHead = 0U;
	sgulUsartTxTail = 0U;
	sgulUsartTxDmaLen = 0U;
	sgbUsartTxDmaBusy = false;
	sgbUsartTxDmaReady = true;
	USART_DMACmd(DEBUG_USART, USART_DMAReq_Tx, ENABLE);
	if (0U == ulPrimask) {
		__enable_irq();
	}
}

/**
 * @brief  调试串口发送 DMA 完成中断处理。
 *
 * DMA 完成只代表最后一个字节已经写入 USART->DR，不一定已经离开 TX 引脚。
 * 这里可以立即释放软件缓冲区并启动下一段；只有 Bsp_Usart_Flush() 才需要额外
 * 等待 USART_FLAG_TC，普通 printf 不做这个等待。
 */
VOID Bsp_Usart_TxDmaHandler(VOID)
{
	uint32_t ulPrimask;

	if (RESET == DMA_GetITStatus(DEBUG_USART_TX_DMA_STREAM,
								  DEBUG_USART_TX_DMA_TC_IT)) {
		return;
	}

	DMA_ClearITPendingBit(DEBUG_USART_TX_DMA_STREAM,
						  DEBUG_USART_TX_DMA_TC_IT);

	ulPrimask = __get_PRIMASK();
	__disable_irq();

	/* 释放刚刚完成的连续数据段，随后尝试发送环形缓冲区中的下一段。 */
	sgulUsartTxTail = (sgulUsartTxTail + sgulUsartTxDmaLen) &
					 USART_TX_BUF_MASK;
	sgulUsartTxDmaLen = 0U;
	sgbUsartTxDmaBusy = false;
	Bsp_Usart_TxDmaStartLocked();

	if (0U == ulPrimask) {
		__enable_irq();
	}
}

/**
 * @brief  查询调试串口是否已经切换到异步 DMA 发送。
 * @note   olstatus 会显示这个结果，便于区分“DMA 没启动”和“FOC 占用过高”。
 */
bool Bsp_Usart_TxDmaIsReady(VOID)
{
	return sgbUsartTxDmaReady;
}

/**
 * @Description: 串口输出字符串
 * @author: lkc
 * @Date: 2023-02-17 23:01:32
 * @return {*}
 */
void Bsp_Usart_PutString(const CHAR *cStr)
{
	/* 直到字符串结束。DMA 未初始化时 PutChar 会自动退回轮询发送。 */
	while (*cStr != '\0')
	{
		Bsp_Usart_PutChar((UCHAR)*cStr++);
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
		/* Windows 风格终端需要在 LF 前补 CR。 */
#ifdef ENTER_PRI
		if (*cStr == '\n')	
		{
			Bsp_Usart_PutChar((UCHAR)ENTER_PRI);
		}
#endif
		Bsp_Usart_PutChar(*cStr++);
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
	if (!sgbUsartTxDmaReady) {
		Bsp_Usart_PollingPutChar(cKey);
		return;
	}

	/* 普通任务几乎总能成功；只有中断/临界区遇到满队列时才会安全丢弃。 */
	(void)Bsp_Usart_TxQueueByte(cKey);

	return;
}

/**
 * @brief 等待发送队列清空，并等待最后一个字符完全从 TX 引脚发出。
 * @note  普通 printf 和 Shell 返回值不调用本函数，它们依靠同一个 FIFO 队列
 *        保证顺序。只有关机、切换串口参数等必须确认物理发送完成的场景才调用。
 */
VOID Bsp_Usart_Flush(VOID)
{
	bool bQueueEmpty;
	uint32_t ulPrimask;

	if (sgbUsartTxDmaReady) {
		do {
			ulPrimask = __get_PRIMASK();
			__disable_irq();
			bQueueEmpty = (sgulUsartTxHead == sgulUsartTxTail) &&
						  !sgbUsartTxDmaBusy;
			if (0U == ulPrimask) {
				__enable_irq();
			}
		} while (!bQueueEmpty);
	}

	IS_TIME_OUT(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET);
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
	/* f 只是 printf 传入的标准输出句柄，本驱动并不需要使用它。 */
	(void)f;

#ifdef ENTER_PRI
	/*
	 * 终端需要 CR+LF 才能换行，因此在 LF 前补一个 CR。
	 * PutChar 现在只把字符放进软件 FIFO，DMA 会在后台依次发出 CR 和 LF。
	 */
	if ('\n' == ch)
	{
		Bsp_Usart_PutChar((UCHAR)ENTER_PRI);
	}
#endif

	/* DMA 初始化前自动使用轮询；初始化完成后仅入队，不再逐字符忙等。 */
	Bsp_Usart_PutChar((UCHAR)ch);

    return ch;
}

#ifdef __cplusplus
}
#endif
