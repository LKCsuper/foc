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
#define USART_TX_WRITE_CHUNK_MAX          (128U)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
 * 调试串口发送队列。容量为 2 的幂，放在普通 SRAM：STM32F405 的 DMA 不能访问 CCM。
 * 多个任务写入时用短临界区串起来；DMA 只读 peek_contig 返回的连续段，完成后 drop。
 */
STATIC UCHAR sgucUsartTxBuf[USART_TX_BUF_SIZE];
STATIC ringbuf_t s_usart_tx;

/* 当前 DMA 正在发送的连续字节数，完成中断中用它 drop。 */
STATIC volatile ULONG sgulUsartTxDmaLen = 0U;
STATIC volatile ULONG sgulUsartTxBytes = 0U;
STATIC volatile ULONG sgulUsartTxDrops = 0U;
/* true 表示 DMA 正在使用 sgucUsartTxBuf 中的一段数据。 */
STATIC volatile bool sgbUsartTxDmaBusy = false;
/* DMA 初始化前，启动日志仍使用轮询发送；初始化后自动切换为异步 DMA。 */
STATIC volatile bool sgbUsartTxDmaReady = false;
/* Private function prototypes -----------------------------------------------*/
STATIC VOID Bsp_Usart_PollingPutChar(UCHAR ucData);
STATIC VOID Bsp_Usart_TxDmaStartLocked(VOID);
STATIC VOID Bsp_Usart_TxDmaCompleteLocked(VOID);
STATIC VOID Bsp_Usart_TxDmaPollLocked(VOID);
STATIC bool Bsp_Usart_TxQueueByte(UCHAR ucData);
STATIC VOID Bsp_Usart_YieldToIsr(VOID);
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  任务里让出一次 tick，让 FOC 中断按原节拍跑。
 * @note   调度器未启动或当前在中断里时不延时。tick 为 10 kHz，一次是 0.1 ms。
 */
STATIC VOID Bsp_Usart_YieldToIsr(VOID)
{
	if ((__get_IPSR() != 0U) || (osKernelGetState() != osKernelRunning)) {
		return;
	}
	(void)osDelay(1U);
}

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
 * 环形缓冲在物理内存末尾会回绕，而一次 DMA 只能发送一段连续内存。
 * peek_contig 只给出从 out 到环尾的那一段，drop 之后再启动下一段。
 */
STATIC VOID Bsp_Usart_TxDmaStartLocked(VOID)
{
	const uint8_t *pucData;
	uint32_t ulContinuousLen;

	if (!sgbUsartTxDmaReady || sgbUsartTxDmaBusy) {
		return;
	}

	/* 正常模式 DMA 在计数归零后会自动关闭；只有关闭状态才能改地址和长度。 */
	if (DMA_GetCmdStatus(DEBUG_USART_TX_DMA_STREAM) != DISABLE) {
		return;
	}

	ulContinuousLen = ringbuf_peek_contig(&s_usart_tx, &pucData);
	if ((ulContinuousLen == 0U) || (pucData == NULL)) {
		return;
	}

	sgulUsartTxDmaLen = ulContinuousLen;
	sgbUsartTxDmaBusy = true;

	/* 清除上一次发送标志，再设置本次连续内存块的首地址和长度。 */
	DMA_ClearFlag(DEBUG_USART_TX_DMA_STREAM, DEBUG_USART_TX_DMA_FLAGS);
	DEBUG_USART_TX_DMA_STREAM->M0AR = (uint32_t)pucData;
	DMA_SetCurrDataCounter(DEBUG_USART_TX_DMA_STREAM, ulContinuousLen);

	/* 保证缓冲区内容和 DMA 寄存器写入完成后，才真正使能 DMA。 */
	__DMB();
	DMA_Cmd(DEBUG_USART_TX_DMA_STREAM, ENABLE);
}

/**
 * @brief  处理一次 TX DMA 完成：释放已发送段并启动下一段。
 * @note   调用前必须关中断。FOC 高负载时 UART DMA 中断可能迟迟不到，任务侧也会调用。
 */
STATIC VOID Bsp_Usart_TxDmaCompleteLocked(VOID)
{
	volatile uint32_t ulTimeout = 1000U;

	(void)ringbuf_drop(&s_usart_tx, sgulUsartTxDmaLen);
	sgulUsartTxDmaLen = 0U;
	sgbUsartTxDmaBusy = false;

	/* 普通模式硬件会清 EN；任务侧补轮询时偶发 EN 仍为 1，不关掉就无法启动下一段。 */
	DMA_Cmd(DEBUG_USART_TX_DMA_STREAM, DISABLE);
	while ((DMA_GetCmdStatus(DEBUG_USART_TX_DMA_STREAM) != DISABLE) &&
		   (ulTimeout > 0U)) {
		ulTimeout--;
	}

	Bsp_Usart_TxDmaStartLocked();
}

/**
 * @brief  任务侧补看 TX DMA 完成标志，并在队列非空时尝试启动下一段。
 * @note   调用前必须关中断。用 Flag 而不是 ITStatus：后者还要求 TCIE，任务轮询会漏完成。
 */
STATIC VOID Bsp_Usart_TxDmaPollLocked(VOID)
{
	if (sgbUsartTxDmaBusy &&
	    (DMA_GetFlagStatus(DEBUG_USART_TX_DMA_STREAM, DEBUG_USART_TX_DMA_TC_FLAG) != RESET)) {
		DMA_ClearFlag(DEBUG_USART_TX_DMA_STREAM, DEBUG_USART_TX_DMA_FLAGS);
		DMA_ClearITPendingBit(DEBUG_USART_TX_DMA_STREAM, DEBUG_USART_TX_DMA_TC_IT);
		Bsp_Usart_TxDmaCompleteLocked();
		return;
	}

	if (!sgbUsartTxDmaBusy) {
		Bsp_Usart_TxDmaStartLocked();
	}
}

/**
 * @brief  将一个字节放入发送环形缓冲区。
 * @retval true  入队成功。
 * @retval false 当前在中断/临界区中且缓冲区已满，为避免死锁而放弃该字节。
 *
 * 普通任务遇到缓冲区满时可以等待；不能只空转等 UART DMA 中断，因为其优先级
 * 低于 FOC，约 95% 负载时中断长期跑不到，olstatus 会把命令任务卡死。
 */
STATIC bool Bsp_Usart_TxQueueByte(UCHAR ucData)
{
	uint32_t ulPrimask;

	while (true) {
		ulPrimask = __get_PRIMASK();
		__disable_irq();

		Bsp_Usart_TxDmaPollLocked();
		if (ringbuf_put_byte(&s_usart_tx, ucData) != 0U) {
			sgulUsartTxBytes++;
			Bsp_Usart_TxDmaStartLocked();

			if (0U == ulPrimask) {
				__enable_irq();
			}
			return true;
		}

		/* 中断或外部临界区中绝不等待，避免 DMA 完成中断无法执行。 */
		if ((__get_IPSR() != 0U) || (ulPrimask != 0U)) {
			sgulUsartTxDrops++;
			if (0U == ulPrimask) {
				__enable_irq();
			}
			return false;
		}

		if (0U == ulPrimask) {
			__enable_irq();
		}
		/* 队列已满。空转关中断会把 15 kHz 电流环挤偏，改成等 DMA 腾出空间。 */
		Bsp_Usart_YieldToIsr();
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
	stNvicInit.NVIC_IRQChannel = DEBUG_USART_TX_DMA_IRQn;
	stNvicInit.NVIC_IRQChannelPreemptionPriority = IRQ_DEBUG_TX;
	stNvicInit.NVIC_IRQChannelSubPriority = 0U;
	stNvicInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&stNvicInit);

	ulPrimask = __get_PRIMASK();
	__disable_irq();
	if (ringbuf_init(&s_usart_tx, sgucUsartTxBuf, USART_TX_BUF_SIZE) == 0U) {
		if (0U == ulPrimask) {
			__enable_irq();
		}
		return;
	}
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

	/* 先关中断再看 TC：任务侧可能已经把同一完成标志清掉并启动了下一段。 */
	ulPrimask = __get_PRIMASK();
	__disable_irq();
	if (DMA_GetITStatus(DEBUG_USART_TX_DMA_STREAM, DEBUG_USART_TX_DMA_TC_IT) != RESET) {
		DMA_ClearITPendingBit(DEBUG_USART_TX_DMA_STREAM, DEBUG_USART_TX_DMA_TC_IT);
		Bsp_Usart_TxDmaCompleteLocked();
	}
	if (0U == ulPrimask) {
		__enable_irq();
	}
}

/**
 * @brief  在命令任务空闲时补轮询 TX DMA，不依赖完成中断。
 */
VOID Bsp_Usart_TxDmaPoll(VOID)
{
	uint32_t ulPrimask;

	if (!sgbUsartTxDmaReady) {
		return;
	}

	ulPrimask = __get_PRIMASK();
	__disable_irq();
	Bsp_Usart_TxDmaPollLocked();
	if (0U == ulPrimask) {
		__enable_irq();
	}
}

/**
 * @description: 已进入 DMA 发送队列的字节数
 * @return {ULONG} 累计入队字节数
 */
ULONG Bsp_Usart_GetTxBytes(VOID)
{
	return sgulUsartTxBytes;
}

/**
 * @description: 队列满时从中断或临界区丢弃的字节数
 * @return {ULONG} 累计丢弃字节数
 */
ULONG Bsp_Usart_GetTxDrops(VOID)
{
	return sgulUsartTxDrops;
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
 * @brief  把一整块字节写入 UART DMA 环形队列。
 * @detail 任务上下文会等到全部入队；ISR/外部临界区只写当前空闲部分。
 *         每块只进入一次短临界区，避免 printf/fputc 逐字节反复关中断。
 * @return 已入队字节数。
 */
ULONG Bsp_Usart_Write(const UCHAR *data, ULONG len)
{
	ULONG written = 0U;

	if (data == NULL || len == 0U) {
		return 0U;
	}
	if (!sgbUsartTxDmaReady) {
		while (written < len) {
			Bsp_Usart_PollingPutChar(data[written++]);
		}
		return written;
	}

	while (written < len) {
		ULONG free_len;
		ULONG continuous_len;
		uint32_t ulPrimask = __get_PRIMASK();

		__disable_irq();
		Bsp_Usart_TxDmaPollLocked();
		free_len = ringbuf_free(&s_usart_tx);
		continuous_len = len - written;
		if (continuous_len > free_len) {
			continuous_len = free_len;
		}
		if (continuous_len > USART_TX_WRITE_CHUNK_MAX) {
			continuous_len = USART_TX_WRITE_CHUNK_MAX;
		}
		if (continuous_len > 0U) {
			continuous_len = ringbuf_write(&s_usart_tx, &data[written], continuous_len);
			if (continuous_len > 0U) {
				written += continuous_len;
				sgulUsartTxBytes += continuous_len;
				Bsp_Usart_TxDmaStartLocked();
			}
		}
		if ((continuous_len == 0U) &&
		    ((__get_IPSR() != 0U) || (ulPrimask != 0U))) {
			sgulUsartTxDrops += (len - written);
		}
		if (0U == ulPrimask) {
			__enable_irq();
		}

		if ((continuous_len == 0U) &&
		    ((__get_IPSR() != 0U) || (ulPrimask != 0U))) {
			break;
		}
		if (continuous_len == 0U) {
			/* 队列已满。空转关中断会把 15 kHz 电流环挤偏，改成等 DMA 腾出空间。 */
			Bsp_Usart_YieldToIsr();
		}
	}
	return written;
}

/**
 * @brief 等待发送队列清空，并等待最后一个字符完全从 TX 引脚发出。
 * @note  等待期间轮询 DMA 完成标志，不依赖 UART DMA 中断。olstart / olsnap
 *        在 FOC 高负载下必须调用，否则回绕后的下一段会永远留在环形缓冲区里。
 */
VOID Bsp_Usart_Flush(VOID)
{
	bool bQueueEmpty;
	uint32_t ulPrimask;
	uint32_t ulSpins = 0U;
	uint32_t ulTcWait = 20000U;

	if (sgbUsartTxDmaReady) {
		do {
			ulPrimask = __get_PRIMASK();
			__disable_irq();
			Bsp_Usart_TxDmaPollLocked();
			bQueueEmpty = (ringbuf_is_empty(&s_usart_tx) != 0U) &&
						  !sgbUsartTxDmaBusy;
			if (0U == ulPrimask) {
				__enable_irq();
			}
			if (bQueueEmpty) {
				break;
			}
			ulSpins++;
			/* 2 Mbaud 下整页大约 15 ms。让出 tick，避免空转关中断拖住电流环。 */
			Bsp_Usart_YieldToIsr();
			if (ulSpins >= 200U) {
				break;
			}
		} while (!bQueueEmpty);
	}

	while ((USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET) && (ulTcWait > 0U)) {
		ulTcWait--;
	}
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
