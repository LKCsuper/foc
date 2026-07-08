/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-26 20:50:09
 * @FilePath: \foc\Library\Bsp\bsp_dma.c
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
/* 串口接收中断 */
UCHAR gucURxBuf[USART_RX_BUF_SIZE] = {0};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: DMA2 Stream4 初始化（ADC 采样数据自动搬运）
 * @detail:
 *   配置 DMA2 的 Stream4，将 ADC 转换结果从 ADC 通用数据寄存器自动搬运到内存数组：
 *   - 数据流：ADC->CDR（外设）→ ADC_Value[]（内存），半字（16bit）单位
 *   - 通道：DMA_Channel_0，对应 ADC1 的 DMA 请求
 *   - 模式：循环模式（Circular），每轮 ADC 扫描完成后自动重新开始，不断刷新采样数据
 *   - 缓冲区大小：HW_ADC_CHANNELS（ADC 通道数），每轮传输所有通道的转换结果
 *   - 外设地址固定（CDR 寄存器只有一个地址），内存地址递增（填充数组各元素）
 *   - 高优先级，确保 ADC 数据及时搬运不被其他 DMA 请求阻塞
 *   - FIFO 禁用，单次突发传输（直接模式，每个 DMA 请求传输一个半字）
 *
 *   对应中断配置见 Bsp_Dma2_NVIC()：
 *   - 前三个 ADC 通道为电流采样时使用半传输中断（HT），提前开始 FOC 计算
 *   - 否则使用传输完成中断（TC）
 * @return {VOID}
 * @author: lkc
 */
STATIC VOID Bsp_DMA2_Init(VOID)
{
	/* 使能 DMA2 时钟（AHB1总线） */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	/* modify:codex 原因：重新初始化 ADC DMA 前先关闭 Stream4 并清旧标志，避免上次运行残留的 TC/HT/错误标志误触发 FOC 中断。 */
	DMA_DeInit(DMA2_Stream4);
	volatile uint32_t dma_timeout = 100000;
	while (DMA_GetCmdStatus(DMA2_Stream4) != DISABLE && --dma_timeout) {}
	DMA_ClearFlag(DMA2_Stream4, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_TEIF4 |
			DMA_FLAG_DMEIF4 | DMA_FLAG_FEIF4);

	DMA_InitTypeDef DMA_InitStructure;
	/* 填充默认值，避免栈上随机值被误写入寄存器 */
	DMA_StructInit(&DMA_InitStructure);
	/* 通道选择：通道0 → 对应 ADC1 的 DMA 请求 */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	/* 内存基地址：ADC 采样结果数组 ADC_Value[] */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value;
	/* 外设基地址：ADC 通用规则数据寄存器（CDR），所有通道转换结果在此 */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
	/* 传输方向：外设 → 内存（ADC 读到数组） */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	/* 缓冲区大小：等于 ADC 通道数，每轮传输所有通道的转换结果 */
	DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS;
	/* 外设地址不递增：CDR 寄存器地址固定，每次读取同一个地址 */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	/* 内存地址递增：依次填入 ADC_Value[0], ADC_Value[1], ... */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	/* 外设数据宽度：半字（16bit），ADC 转换精度 12bit */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	/* 内存数据宽度：半字（16bit），与外设一致 */
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	/* 循环模式：ADC 持续扫描，DMA 自动循环搬运，无需软件重新配置 */
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	/* 优先级：高，ADC 采样数据需要及时搬运避免覆盖 */
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	/* FIFO 模式：禁用，使用直接模式（每个请求立即触发一次传输） */
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	/* FIFO 阈值：仅 FIFO 使能时有效，此处无实际作用 */
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	/* 内存突发传输：单次（直接模式下一拍一个数据） */
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	/* 外设突发传输：单次 */
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	/* modify:codex 原因：先只写入 DMA 配置，待中断源和 NVIC 配好后再使能 Stream4，避免初始化窗口内漏触发或误触发。 */
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);

	return;
}

/**
 * @description: DMA2 Stream4 中断配置（ADC 采样完成 / 半完成中断）
 * @detail:
 *   配置 DMA2 Stream4 的 NVIC 中断，用于在 ADC DMA 搬运完成后触发 FOC 算法处理：
 *
 *   中断策略（关键优化）：
 *   - 当 ADC 前三个通道为电流采样（ADC_IND_CURR1/2/3 < 3）时，使用半传输中断（HT）
 *     原因：DMA 搬运一半数据时，三相电流 + 电压数据已经到达内存，可以提前开始 FOC 计算
 *     优势：在下一个零矢量到来之前有更多 CPU 周期完成计算并更新定时器，支持更高的 PWM 频率
 *   - 否则使用传输完成中断（TC）：等待所有 ADC 通道数据到齐后再处理
 *
 *   中断优先级：IRQ_DMA2_STREAM2，子优先级 0（最高），确保 ADC 数据及时处理
 * @return {VOID}
 * @author: lkc
 */
STATIC VOID Bsp_Dma2_NVIC(VOID)
{
	/* NVIC 中断初始化结构体 */
	NVIC_InitTypeDef NVIC_InitStre;
	/* 中断通道：DMA2 Stream4，对应 ADC1 DMA 请求 */
	NVIC_InitStre.NVIC_IRQChannel = DMA2_Stream4_IRQn;
	/* modify:codex 原因：该中断实际使用 DMA2 Stream4，宏名改为 IRQ_DMA2_STREAM4，避免和 Stream2 混淆。 */
    NVIC_InitStre.NVIC_IRQChannelPreemptionPriority = IRQ_DMA2_STREAM4;
	/* 子优先级：0（最高），优先处理 ADC 数据 */
    NVIC_InitStre.NVIC_IRQChannelSubPriority = 0x0;
	/* 使能中断通道 */
    NVIC_InitStre.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStre);

	/* 根据 ADC 通道排列选择中断触发点：
	 * 前三个通道是电流 → 半传输中断（HT）：DMA 搬运一半时电流电压已就绪，可提前开始 FOC
	 * 否则 → 传输完成中断（TC）：所有通道数据到齐再处理 */
#if ADC_IND_CURR1 < 3 && ADC_IND_CURR2 < 3 && ADC_IND_CURR3 < 3
	DMA_ITConfig(DMA2_Stream4, DMA_IT_HT, ENABLE);
#else
	DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
#endif
	/* modify:codex 原因：中断源配置完成后再次清 Stream4 标志，再使能 DMA，防止 pending 标志导致一开中断就进入 Motor_DmaInt。 */
	DMA_ClearFlag(DMA2_Stream4, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_TEIF4 |
			DMA_FLAG_DMEIF4 | DMA_FLAG_FEIF4);
	DMA_Cmd(DMA2_Stream4, ENABLE);

	return;
}

/**
 * @description: ADC DMA 采样初始化入口
 * @detail:
 *   调用 DMA2 初始化及中断配置，完成 ADC 采样数据的 DMA 自动搬运链路：
 *   Bsp_DMA2_Init() → 配置 DMA2 Stream4 循环搬运 ADC->CDR → ADC_Value[]
 *   Bsp_Dma2_NVIC() → 配置传输完成/半完成中断，触发 FOC 电流环计算
 * @return {VOID}
 * @author: lkc
 */
VOID Bsp_Dma_AdcSample(VOID)
{
	/* 初始化 DMA2 Stream4：ADC 数据自动搬运 */
	Bsp_DMA2_Init();
	/* 配置 DMA2 Stream4 中断：采样完成后触发 FOC 算法 */
	Bsp_Dma2_NVIC();

	return;
}

/**
 * @description: DMA1 Stream1 初始化（串口接收 DMA）
 * @detail:
 *   配置 DMA1 Stream1 通道4，将 USART 接收数据寄存器 DR 的内容自动搬运到接收缓冲区：
 *   - 数据流：DEBUG_USART->DR（外设）→ gucURxBuf[]（内存），字节单位
 *   - 通道：DMA_Channel_4，对应 USART1_RX 的 DMA 请求
 *   - 模式：循环模式（必须），否则接收一包数据后 DMA 自动停止
 *   - 缓冲区大小：32 字节，外设地址固定（DR 寄存器），内存地址递增
 *   - FIFO 禁用，单次突发传输，高优先级确保串口数据不丢失
 *
 *   注意：本函数仅初始化 DMA 结构体，不使能 DMA 和 USART DMA 请求
 *   （由 Bsp_Dma_UsartRx 完成使能）。
 * @return {VOID}
 * @author: lkc
 */
VOID Bsp_Dma1_Init(VOID)
{
	/* 复位 DMA1 Stream1 至默认状态 */
	DMA_DeInit(DEBUG_USART_RX_DMA_STREAM);
	/* 使能 DMA1 时钟（AHB1总线） */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_StructInit(&DMA_InitStructure);
	/* 等待上一次 DMA 配置彻底关闭，超时约 1ms 后强制退出，防止死循环 */
	volatile uint32_t dma_timeout = 100000;
	while (DMA_GetCmdStatus(DEBUG_USART_RX_DMA_STREAM) != DISABLE && --dma_timeout) {}
	/* 通道4 → 对应 USART1_RX DMA 请求 */
	DMA_InitStructure.DMA_Channel = DEBUG_USART_RX_DMA_CHANNEL;
	/* 外设地址：串口数据寄存器 DR */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(DEBUG_USART->DR);
	/* 内存地址：接收缓冲区 gucURxBuf */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)gucURxBuf;
	/* 传输方向：外设 → 内存 */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	/* 缓冲区大小：与 USART_RX_BUF_SIZE 一致 */
	DMA_InitStructure.DMA_BufferSize = (uint32_t)USART_RX_BUF_SIZE;
	/* 外设地址固定：DR 寄存器只有一个地址 */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	/* 内存地址递增：依次填入 gucURxBuf[0], [1], ... */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	/* 外设数据宽度：字节（8bit），串口数据寄存器低 8 位有效 */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	/* 内存数据宽度：字节（8bit） */
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	/* 循环模式：必须！否则接收完一包数据后 DMA 自动停止 */
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	/* 优先级：高，确保串口数据及时搬运 */
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	/* FIFO 模式：禁用，使用直接模式 */
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	/* FIFO 阈值：FIFO 禁用时无效 */
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	/* 内存突发传输：单次 */
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	/* 外设突发传输：单次 */
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DEBUG_USART_RX_DMA_STREAM, &DMA_InitStructure);

	return;
}

/**
 * @description: 串口 DMA 接收初始化入口
 * @detail:
 *   调用 Bsp_Dma1_Init() 初始化 DMA1 Stream1，然后使能 DMA 传输和 USART DMA 请求：
 *   1. DMA_SetCurrDataCounter → 设置当前传输计数 = 缓冲区大小
 *   2. DMA_Cmd → 使能 DMA1 Stream1，开始等待 USART DMA 请求
 *   3. USART_DMACmd → 使能 USART 接收 DMA 请求，串口收到字节时自动触发 DMA 搬运
 *
 *   注意：USART_DMACmd 必须在 DMA_Cmd 之后调用，否则 DMA 不会响应串口请求。
 * @return {VOID}
 * @author: lkc
 */
VOID Bsp_Dma_UsartRx(VOID)
{
	/* 初始化 DMA1 Stream1 通道4 配置 */
	Bsp_Dma1_Init();

	/* 设置 DMA 传输数据量 = 接收缓冲区大小 */
	DMA_SetCurrDataCounter(DEBUG_USART_RX_DMA_STREAM, USART_RX_BUF_SIZE);
	/* 使能 DMA1 Stream1，开始监听 USART DMA 请求 */
	DMA_Cmd(DEBUG_USART_RX_DMA_STREAM, ENABLE);

	/* 使能 USART 接收 DMA：串口每收到一个字节，自动触发一次 DMA 搬运
	 * 必须在 DMA_Cmd 之后才使能，否则 DMA 不响应 */
	USART_DMACmd(DEBUG_USART, USART_DMAReq_Rx, ENABLE);

	return;
}

/**
 * @description: DMA 系统初始化顶层入口
 * @detail:
 *   初始化系统全部 DMA 通道：
 *   1. Bsp_Dma_AdcSample() → DMA2 Stream4：ADC 采样数据自动搬运至内存
 *   2. Bsp_Dma_UsartRx()  → DMA1 Stream1：串口接收数据自动搬运至缓冲区
 * @return {VOID}
 * @author: lkc
 */
VOID Bsp_Dma_Init(VOID)
{
	/* DMA2 Stream4：ADC 数据搬运 + 中断配置 */
	Bsp_Dma_AdcSample();
	/* DMA1 Stream1：串口接收数据搬运 */
	Bsp_Dma_UsartRx();

	return;
}

#ifdef __cplusplus
}
#endif
