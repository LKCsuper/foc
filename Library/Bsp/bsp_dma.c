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

	DMA_InitTypeDef DMA_InitStructure;
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
	/* 写入 DMA 配置并立即使能 Stream4 */
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream4, ENABLE);

	return;
}

/**
 * @description: dma2 中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
STATIC VOID Bsp_Dma2_NVIC(VOID)
{
	/* 中断 */
	NVIC_InitTypeDef NVIC_InitStre;
	NVIC_InitStre.NVIC_IRQChannel = DMA2_Stream4_IRQn;
    NVIC_InitStre.NVIC_IRQChannelPreemptionPriority = IRQ_DMA2_STREAM2;
    NVIC_InitStre.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStre.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStre);
/*
	 Note: The half transfer interrupt is used as we already have all current and voltage
	 samples by then and we can start processing them. Entering the interrupt earlier gives
	 more cycles to finish it and update the timer before the next zero vector. This helps
	 at higher f_zv. Only use this if the three first samples are current samples.
*/
	/* 这里尤其注意一下,半dma中断模式,已经采样完成电流电压,可以提高采样频率 */
#if ADC_IND_CURR1 < 3 && ADC_IND_CURR2 < 3 && ADC_IND_CURR3 < 3
	DMA_ITConfig(DMA2_Stream4, DMA_IT_HT, ENABLE);
#else
	/* 开启传输完成中断 */
	DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
#endif

	// TODO 清空一遍中断标志
	//DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);

	return;
}

/**
 * @description: DMA adc采样触发
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Dma_AdcSample(VOID)
{
	Bsp_DMA2_Init();
	Bsp_Dma2_NVIC();

	return;
}

/**
 * @description: dma1 初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Dma1_Init(VOID)
{
	/* 每个dma数据流和通道对应不同的外设 */
	DMA_DeInit(DMA1_Stream1);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStructure;
	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE){} /* 检测dma是否之前配置过 */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(DEBUG_USART->DR); /* 外设地址 */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)gucURxBuf; /* 内存地址 */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; /* 外设到内存 */
	DMA_InitStructure.DMA_BufferSize = (uint32_t)32; /* 一次性传输大小 */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /* 外设地址不变 */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; /* memory地址自增 */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; /* 外设地址数据单位 */
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; /* memory地址数据单位 */
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; /* 注意! 这里一定要使用循环模式,否则收到一包之后就停止了 */
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; /* 优先级 */
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;       
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; /* 存储器突发模式选择，可选单次模式、 4 节拍的增量突发模式、 8 节拍的增量突发模式或 16 节拍的增量突发模式。*/
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; /* 外设突发模式选择，可选单次模式、 4 节拍的增量突发模式、 8 节拍的增量突发模式或 16 节拍的增量突发模式。 */
	DMA_Init(DMA1_Stream1, &DMA_InitStructure); 

	return;
}

/**
 * @description: 串口接受dma
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Dma_UsartRx(VOID)
{
	Bsp_Dma1_Init();

	DMA_SetCurrDataCounter(DMA1_Stream1, USART_RX_BUF_SIZE);
	DMA_Cmd(DMA1_Stream1, ENABLE);

	// TODO 注意 如果是使能dma一定要加这个
	USART_DMACmd(DEBUG_USART, USART_DMAReq_Rx, ENABLE);

	return;
}

/**
 * @description: dma初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Dma_Init(VOID)
{
	/* dma ADC 采样 */
	Bsp_Dma_AdcSample();
	/* 串口接收 */
	Bsp_Dma_UsartRx();

	return;
}

#ifdef __cplusplus
}
#endif
