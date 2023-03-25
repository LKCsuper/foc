/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-25 18:32:46
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
 * @description: dma2 初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
STATIC VOID Bsp_DMA2_Init(VOID)
{
	/* 时钟初始化 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* 开启中断 */
    DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; 	/* 选择通道0 */
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value; 	/* 外设到内存 */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR; 	/* 外设地址 */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	/* 传输方向 */
	DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS; /* 传输大小15 */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /* 外设地址增量模式,就是外设地址递增 */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; /* 内存的地址是变化的 */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /* 外设大小是半字 */
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; /* 内存地址大小半字 */
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; /* 循环接受直到接受完成 */
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; /* 优先等级高 */
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; /* FIFO模式关闭 */
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; /* 内存突发传输 */
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; /* 外设突发传输 */
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
