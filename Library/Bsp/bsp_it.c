/*
 * @Description: 中断
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-08-27 10:31:55
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-09 20:46:12
 * @detail: 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "motor_timing.h"
#include "hal_motor.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 定时器3中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
extern ULONG ulFreeRTOSRunTimeTicks;
VOID Bsp_Tim3_IRQ(VOID)
{
	if(SET == TIM_GetITStatus(HW_COUNT_TIM, TIM_IT_Update)) //溢出中断
	{
		ulFreeRTOSRunTimeTicks++;
	}
	TIM_ClearITPendingBit(HW_COUNT_TIM, TIM_IT_Update);
	return;
}

/**
 * @description: 串口3中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
ULONG ulRxLen = 0;
static CMD_QUEUE_ITEM rx_item;

/**
 * @description: 收取 UART RX DMA 当前帧并重新启动接收。
 * @detail: 先关掉 UART 的 DMA 请求再关流，避免电机噪声持续触发时 EN 永不落下。
 *          等待 DMA 关闭必须有上限：空等会卡住 UART 中断，FOC 仍跑、命令任务永久饿死。
 * @return {void} 无
 */
static void Bsp_Usart_HarvestRx(void)
{
    volatile uint32_t timeout = 1000U;

    DEBUG_USART->SR;
    DEBUG_USART->DR;
    USART_ClearITPendingBit(DEBUG_USART, USART_IT_IDLE);

    USART_DMACmd(DEBUG_USART, USART_DMAReq_Rx, DISABLE);
    DMA_Cmd(DEBUG_USART_RX_DMA_STREAM, DISABLE);
    while ((DMA_GetCmdStatus(DEBUG_USART_RX_DMA_STREAM) != DISABLE) && (timeout > 0U)) {
        timeout--;
    }
    if (DMA_GetCmdStatus(DEBUG_USART_RX_DMA_STREAM) != DISABLE) {
        DEBUG_USART_RX_DMA_STREAM->CR &= ~(uint32_t)DMA_SxCR_EN;
        timeout = 1000U;
        while ((DMA_GetCmdStatus(DEBUG_USART_RX_DMA_STREAM) != DISABLE) && (timeout > 0U)) {
            timeout--;
        }
    }

    if (DMA_GetCmdStatus(DEBUG_USART_RX_DMA_STREAM) == DISABLE) {
        ulRxLen = USART_RX_BUF_SIZE - DMA_GetCurrDataCounter(DEBUG_USART_RX_DMA_STREAM);
        if (ulRxLen > USART_RX_BUF_SIZE) {
            ulRxLen = USART_RX_BUF_SIZE;
        }
        if (queueCmdHandle != NULL && ulRxLen > 0U) {
            rx_item.len = ulRxLen;
            memcpy(rx_item.data, gucURxBuf, ulRxLen);
            osMessageQueuePut(queueCmdHandle, &rx_item, NULL, 0);
        }
        DMA_ClearFlag(DEBUG_USART_RX_DMA_STREAM, DEBUG_USART_RX_DMA_FLAGS);
        DMA_SetCurrDataCounter(DEBUG_USART_RX_DMA_STREAM, USART_RX_BUF_SIZE);
        DMA_Cmd(DEBUG_USART_RX_DMA_STREAM, ENABLE);
    }
    USART_DMACmd(DEBUG_USART, USART_DMAReq_Rx, ENABLE);
}

/**
 * @description: UART 空闲中断：收取一帧命令并重启 RX DMA。
 * @return {void} 无
 */
VOID Bsp_Usart_Handler(VOID)
{
    if (USART_GetITStatus(DEBUG_USART, USART_IT_IDLE) != RESET) {
        Bsp_Usart_HarvestRx();
    }
}

/**
 * @description: dma中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Dma2Stream4_Handler(VOID)
{
    /* modify:codex 原因：只在当前启用的 HT/TC 标志置位时调用 Motor_DmaInt，避免非目标或残留 DMA 标志误触发 FOC 计算。 */
#if ADC_IND_CURR1 < 3 && ADC_IND_CURR2 < 3 && ADC_IND_CURR3 < 3
    if (RESET != DMA_GetITStatus(DMA2_Stream4, DMA_IT_HTIF4))
    {
        DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_HTIF4);
        /* 只统计回调耗时，不改变清标志和 Motor_DmaInt 的顺序。 */
        Motor_Timing_CallbackBegin(Hal_Motor_TimeUs());
        Motor_DmaInt(NULL, NULL);
        Motor_Timing_CallbackEnd(Hal_Motor_TimeUs(), RESET != DMA_GetITStatus(DMA2_Stream4, DMA_IT_HTIF4));
    }
#else
    if (RESET != DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4))
    {
        DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);
        /* 只统计回调耗时，不改变清标志和 Motor_DmaInt 的顺序。 */
        Motor_Timing_CallbackBegin(Hal_Motor_TimeUs());
        Motor_DmaInt(NULL, NULL);
        Motor_Timing_CallbackEnd(Hal_Motor_TimeUs(), RESET != DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4));
    }
#endif
    return;
}

/**
 * @description: sysTick 中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
extern void xPortSysTickHandler(void);
VOID Bsp_SysTick_Handler(VOID)
{
    #if (INCLUDE_xTaskGetSchedulerState  == 1 )
      if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
      {
    #endif  /* INCLUDE_xTaskGetSchedulerState */  
        xPortSysTickHandler();
    #if (INCLUDE_xTaskGetSchedulerState  == 1 )
      }
    #endif  /* INCLUDE_xTaskGetSchedulerState */
    return;
}

/**
 * @description: 定时器2中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim2_IRQ(VOID)
{
	if(SET == TIM_GetITStatus(HW_TRIG_TIM, TIM_IT_CC2)) //溢出中断
	{
		ulFreeRTOSRunTimeTicks++;
	}
	TIM_ClearITPendingBit(HW_TRIG_TIM, TIM_IT_CC2);
	return;
}

/**
 * @description: 中断初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_It_Init(VOID)
{
	// NVIC_InitTypeDef NVIC_InitStre;
	// NVIC_InitStre.NVIC_IRQChannel = TIM3_IRQn;
    // NVIC_InitStre.NVIC_IRQChannelPreemptionPriority = IRQ_TIM3;
    // NVIC_InitStre.NVIC_IRQChannelSubPriority = 0x0;
    // NVIC_InitStre.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStre);

	// TIM_ITConfig(HW_COUNT_TIM, TIM_IT_Update, ENABLE);
	// TIM_ClearITPendingBit(HW_COUNT_TIM, TIM_IT_Update);

	
	return;
}
#ifdef __cplusplus
}
#endif
