/*
 * @Description: 中断
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-08-27 10:31:55
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-03 09:18:52
 * @detail: 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
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

/**
 * @description: dma中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Dma2Stream4_Handler(VOID)
{
    Motor_DmaInt(NULL, NULL);
    DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);
    DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_HTIF4);
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
