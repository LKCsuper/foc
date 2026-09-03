/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 0 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "includes.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @description: NMI 不可屏蔽中断异常处理。
 * @return {void} 无
 * @author: lkc
 */
void NMI_Handler(void)
{
    return;
}

/**
 * @description: HardFault 硬件故障异常处理，进入死循环。
 * @return {void} 无
 * @author: lkc
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @description: MemManage 内存管理异常处理，进入死循环。
 * @return {void} 无
 * @author: lkc
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
 * @description: BusFault 总线故障异常处理，进入死循环。
 * @return {void} 无
 * @author: lkc
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1){}
}

/**
 * @description: UsageFault 用法故障异常处理，进入死循环。
 * @return {void} 无
 * @author: lkc
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1){}
}

/**
 * @description: DebugMon 调试监视器异常处理。
 * @return {void} 无
 * @author: lkc
 */
void DebugMon_Handler(void)
{

}

/**
 * @description: SysTick 系统节拍中断，驱动 FreeRTOS 与 BSP 时基。
 * @return {void} 无
 * @author: lkc
 */
void SysTick_Handler(void)
{	
    Bsp_SysTick_Handler();

    return;
}

/**
 * @description: DMA2 Stream4 中断 — ADC 采集完成回调。
 * @return {void} 无
 * @author: lkc
 */
void DMA2_Stream4_IRQHandler(void)
{
    Bsp_Dma2Stream4_Handler();
    
    return;
}

#if (4 == USART_NUM)
/**
 * @description: DMA1 Stream4 中断 — 调试串口 TX DMA 发送完成。
 * @return {void} 无
 * @author: lkc
 */
void DMA1_Stream4_IRQHandler(void)
{
    Bsp_Usart_TxDmaHandler();
}
#elif (3 == USART_NUM)
/**
 * @description: DMA1 Stream3 中断 — 调试串口 TX DMA 发送完成。
 * @return {void} 无
 * @author: lkc
 */
void DMA1_Stream3_IRQHandler(void)
{
    Bsp_Usart_TxDmaHandler();
}
#elif (1 == USART_NUM)
/**
 * @description: DMA2 Stream7 中断 — 调试串口 TX DMA 发送完成。
 * @return {void} 无
 * @author: lkc
 */
void DMA2_Stream7_IRQHandler(void)
{
    Bsp_Usart_TxDmaHandler();
}
#endif

/**
 * @description: ADC 注入转换完成中断（FOC 未使用，仅清标志）。
 * @return {void} 无
 * @author: lkc
 */
void ADC_IRQHandler(void)
{
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    return;
}

#if (3 == USART_NUM)
/**
 * @description: USART3 串口接收中断 — 命令行输入。
 * @return {void} 无
 * @author: lkc
 */
void USART3_IRQHandler( void )
{
    Bsp_Usart_Handler();
    return;
}
#elif (4 == USART_NUM)
/**
 * @description: UART4 串口接收中断 — 命令行输入。
 * @return {void} 无
 * @author: lkc
 */
void UART4_IRQHandler( void )
{
    Bsp_Usart_Handler();
    return;
}
#endif

/**
 * @description: TIM3 定时器中断 — FreeRTOS 运行时统计计数。
 * @return {void} 无
 * @author: lkc
 */
void TIM3_IRQHandler(void)
{
    Bsp_Tim3_IRQ();

    return;
}
#if 0
/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
    xPortPendSVHandler();
}
/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
    vPortSVCHandler();
}
#endif
/**
  * @}
  */ 

/**
  * @}
  */ 


