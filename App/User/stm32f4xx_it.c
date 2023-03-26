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
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
    return;
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1){}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1){}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{

}

/**
 * @description: sysTick ,freertos任务
 * @detail: 
 * @return {*}
 * @author: lkc
 */
extern void xPortSysTickHandler(void);
void SysTick_Handler(void)
{	
    #if (INCLUDE_xTaskGetSchedulerState  == 1 )
      if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
      {
    #endif  /* INCLUDE_xTaskGetSchedulerState */  
        xPortSysTickHandler();
    #if (INCLUDE_xTaskGetSchedulerState  == 1 )
      }
    #endif  /* INCLUDE_xTaskGetSchedulerState */
}

/**
 * @description: DMA 采集完成adc
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void DMA2_Stream4_IRQHandler(void)
{
    Motor_DmaInt(NULL, NULL);
    DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);
    DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_HTIF4);
    
    return;
}

/**
 * @description: foc用不到
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void ADC_IRQHandler(void)
{
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    return;
}

#if (3 == USART_NUM)
/**
 * @description: 串口3中断,用于命令行
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void USART3_IRQHandler( void )
{
    Bsp_Usart_Handler();
    return;
}
#elif (4 == USART_NUM)
/**
 * @description: 串口3中断,用于命令行
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void UART4_IRQHandler( void )
{
    Bsp_Usart_Handler();
    return;
}
#endif

/**
 * @description: 定时器中断,用于统计freertos
 * @detail: 
 * @return {*}
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


