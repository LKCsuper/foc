/*
 * @Description: Cortex-M4 异常与外设中断服务程序声明
 * @Author: lkc
 */
/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/stm32f4xx_it.h 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
 * @description: NMI 不可屏蔽中断异常处理。
 * @return {void} 无
 * @author: lkc
 */
void NMI_Handler(void);
/**
 * @description: HardFault 硬件故障异常处理。
 * @return {void} 无
 * @author: lkc
 */
void HardFault_Handler(void);
/**
 * @description: MemManage 内存管理异常处理。
 * @return {void} 无
 * @author: lkc
 */
void MemManage_Handler(void);
/**
 * @description: BusFault 总线故障异常处理。
 * @return {void} 无
 * @author: lkc
 */
void BusFault_Handler(void);
/**
 * @description: UsageFault 用法故障异常处理。
 * @return {void} 无
 * @author: lkc
 */
void UsageFault_Handler(void);
/**
 * @description: DebugMon 调试监视器异常处理。
 * @return {void} 无
 * @author: lkc
 */
void DebugMon_Handler(void);
/**
 * @description: SysTick 系统节拍中断处理。
 * @return {void} 无
 * @author: lkc
 */
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

