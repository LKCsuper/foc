
/*
 * @Description: 板级初始化与中断优先级配置
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 10:03:36
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-13 23:02:32
 */
#ifndef _BSP_H
#define _BSP_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 这里存储中断优先级,方便安排 */
#define IRQ_TIM3                (6)
#define IRQ_DEBUG_IDE           (5)
/* TX DMA 完成不调用 RTOS，可以高于 configLIBRARY_MAX_SYSCALL(5) 并抢占 FOC。
 * UART IDLE 必须保持 IRQ_DEBUG_IDE：HarvestRx 会 osMessageQueuePut。 */
#define IRQ_DEBUG_TX            (0)
/* modify:codex 原因：ADC DMA 中断实际是 DMA2 Stream4；保留旧宏别名，避免其他旧引用编译失败。 */
#define IRQ_DMA2_STREAM4        (1)
#define IRQ_DMA2_STREAM2        IRQ_DMA2_STREAM4

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 初始化系统时钟与 FreeRTOS 基础外设
 * @return {void} 无
 */
VOID Bsp_RTOS_Init(VOID);

/**
 * @description: 板级硬件初始化（GPIO/串口/ADC/DMA/定时器）
 * @return {void} 无
 */
VOID Bsp_Init(VOID);
#ifdef __cplusplus
}
#endif

#endif
