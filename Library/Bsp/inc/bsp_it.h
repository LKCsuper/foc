/*
 * @Description: 中断处理函数声明（DMA/串口/定时器/SysTick）
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-08-28 19:45:06
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-03 09:20:14
 * @detail: 
 */
#ifndef _BSP_IT_H
#define _BSp_IT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: DMA2 Stream4 中断处理（ADC 采样完成触发 FOC 计算）
 * @return {void} 无
 */
VOID Bsp_Dma2Stream4_Handler(VOID);

/**
 * @description: 调试串口空闲中断处理（收取一帧命令）
 * @return {void} 无
 */
VOID Bsp_Usart_Handler(VOID);

/**
 * @description: 定时器3 中断处理（FreeRTOS 运行时间统计）
 * @return {void} 无
 */
VOID Bsp_Tim3_IRQ(VOID);

/**
 * @description: 定时器2 CC2 中断处理
 * @return {void} 无
 */
VOID Bsp_Tim2_IRQ(VOID);

/**
 * @description: SysTick 中断处理（转发到 FreeRTOS）
 * @return {void} 无
 */
VOID Bsp_SysTick_Handler(VOID);

/**
 * @description: 中断初始化入口
 * @return {void} 无
 */
VOID Bsp_It_Init(VOID);
#ifdef __cplusplus
}
#endif
#endif
