
/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 10:03:36
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-10 22:51:34
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
#define IRQ_TIM3             (6)
#define IRQ_USART3_IDE       (5)
#define IRQ_DMA2_STREAM2     (1)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Bsp_Init(void);
#ifdef __cplusplus
}
#endif

#endif
