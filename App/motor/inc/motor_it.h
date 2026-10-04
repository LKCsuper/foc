/*
 * @Description: ADC-DMA 中断与 FOC 控制环入口
 * @Author: lkc
 */
#ifndef _MOTOR_IT_H
#define _MOTOR_IT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: ADC-DMA 传输完成中断回调，FOC 控制链驱动入口。
 * @detail: 按校准、BLDC、快速 V/F 与正常 FOC 的顺序分派，本函数不承载控制算法。
 * @param {void *} p 回调用户参数，未使用
 * @param {uint32_t} flags DMA 事件标志，未使用
 * @return {void} 无
 */
void Motor_DmaInt(void *p, uint32_t flags);

#ifdef __cplusplus
}
#endif

#endif
