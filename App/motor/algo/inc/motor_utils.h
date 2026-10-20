/*
 * @Description: 电机外设与 PWM 辅助工具
 * @Author: lkc
 */
#ifndef _MOTOR_UTILS_H
#define _MOTOR_UTILS_H
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
 * @description: 根据死区时间与内核时钟计算 TIM 死区寄存器 DTG 值。
 * @param {float} deadtime_ns       目标死区时间 (ns)
 * @param {float} core_clock_freq   内核时钟频率 (Hz)
 * @return {uint8_t} DTG 寄存器编码值
 * @author: lkc
 */
uint8_t calculate_deadtime(float deadtime_ns, float core_clock_freq);

#ifdef __cplusplus
}
#endif

#endif
