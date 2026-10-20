/*
 * @Description: 电流偏移校准的 ADC 中断包装
 * @Author: lkc
 */
#ifndef MOTOR_DETECT_ISR_H
#define MOTOR_DETECT_ISR_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 为偏移校准处理一次 ADC 中断样本。
 * @detail: 先换算采样帧，再按 PWM 方向把样本交给非阻塞校准状态机。
 *          不执行 FOC，也不做软件过流判断。仅在 ADC 中断、校准占用输出时调用。
 * @param {MOTOR_PARAM_S *} motor 电机采样对象，不可为空
 * @param {bool} pwm_down true=PWM 向下计数侧
 * @return {void} 无
 */
void Motor_DetectIsr_Step(MOTOR_PARAM_S *motor, bool pwm_down);

/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
