/*
 * @Description: 电流偏移校准的 ADC 中断包装
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_detect_isr.h"
#include "common_assert.h"

#include <stddef.h>

#include "motor_detect.h"
#include "motor_sense.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 为偏移校准处理一次 ADC 中断样本。
 * @detail: 先换算采样帧，再按 PWM 方向把样本交给非阻塞校准状态机。
 * @param {MOTOR_PARAM_S *} motor 电机采样对象，不可为空
 * @param {bool} pwm_down true=PWM 向下计数侧
 * @return {void} 无
 */
void Motor_DetectIsr_Step(MOTOR_PARAM_S *motor, bool pwm_down)
{
    COMMON_ASSERT(motor != NULL);

    Motor_Sense_Update(motor);
    Motor_Detect_IsrFeed(pwm_down);
}
