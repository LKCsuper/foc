/*
 * @Description: BLDC 六步控制结果到电机 HAL 的应用适配
 * @Author: lkc
 */
#ifndef MOTOR_BLDC_SIX_STEP_HAL_H
#define MOTOR_BLDC_SIX_STEP_HAL_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_bldc_six_step.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 将六步控制器输出转换为平台无关 HAL 桥臂命令。
 * @param {const motor_bldc_output_t *} output 六步控制输出；NULL 时三相悬空
 * @param {void *} user 保留的回调上下文，当前不使用
 * @return {void} 无
 */
void Motor_BldcSixStep_HalApply(const motor_bldc_output_t *output, void *user);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif /* MOTOR_BLDC_SIX_STEP_HAL_H */
