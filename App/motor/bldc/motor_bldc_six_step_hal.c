/*
 * @Description: BLDC 六步控制结果到电机 HAL 的应用适配
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "hal_motor_bldc.h"
#include "motor_bldc_six_step_hal.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static hal_motor_bldc_phase_mode_t Motor_BldcSixStep_ToHal(motor_bldc_phase_mode_t mode);
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 将控制层桥臂状态转换为 HAL 桥臂状态。
 * @param {motor_bldc_phase_mode_t} mode 控制层桥臂状态
 * @return {hal_motor_bldc_phase_mode_t} HAL 桥臂状态
 * @author: lkc
 */
static hal_motor_bldc_phase_mode_t Motor_BldcSixStep_ToHal(motor_bldc_phase_mode_t mode)
{
    if (mode == MOTOR_BLDC_PHASE_PWM) {
        return HAL_MOTOR_BLDC_PHASE_PWM;
    }
    if (mode == MOTOR_BLDC_PHASE_LOW) {
        return HAL_MOTOR_BLDC_PHASE_LOW;
    }
    return HAL_MOTOR_BLDC_PHASE_FLOAT;
}

/**
 * @description: 将六步控制结果应用到平台无关电机 HAL。
 * @param {const motor_bldc_output_t *} output 六步控制输出；NULL 表示三相悬空
 * @param {void *} user 保留的回调上下文，当前不使用
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSixStep_HalApply(const motor_bldc_output_t *output, void *user)
{
    (void)user;
    if (output == NULL) {
        Hal_Motor_BldcApply(HAL_MOTOR_BLDC_PHASE_FLOAT, HAL_MOTOR_BLDC_PHASE_FLOAT,
                            HAL_MOTOR_BLDC_PHASE_FLOAT, 0.0f);
        return;
    }
    Hal_Motor_BldcApply(Motor_BldcSixStep_ToHal(output->phase[0]),
                        Motor_BldcSixStep_ToHal(output->phase[1]),
                        Motor_BldcSixStep_ToHal(output->phase[2]), output->duty);
}
#ifdef __cplusplus
}
#endif
