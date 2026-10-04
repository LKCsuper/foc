/*
 * @Description: BM75/HD75 实机验证过的 BLDC 默认启动参数
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_bldc_defaults.h"

#include <stddef.h>

#include "motor_bldc_six_step_hal.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 装入 BM75/HD75 已验证的六步与 BEMF 默认参数并初始化实例。
 * @detail: 不启动功率输出。换相输出回调接到板级 HAL。
 * @param {motor_bldc_six_step_t *} commutator 六步换相实例，不可为空
 * @param {motor_bldc_sensorless_t *} sensorless 无感控制实例，不可为空
 * @return {bool} 两个实例都初始化成功返回 true
 */
bool Motor_BldcDefaults_Init(motor_bldc_six_step_t *commutator, motor_bldc_sensorless_t *sensorless)
{
    motor_bldc_six_step_config_t six_config;
    motor_bldc_sensorless_config_t sensorless_config;

    if (commutator == NULL || sensorless == NULL) {
        return false;
    }
    Motor_BldcSixStep_DefaultConfig(&six_config);
    six_config.apply_output = Motor_BldcSixStep_HalApply;
    six_config.output_user = NULL;
    if (!Motor_BldcSixStep_Init(commutator, &six_config)) {
        return false;
    }

    Motor_BldcSensorless_DefaultConfig(&sensorless_config);
    /* 24 V 实测 8%/12ms 基本同步，14ms 明显抖动。 */
    sensorless_config.startup_duty = 0.08f;
    sensorless_config.duty_slew_rate_per_s = 0.05f;
    sensorless_config.startup_initial_period_s = 0.050f;
    sensorless_config.startup_final_period_s = 0.012f;
    sensorless_config.startup_ramp_time_s = 0.60f;
    sensorless_config.startup_timeout_s = 3.0f;
    sensorless_config.closed_loop_timeout_s = 0.150f;
    sensorless_config.period_change_limit = 0.01f;
    sensorless_config.closed_loop_min_period_s = 0.001f;
    sensorless_config.duty_period_enabled = true;
    sensorless_config.bemf_integral_threshold_vs = 0.004f;
    sensorless_config.integral_auto_detect_enabled = false;
    sensorless_config.integral_detect_scale = 1.0f;
    sensorless_config.bemf_integral_noise_v = 0.05f;
    sensorless_config.step0_rising = true;
    return Motor_BldcSensorless_Init(sensorless, commutator, &sensorless_config);
}
