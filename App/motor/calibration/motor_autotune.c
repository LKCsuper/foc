/* 基于 RL 模型的电流 PI 初值整定；纯计算，不驱动电机、不应用参数。 */
/* Includes ------------------------------------------------------------------*/
#include "motor_autotune.h"
#include <math.h>
#include <stddef.h>

/* Private define ------------------------------------------------------------*/
#define AUTOTUNE_PI (3.14159265358979323846f)
#define AUTOTUNE_MAX_W_DT (0.2f)

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 按 Kp=L·ωc、Ki=R·ωc 生成 dq 电流 PI 候选初值。
 * @detail: 纯计算，不驱动电机、不应用参数、不写 Flash；检查 ωc·dt≤0.2 与延时相位裕量预算。
 * @param {const motor_autotune_config_t *} config 输入配置，不可为空
 * @param {motor_autotune_result_t *} result 输出结果，不可为空
 * @return {motor_autotune_status_t} 成功或具体拒绝原因
 */
motor_autotune_status_t Motor_Autotune_CurrentPi(const motor_autotune_config_t *config,
                                               motor_autotune_result_t *result)
{
    motor_autotune_result_t candidate = {0};
    float omega, phase_loss, l_shared;

    if (result == NULL) return MOTOR_AUTOTUNE_INVALID_INPUT;
    *result = candidate;
    if (config == NULL || !isfinite(config->resistance_ohm) || config->resistance_ohm <= 0.0f ||
        !isfinite(config->ld_h) || config->ld_h <= 0.0f || !isfinite(config->lq_h) || config->lq_h <= 0.0f ||
        !isfinite(config->control_dt_s) || config->control_dt_s <= 0.0f ||
        !isfinite(config->bandwidth_hz) || config->bandwidth_hz <= 0.0f ||
        !isfinite(config->total_delay_s) || config->total_delay_s < 0.5f * config->control_dt_s ||
        !isfinite(config->min_phase_margin_deg) || config->min_phase_margin_deg <= 0.0f ||
        config->min_phase_margin_deg >= 90.0f) {
        return MOTOR_AUTOTUNE_INVALID_INPUT;
    }
    omega = 2.0f * AUTOTUNE_PI * config->bandwidth_hz;
    phase_loss = omega * config->total_delay_s * (180.0f / AUTOTUNE_PI);
    if (!isfinite(omega) || !isfinite(phase_loss)) return MOTOR_AUTOTUNE_NUMERIC_ERROR;
    if (omega * config->control_dt_s > AUTOTUNE_MAX_W_DT) return MOTOR_AUTOTUNE_SAMPLE_LIMIT;
    if (phase_loss > 90.0f - config->min_phase_margin_deg) return MOTOR_AUTOTUNE_DELAY_LIMIT;
    candidate.d.kp = config->ld_h * omega;
    candidate.q.kp = config->lq_h * omega;
    candidate.d.ki = candidate.q.ki = config->resistance_ohm * omega;
    l_shared = fminf(config->ld_h, config->lq_h);
    candidate.shared.kp = l_shared * omega;
    candidate.shared.ki = candidate.d.ki;
    candidate.bandwidth_hz = config->bandwidth_hz;
    candidate.sample_limit_hz = AUTOTUNE_MAX_W_DT / (2.0f * AUTOTUNE_PI * config->control_dt_s);
    candidate.delay_limit_hz = (90.0f - config->min_phase_margin_deg) / (360.0f * config->total_delay_s);
    candidate.model_phase_margin_deg = 90.0f - phase_loss;
    if (!isfinite(candidate.d.kp) || candidate.d.kp <= 0.0f || !isfinite(candidate.q.kp) ||
        candidate.q.kp <= 0.0f || !isfinite(candidate.d.ki) || candidate.d.ki <= 0.0f ||
        !isfinite(candidate.sample_limit_hz) || !isfinite(candidate.delay_limit_hz)) {
        return MOTOR_AUTOTUNE_NUMERIC_ERROR;
    }
    *result = candidate;
    return MOTOR_AUTOTUNE_OK;
}
