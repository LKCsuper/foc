/*
 * @Description: 纯位置 P 串速度 PI，状态由调用者持有
 * @Author: lkc
 */
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "motor_position_control.h"

/**
 * @description: 把数值钳到对称区间。
 * @param {float} value 待钳位值
 * @param {float} limit 对称限幅，须为非负
 * @return {float} 钳位结果，范围 [-limit, limit]
 */
static float clamp_symmetric(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/**
 * @description: 检查位置控制配置的有限性和符号。
 * @param {const motor_position_control_config_t *} config 控制配置
 * @return {bool} 可用于本拍计算返回 true
 */
static bool config_valid(const motor_position_control_config_t *config)
{
    return config != NULL && isfinite(config->position_kp_rpm_per_deg) &&
           isfinite(config->maximum_rpm) && isfinite(config->velocity_kp_v_per_rpm) &&
           isfinite(config->velocity_ki_v_per_rpm_s) && isfinite(config->voltage_limit_v) &&
           isfinite(config->acceleration_limit_rpm_s) && isfinite(config->feedforward_v_per_rpm) &&
           isfinite(config->maximum_dt_s) &&
           config->position_kp_rpm_per_deg > 0.0f &&
           config->maximum_rpm > 0.0f && config->velocity_kp_v_per_rpm >= 0.0f &&
           config->velocity_ki_v_per_rpm_s >= 0.0f && config->voltage_limit_v > 0.0f &&
           config->acceleration_limit_rpm_s > 0.0f && config->feedforward_v_per_rpm >= 0.0f &&
           config->maximum_dt_s > 0.0f;
}

/**
 * @description: 按配置检查并清零位置控制状态。
 * @param {motor_position_control_t *} control              控制状态，不可为空
 * @param {const motor_position_control_config_t *} config  控制配置，不可为空
 * @return {bool} 初始化成功返回 true
 * @author: lkc
 */
bool Motor_PositionControl_Init(motor_position_control_t *control,
                                const motor_position_control_config_t *config)
{
    if (control == NULL || !config_valid(config)) return false;
    memset(control, 0, sizeof(*control));
    return true;
}

/**
 * @description: 清除速度积分。
 * @param {motor_position_control_t *} control 控制状态，为空则直接返回
 * @return {void} 无
 * @author: lkc
 */
void Motor_PositionControl_Reset(motor_position_control_t *control)
{
    if (control != NULL) {
        control->velocity_integrator_v = 0.0f;
        control->commanded_rpm = 0.0f;
    }
}

/**
 * @description: 推进一拍位置 P 串速度 PI。
 * @detail: 位置误差先换成转速并限幅。积分只在未顶饱和的方向上更新。
 * @param {motor_position_control_t *} control              控制状态，不可为空
 * @param {const motor_position_control_config_t *} config  控制配置，不可为空
 * @param {const motor_position_control_input_t *} input    本拍输入，不可为空
 * @param {motor_position_control_output_t *} output        本拍输出，不可为空
 * @return {bool} 本拍计算完成返回 true
 * @author: lkc
 */
bool Motor_PositionControl_Step(motor_position_control_t *control,
                                const motor_position_control_config_t *config,
                                const motor_position_control_input_t *input,
                                motor_position_control_output_t *output)
{
    float velocity_error;
    float proportional;
    float feedforward;
    float integral_candidate;
    float unsaturated;
    float dt;
    float raw_target_rpm;
    float command_step;
    bool winding_up;

    if (control == NULL || !config_valid(config) || input == NULL || output == NULL ||
        !isfinite(input->target_deg) || !isfinite(input->position_deg) ||
        !isfinite(input->measured_rpm) || !isfinite(input->dt)) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    output->error_deg = input->target_deg - input->position_deg;
    raw_target_rpm = clamp_symmetric(config->position_kp_rpm_per_deg * output->error_deg,
                                     config->maximum_rpm);
    output->target_rpm = raw_target_rpm;
    if (!input->enabled || !input->motor_running || !input->feedback_valid) {
        Motor_PositionControl_Reset(control);
        output->target_rpm = 0.0f;
        return true;
    }

    dt = (input->dt > 0.0f && input->dt <= config->maximum_dt_s) ? input->dt : 0.0f;
    if (dt > 0.0f) {
        command_step = config->acceleration_limit_rpm_s * dt;
        control->commanded_rpm += clamp_symmetric(raw_target_rpm - control->commanded_rpm, command_step);
    }
    output->target_rpm = control->commanded_rpm;
    velocity_error = output->target_rpm - (input->speed_valid ? input->measured_rpm : 0.0f);
    proportional = config->velocity_kp_v_per_rpm * velocity_error;
    feedforward = config->feedforward_v_per_rpm * output->target_rpm;
    integral_candidate = control->velocity_integrator_v;
    if (dt > 0.0f) {
        integral_candidate += config->velocity_ki_v_per_rpm_s * velocity_error * dt;
    }
    integral_candidate = clamp_symmetric(integral_candidate, config->voltage_limit_v);
    unsaturated = feedforward + proportional + integral_candidate;
    /*
     * 条件积分：比例与积分之和已经顶到电压限，且速度误差仍推向同一侧时，本拍不更新积分。
     * 积分候选值先钳在 ±voltage_limit_v。
     */
    winding_up = (unsaturated > config->voltage_limit_v && velocity_error > 0.0f) ||
                 (unsaturated < -config->voltage_limit_v && velocity_error < 0.0f);
    if (!winding_up) control->velocity_integrator_v = integral_candidate;

    output->hold_active = true;
    output->output_v = clamp_symmetric(feedforward + proportional + control->velocity_integrator_v,
                                       config->voltage_limit_v);
    return true;
}
