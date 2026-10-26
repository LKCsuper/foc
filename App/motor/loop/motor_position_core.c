/* 位置命令、多圈零点与混合控制状态；不访问硬件或全局电机。 */
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "common_assert.h"
#include <stddef.h>
#include <string.h>
#include "motor_position_core.h"

#include "motor_config.h"
#if MOTOR_CFG_POSITION
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把 uint32 累计计数转成 int32 回绕的有符号计数。
 * @param {uint32_t} count 计数值
 * @return {int32_t} 本次计算或查询的结果
 */
static int32_t SignedCount(uint32_t count)
{
    return count <= INT32_MAX ? (int32_t)count : -1 - (int32_t)(UINT32_MAX - count);
}

/**
 * @description: 用相对零点的回绕差值计算机械角度 (deg)。
 * @param {const motor_position_runtime_t *} runtime 位置环运行状态
 * @param {const motor_position_config_t *} config 配置
 * @param {uint32_t} count 计数值
 * @return {float} 本次计算或查询的结果
 */
static float PositionDeg(const motor_position_runtime_t *runtime, const motor_position_config_t *config,
                          uint32_t count)
{
    uint32_t delta = count - (uint32_t)runtime->state.origin_count;
    /* 明确uint32回绕的有符号差值，避免超范围转换依赖编译器。 */
    int64_t signed_delta = delta <= INT32_MAX ? (int64_t)delta : (int64_t)delta - 4294967296LL;
    return (float)signed_delta * 360.0f / (float)config->counts_per_rev;
}

/**
 * @description: 按配置初始化位置运行状态，非法配置不改状态。
 * @detail: 写入零点、默认比例与转速上限，控制积分清零并进入电压保持模式。
 * @param {motor_position_runtime_t *} runtime  运行状态，不可为空
 * @param {const motor_position_config_t *} config  配置，不可为空
 * @param {uint32_t} origin_count  多圈零点累计计数
 * @param {float} tolerance_deg    兼容保留的诊断容差 (deg)
 * @return {bool} 初始化成功返回 true
 */
bool Motor_PositionCore_Init(motor_position_runtime_t *runtime, const motor_position_config_t *config,
                             uint32_t origin_count, float tolerance_deg)
{
    motor_position_runtime_t fresh;
    if (runtime == NULL || config == NULL || config->counts_per_rev == 0U ||
        !isfinite(config->hold_enter_deg) || !isfinite(config->hold_exit_deg) ||
        config->hold_enter_deg < 0.0f || config->hold_exit_deg < config->hold_enter_deg ||
        !isfinite(tolerance_deg) || tolerance_deg < 0.0f) return false;
    memset(&fresh, 0, sizeof(fresh));
    if (!Motor_PositionControl_Init(&fresh.control, &config->control)) return false;
    fresh.state.origin_count = SignedCount(origin_count);
    fresh.state.kp_rpm_per_deg = config->control.position_kp_rpm_per_deg;
    fresh.state.max_rpm = config->control.maximum_rpm;
    fresh.state.tolerance_deg = tolerance_deg;
    fresh.state.voltage_hold_mode = true;
    *runtime = fresh;
    return true;
}

/**
 * @description: 退出位置保持，清除速度积分并回到电压保持模式。
 * @param {motor_position_runtime_t *} runtime  运行状态，不可为空
 * @return {void} 无
 */
void Motor_PositionCore_ClearHold(motor_position_runtime_t *runtime)
{
    COMMON_ASSERT(runtime != NULL);
    Motor_PositionControl_Reset(&runtime->control);
    runtime->state.voltage_hold_mode = true;
}

/**
 * @description: 置多圈零点并清目标、位置与速度积分。
 * @param {motor_position_runtime_t *} runtime  运行状态，不可为空
 * @param {uint32_t} count  新的零点累计计数
 * @return {void} 无
 */
void Motor_PositionCore_Zero(motor_position_runtime_t *runtime, uint32_t count)
{
    COMMON_ASSERT(runtime != NULL);
    runtime->state.origin_count = SignedCount(count);
    runtime->state.position_deg = 0.0f;
    runtime->state.target_deg = 0.0f;
    Motor_PositionCore_ClearHold(runtime);
}

/**
 * @description: 使能或禁用位置环。
 * @detail: 使能前要求 ready；禁用时清控制积分和保持模式。
 * @param {motor_position_runtime_t *} runtime  运行状态，不可为空
 * @param {bool} enable  true=使能
 * @param {bool} ready   true=可以进入使能
 * @return {bool} 设置成功返回 true
 */
bool Motor_PositionCore_Enable(motor_position_runtime_t *runtime, bool enable, bool ready)
{
    if (runtime == NULL || (enable && !ready)) return false;
    runtime->state.enabled = enable;
    if (!enable) Motor_PositionCore_ClearHold(runtime);
    return true;
}

/**
 * @description: 设置绝对目标位置。
 * @detail: 目标变化时复位速度积分。
 * @param {motor_position_runtime_t *} runtime  运行状态
 * @param {float} target_deg  目标 (deg)，须为有限值
 * @return {bool} 写入成功返回 true
 */
bool Motor_PositionCore_SetTarget(motor_position_runtime_t *runtime, float target_deg)
{
    if (runtime == NULL || !isfinite(target_deg)) return false;
    if (target_deg != runtime->state.target_deg) Motor_PositionControl_Reset(&runtime->control);
    runtime->state.target_deg = target_deg;
    return true;
}

/**
 * @description: 在当前目标上叠加一段位移。
 * @param {motor_position_runtime_t *} runtime  运行状态
 * @param {float} delta_deg  增量 (deg)，须为有限值
 * @return {bool} 相加后目标仍为有限值返回 true
 */
bool Motor_PositionCore_Move(motor_position_runtime_t *runtime, float delta_deg)
{
    float target;
    if (runtime == NULL || !isfinite(delta_deg)) return false;
    target = runtime->state.target_deg + delta_deg;
    if (!isfinite(target)) return false;
    runtime->state.target_deg = target;
    Motor_PositionControl_Reset(&runtime->control);
    return true;
}

/**
 * @description: 设置位置比例。
 * @param {motor_position_runtime_t *} runtime  运行状态
 * @param {float} kp  比例 (RPM/deg)，范围 (0, 100]
 * @return {bool} 写入成功返回 true
 */
bool Motor_PositionCore_SetKp(motor_position_runtime_t *runtime, float kp)
{
    if (runtime == NULL || !isfinite(kp) || kp <= 0.0f || kp > 100.0f) return false;
    runtime->state.kp_rpm_per_deg = kp;
    return true;
}

/**
 * @description: 设置位置环给出的速度上限。
 * @param {motor_position_runtime_t *} runtime  运行状态
 * @param {float} rpm         速度上限 (RPM)
 * @param {float} allowed_rpm 允许的电机上限 (RPM)
 * @return {bool} 写入成功返回 true
 */
bool Motor_PositionCore_SetMaxRpm(motor_position_runtime_t *runtime, float rpm, float allowed_rpm)
{
    if (runtime == NULL || !isfinite(rpm) || !isfinite(allowed_rpm) || rpm < 1.0f || rpm > allowed_rpm) return false;
    runtime->state.max_rpm = rpm;
    return true;
}

/**
 * @description: 推进一拍位置控制与保持模式切换。
 * @detail: 输出关断和写 PWM 由调用者完成。初始化后调用，输入须有限。
 *          控制配置非法时返回 false 并释放保持，命令所有权由 enabled 决定。
 * @param {motor_position_runtime_t *} runtime  运行状态，不可为空
 * @param {const motor_position_config_t *} config  配置，不可为空
 * @param {const motor_position_input_t *} input  本拍输入，不可为空
 * @param {motor_position_output_t *} output  本拍输出，不可为空
 * @return {bool} 本拍计算完成返回 true
 */
bool Motor_PositionCore_Step(motor_position_runtime_t *runtime, const motor_position_config_t *config,
                             const motor_position_input_t *input, motor_position_output_t *output)
{
    motor_position_control_config_t control_config;
    motor_position_control_input_t control_input;
    motor_position_control_output_t control_output;
    if (runtime == NULL || config == NULL || input == NULL || output == NULL || config->counts_per_rev == 0U) return false;
    memset(output, 0, sizeof(*output));
    output->owns_speed_command = runtime->state.enabled;
    runtime->state.position_deg = PositionDeg(runtime, config, input->accumulated_count);
    if (!runtime->state.enabled) {
        Motor_PositionCore_ClearHold(runtime);
        return true;
    }
    control_config = config->control;
    control_config.position_kp_rpm_per_deg = runtime->state.kp_rpm_per_deg;
    control_config.maximum_rpm = runtime->state.max_rpm;
    control_input.enabled = runtime->state.enabled;
    control_input.motor_running = input->motor_running;
    control_input.feedback_valid = input->feedback_valid;
    control_input.speed_valid = input->speed_valid;
    control_input.target_deg = runtime->state.target_deg;
    control_input.position_deg = runtime->state.position_deg;
    control_input.measured_rpm = input->speed_valid ? input->measured_rpm : 0.0f;
    control_input.dt = input->dt;
    if (!Motor_PositionControl_Step(&runtime->control, &control_config, &control_input, &control_output)) {
        Motor_PositionCore_ClearHold(runtime);
        output->owns_speed_command = false; /* 原入口失败时只释放保持，不覆盖速度目标。 */
        return false;
    }
    runtime->state.error_deg = control_output.error_deg;
    runtime->state.target_rpm = control_output.target_rpm;
    output->hold_active = control_output.hold_active;
    if (config->hybrid_speed_enabled && output->hold_active) {
        float error = fabsf(control_output.error_deg);
        if (runtime->state.voltage_hold_mode) {
            if (error > config->hold_exit_deg) runtime->state.voltage_hold_mode = false;
        } else if (error <= config->hold_enter_deg) {
            runtime->state.voltage_hold_mode = true;
        }
    } else {
        runtime->state.voltage_hold_mode = true;
    }
    output->voltage_hold = output->hold_active && runtime->state.voltage_hold_mode;
    if (output->voltage_hold) output->voltage_v = control_output.output_v;
    else if (output->hold_active) {
        Motor_PositionControl_Reset(&runtime->control);
        output->speed_rpm = control_output.target_rpm;
    }
    return true;
}

/**
 * @description: 只刷新返回快照，不推进积分。
 * @detail: config 与 runtime 须已初始化。
 * @param {const motor_position_runtime_t *} runtime  运行状态，不可为空
 * @param {const motor_position_config_t *} config  配置，不可为空
 * @param {uint32_t} count  当前累计计数
 * @param {bool} feedback_valid  true=位置反馈可用
 * @param {motor_position_state_t *} snapshot  输出快照，不可为空
 * @return {void} 无
 */
void Motor_PositionCore_Snapshot(const motor_position_runtime_t *runtime, const motor_position_config_t *config,
                                 uint32_t count, bool feedback_valid, motor_position_state_t *snapshot)
{
    COMMON_ASSERT(runtime != NULL && config != NULL && snapshot != NULL);
    if (config->counts_per_rev == 0U) return;
    *snapshot = runtime->state;
    snapshot->position_deg = PositionDeg(runtime, config, count);
    snapshot->error_deg = snapshot->target_deg - snapshot->position_deg;
    snapshot->target_rpm = feedback_valid ? fmaxf(-snapshot->max_rpm,
        fminf(snapshot->kp_rpm_per_deg * snapshot->error_deg, snapshot->max_rpm)) : 0.0f;
}
#endif /* MOTOR_CFG_POSITION */
