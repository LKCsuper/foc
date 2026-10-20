/*
 * @Description: 关节级联控制（位置/速度外环生成Iq，电流内环生成电压）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "common_assert.h"
#include <stddef.h>
#include <string.h>
#include "motor_joint.h"
#include "motor_joint_control.h"
#include "motor_trajectory.h"
#include "motor_encoder.h"
#include "motor_mt6701.h"
#include "motor_param.h"
#include "motor_crit.h"
#include "hal_motor.h"
#if MOTOR_CFG_JOINT
/* Private define ------------------------------------------------------------*/
#define MOTOR_JOINT_KP_DEFAULT_V_PER_DEG (0.15f)
#define MOTOR_JOINT_KD_DEFAULT_V_PER_RPM (0.02f)
#define MOTOR_JOINT_KI_DEFAULT_V_PER_DEG_S (0.15f)
#define MOTOR_JOINT_V_LIMIT_DEFAULT_V    (2.0f)
#define MOTOR_JOINT_KP_MAX_V_PER_DEG     (2.0f)
#define MOTOR_JOINT_KD_MAX_V_PER_RPM     (0.20f)
#define MOTOR_JOINT_KI_MAX_V_PER_DEG_S   (1.0f)
#define MOTOR_JOINT_BIAS_MAX_V           (0.30f)
#define MOTOR_JOINT_V_LIMIT_MIN_V        (0.20f)
#define MOTOR_JOINT_V_LIMIT_MAX_V        (6.0f)
#define MOTOR_JOINT_TORQUE_FF_MAX_A      (1.0f)
#define MOTOR_JOINT_DT_MAX_S             (0.05f)
#define MOTOR_JOINT_INTEGRAL_MAX_ERROR_DEG (2.0f)
#define MOTOR_JOINT_INTEGRAL_MAX_RPM      (1.0f)
#define MOTOR_JOINT_INTEGRAL_MAX_TARGET_RPM (0.01f)
#define MOTOR_JOINT_TARGET_CHANGE_DEG     (0.01f)
#define MOTOR_JOINT_DEG_TO_RAD            (0.01745329251994329577f)
#define MOTOR_JOINT_RAD_TO_DEG            (57.295779513082320876f)
#define MOTOR_JOINT_RPM_TO_RAD_S          (0.10471975511965977462f)
#define MOTOR_JOINT_RAD_S_TO_RPM          (9.5492965855137201461f)
#define MOTOR_JOINT_TIMEOUT_DEFAULT_US     (100000U)
#define MOTOR_JOINT_TIMEOUT_MIN_US         (1000U)
#define MOTOR_JOINT_TIMEOUT_MAX_US         (1000000U)
#define MOTOR_JOINT_MOTION_FF_A            (0.03f)
#define MOTOR_JOINT_MOTION_FF_MIN_RPM      (0.05f)
#define MOTOR_JOINT_HYBRID_IQ_ENTER_A       (0.25f)
#define MOTOR_JOINT_HYBRID_IQ_EXIT_A        (0.20f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_joint_state_t s_joint;
static int32_t s_joint_origin;
static motor_joint_control_t s_control;
static uint32_t s_joint_last_command_us;
static uint32_t s_joint_timeout_us;
static float s_joint_torque_kd_nm_per_rad_s;
static float s_joint_torque_current_limit_a;
static float s_joint_torque_to_iq;
static motor_trajectory_t s_joint_trajectory;
static bool s_joint_trajectory_enabled;
static float s_joint_trajectory_rpm;
/* Private function prototypes -----------------------------------------------*/
static float Motor_Joint_ReadDeg(void);
static void Motor_Joint_ClearHold(void);
static bool Motor_Joint_ApplyControlConfig(void);
static bool Motor_Joint_ApplyControlCommand(void);
static bool Motor_Joint_ApplyControlSetpoint(float position_deg, float velocity_rpm);
static float Motor_Joint_LimitOutput(float value, float limit);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 用累计机械计数换算相对关节零点的角度。
 * @return {float} 当前位置 (deg)
 * @author: lkc
 */
static float Motor_Joint_ReadDeg(void)
{
    int32_t delta = (int32_t)(Motor_Encoder_GetAccumulatedCount() - (uint32_t)s_joint_origin);
    return (float)delta * 360.0f / (float)MOTOR_MT6701_COUNTS_PER_REV;
}

/**
 * @description: 退出阻抗输出，清掉电压标志。
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Joint_ClearHold(void)
{
    Motor_JointControl_Reset(&s_control);
    s_joint.hybrid_current_active = false;
    s_joint.hybrid_enter_count = 0U;
    s_joint.hybrid_exit_count = 0U;
    s_joint.hybrid_peak_iq_a = 0.0f;
    s_joint.output_v = 0.0f;
    s_joint.bias_v = 0.0f;
    motor.joint_voltage_hold = false;
    motor.joint_hold_v = 0.0f;
    motor.joint_current_hold = false;
    motor.joint_hold_iq = 0.0f;
}

/**
 * @description: 把兼容Shell使用的deg/RPM/V参数换成纯算法的rad、rad/s和输出单位。
 * @return {bool} true=配置有效并已应用
 */
static bool Motor_Joint_ApplyControlConfig(void)
{
    motor_joint_control_config_t config;
    float volts_per_amp = MOTOR_ENCODER_SPEED_VOLT_PER_AMP;

    if (!(volts_per_amp > 0.0f)) return false;

    /* 保持原Shell调参刻度不变，但外环输出由等效电压换算成Iq。 */
    config.kp_output_per_rad = s_joint.kp_v_per_deg * MOTOR_JOINT_RAD_TO_DEG / volts_per_amp;
    config.kd_output_per_rad_s = s_joint.kd_v_per_rpm * MOTOR_JOINT_RAD_S_TO_RPM / volts_per_amp;
    config.ki_output_per_rad_s = s_joint.ki_v_per_deg_s * MOTOR_JOINT_RAD_TO_DEG / volts_per_amp;
    config.output_limit = s_joint.v_limit_v / volts_per_amp;
    config.integral_limit = MOTOR_JOINT_BIAS_MAX_V / volts_per_amp;
    config.integral_max_error_rad = MOTOR_JOINT_INTEGRAL_MAX_ERROR_DEG * MOTOR_JOINT_DEG_TO_RAD;
    config.integral_max_velocity_rad_s = MOTOR_JOINT_INTEGRAL_MAX_RPM * MOTOR_JOINT_RPM_TO_RAD_S;
    config.integral_max_target_rad_s = MOTOR_JOINT_INTEGRAL_MAX_TARGET_RPM * MOTOR_JOINT_RPM_TO_RAD_S;
    config.target_reset_threshold_rad = MOTOR_JOINT_TARGET_CHANGE_DEG * MOTOR_JOINT_DEG_TO_RAD;
    config.dt_max_s = MOTOR_JOINT_DT_MAX_S;
    return Motor_JointControl_Configure(&s_control, &config);
}

/**
 * @description: 把兼容Shell命令换成纯算法命令并整体提交。
 * @return {bool} true=命令有效并已应用
 */
static bool Motor_Joint_ApplyControlCommand(void)
{
    return Motor_Joint_ApplyControlSetpoint(s_joint.target_deg, s_joint.target_rpm);
}

static bool Motor_Joint_ApplyControlSetpoint(float position_deg, float velocity_rpm)
{
    motor_joint_control_command_t command;

    command.position_rad = position_deg * MOTOR_JOINT_DEG_TO_RAD;
    command.velocity_rad_s = velocity_rpm * MOTOR_JOINT_RPM_TO_RAD_S;
    command.feedforward_output = s_joint.torque_ff_a;
    return Motor_JointControl_SetCommand(&s_control, &command);
}

/**
 * @description: 把数值限制在对称区间内。
 * @param {float} value  输入
 * @param {float} limit  绝对值上限
 * @return {float} 限幅后的值
 * @author: lkc
 */
static float Motor_Joint_LimitOutput(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/**
 * @description: 用当前累计计数作为零点，并装入默认刚度、阻尼和电压上限。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Init(void)
{
    motor_joint_control_config_t config;

    s_joint_origin = (int32_t)Motor_Encoder_GetAccumulatedCount();
    s_joint.enabled = false;
    s_joint.target_deg = 0.0f;
    s_joint.position_deg = 0.0f;
    s_joint.target_rpm = 0.0f;
    s_joint.planned_position_deg = 0.0f;
    s_joint.planned_rpm = 0.0f;
    s_joint.trajectory_active = false;
    s_joint.measured_rpm = 0.0f;
    s_joint.torque_ff_a = 0.0f;
    s_joint.kp_v_per_deg = MOTOR_JOINT_KP_DEFAULT_V_PER_DEG;
    s_joint.kd_v_per_rpm = MOTOR_JOINT_KD_DEFAULT_V_PER_RPM;
    s_joint.ki_v_per_deg_s = MOTOR_JOINT_KI_DEFAULT_V_PER_DEG_S;
    s_joint.bias_v = 0.0f;
    s_joint.v_limit_v = MOTOR_JOINT_V_LIMIT_DEFAULT_V;
    s_joint.timeout_enabled = false;
    s_joint.command_timeout = false;
    s_joint.command_age_us = 0U;
    s_joint.speed_valid = false;
    s_joint.encoder_fault = false;
    s_joint.backend = MOTOR_JOINT_BACKEND_POSITION_CURRENT;
    s_joint.torque_command_nm = 0.0f;
    s_joint.iq_command_a = 0.0f;
    s_joint_timeout_us = MOTOR_JOINT_TIMEOUT_DEFAULT_US;
    s_joint_last_command_us = Hal_Motor_TimeUs();
    memset(&config, 0, sizeof(config));
    config.output_limit = 1.0f;
    config.dt_max_s = MOTOR_JOINT_DT_MAX_S;
    (void)Motor_JointControl_Init(&s_control, &config);
    (void)Motor_Joint_ApplyControlConfig();
    s_joint_torque_current_limit_a = (MOTOR_ENCODER_SPEED_VOLT_PER_AMP > 0.0f)
                                         ? s_joint.v_limit_v / MOTOR_ENCODER_SPEED_VOLT_PER_AMP
                                         : 0.0f;
    (void)Motor_Joint_ApplyControlCommand();
    (void)Motor_Trajectory_Init(&s_joint_trajectory, 0.0f, 0.0f);
    s_joint_trajectory_enabled = false;
    s_joint_trajectory_rpm = 0.0f;
    Motor_Joint_ClearHold();
}

/**
 * @description: 把当前位置记为零，并清掉目标与前馈。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Zero(void)
{
    uint32_t primask = Motor_Crit_Enter();

    s_joint_origin = (int32_t)Motor_Encoder_GetAccumulatedCount();
    s_joint.target_deg = 0.0f;
    s_joint.position_deg = 0.0f;
    s_joint.target_rpm = 0.0f;
    s_joint.planned_position_deg = 0.0f;
    s_joint.planned_rpm = 0.0f;
    s_joint.trajectory_active = false;
    s_joint.torque_ff_a = 0.0f;
    s_joint.output_v = 0.0f;
    s_joint.bias_v = 0.0f;
    s_joint.torque_command_nm = 0.0f;
    s_joint.iq_command_a = 0.0f;
    s_joint.backend = MOTOR_JOINT_BACKEND_POSITION_CURRENT;
    s_joint.hybrid_current_active = false;
    (void)Motor_Joint_ApplyControlConfig();
    (void)Motor_Joint_ApplyControlCommand();
    Motor_JointControl_Reset(&s_control);
    (void)Motor_Trajectory_Init(&s_joint_trajectory, 0.0f, 0.0f);
    s_joint_trajectory_enabled = false;
    s_joint_last_command_us = Hal_Motor_TimeUs();
    s_joint.command_timeout = false;
    Motor_Crit_Exit(primask);
}

/**
 * @description: 开关关节阻抗。
 * @param {bool} enable  true=使能
 * @return {bool} true=已设置；false=编码器未就绪
 * @author: lkc
 */
bool Motor_Joint_Enable(bool enable)
{
    uint32_t primask;

    if (enable && (!Motor_Encoder_IsZeroValid() || Motor_Encoder_IsSignalFault())) return false;
    primask = Motor_Crit_Enter();
    s_joint.enabled = enable;
    s_joint_last_command_us = Hal_Motor_TimeUs();
    s_joint.command_timeout = false;
    if (!enable) {
        Motor_Joint_ClearHold();
    }
    Motor_Crit_Exit(primask);
    return true;
}

/**
 * @description: 设置位置、速度和力矩前馈。
 * @param {float} target_deg  相对零点的目标 (deg)
 * @param {float} target_rpm  目标转速 (RPM)
 * @param {float} torque_ff_a  力矩前馈 (A)
 * @return {bool} true=三个量都有限且前馈在允许范围内
 * @author: lkc
 */
bool Motor_Joint_SetTarget(float target_deg, float target_rpm, float torque_ff_a)
{
    float old_target_deg;
    float old_target_rpm;
    float old_torque_ff_a;
    uint32_t primask;

    if (!isfinite(target_deg) || !isfinite(target_rpm) || !isfinite(torque_ff_a)) return false;
    if (fabsf(torque_ff_a) > MOTOR_JOINT_TORQUE_FF_MAX_A) return false;
    primask = Motor_Crit_Enter();
    old_target_deg = s_joint.target_deg;
    old_target_rpm = s_joint.target_rpm;
    old_torque_ff_a = s_joint.torque_ff_a;
    s_joint.target_deg = target_deg;
    s_joint.target_rpm = target_rpm;
    s_joint.torque_ff_a = torque_ff_a;
    s_joint.backend = MOTOR_JOINT_BACKEND_POSITION_CURRENT;
    /* 兼容Shell目标是本地保持命令，不应继承通信/Iq测试留下的超时看门狗。 */
    s_joint.timeout_enabled = false;
    if ((MOTOR_JOINT_TRAJECTORY_ENABLE != 0U) && target_rpm == 0.0f) {
        if (!s_joint_trajectory_enabled) {
            (void)Motor_Trajectory_Init(&s_joint_trajectory, s_joint.position_deg, s_joint.measured_rpm * 6.0f);
        }
        s_joint_trajectory_enabled = Motor_Trajectory_SetGoal(&s_joint_trajectory, target_deg);
        s_joint.planned_position_deg = s_joint_trajectory.position;
        s_joint.planned_rpm = s_joint_trajectory.velocity / 6.0f;
        s_joint.trajectory_active = s_joint_trajectory.active;
    } else {
        s_joint_trajectory_enabled = false;
        s_joint.trajectory_active = false;
        s_joint.planned_position_deg = target_deg;
        s_joint.planned_rpm = target_rpm;
    }
    if (!Motor_Joint_ApplyControlConfig() ||
        !Motor_Joint_ApplyControlSetpoint(s_joint.planned_position_deg, s_joint.planned_rpm)) {
        s_joint.target_deg = old_target_deg;
        s_joint.target_rpm = old_target_rpm;
        s_joint.torque_ff_a = old_torque_ff_a;
        (void)Motor_Joint_ApplyControlCommand();
        Motor_Crit_Exit(primask);
        return false;
    }
    s_joint.bias_v = s_control.integral_output;
    s_joint_last_command_us = Hal_Motor_TimeUs();
    s_joint.command_timeout = false;
    Motor_Crit_Exit(primask);
    return true;
}

/**
 * @description: 校验并整体提交一帧 SI 单位关节命令。
 * @detail: 所有字段在同一短临界区更新；成功提交会刷新超时计时。当前不写 Flash。
 * @param {const motor_joint_command_si_t *} command  位置 (rad)、速度 (rad/s)、前馈 (V) 和增益
 * @return {bool} true=命令合法并已提交
 * @author: lkc
 */
bool Motor_Joint_SubmitCommand(const motor_joint_command_si_t *command)
{
    motor_joint_state_t old_state;
    uint32_t old_timeout_us;
    uint32_t primask;
    uint32_t now_us;
    float feedforward_limit_v = MOTOR_JOINT_TORQUE_FF_MAX_A * MOTOR_ENCODER_SPEED_VOLT_PER_AMP;

    if (command == NULL || !isfinite(command->position_rad) || !isfinite(command->velocity_rad_s) ||
        !isfinite(command->feedforward_v) || !isfinite(command->kp_v_per_rad) ||
        !isfinite(command->kd_v_per_rad_s) || !isfinite(command->ki_v_per_rad_s) ||
        !isfinite(command->output_limit_v)) {
        return false;
    }
    if (command->kp_v_per_rad <= 0.0f ||
        command->kp_v_per_rad > MOTOR_JOINT_KP_MAX_V_PER_DEG * MOTOR_JOINT_RAD_TO_DEG ||
        command->kd_v_per_rad_s < 0.0f ||
        command->kd_v_per_rad_s > MOTOR_JOINT_KD_MAX_V_PER_RPM * MOTOR_JOINT_RAD_S_TO_RPM ||
        command->ki_v_per_rad_s < 0.0f ||
        command->ki_v_per_rad_s > MOTOR_JOINT_KI_MAX_V_PER_DEG_S * MOTOR_JOINT_RAD_TO_DEG ||
        command->output_limit_v < MOTOR_JOINT_V_LIMIT_MIN_V ||
        command->output_limit_v > MOTOR_JOINT_V_LIMIT_MAX_V || fabsf(command->feedforward_v) > feedforward_limit_v) {
        return false;
    }
    if (command->timeout_enable && command->timeout_us != 0U &&
        (command->timeout_us < MOTOR_JOINT_TIMEOUT_MIN_US || command->timeout_us > MOTOR_JOINT_TIMEOUT_MAX_US)) {
        return false;
    }
    if (command->enable && (!Motor_Encoder_IsZeroValid() || Motor_Encoder_IsSignalFault())) return false;

    now_us = Hal_Motor_TimeUs();
    primask = Motor_Crit_Enter();
    old_state = s_joint;
    old_timeout_us = s_joint_timeout_us;
    s_joint.enabled = command->enable;
    s_joint.backend = MOTOR_JOINT_BACKEND_POSITION_CURRENT;
    s_joint_trajectory_enabled = false;
    s_joint.target_deg = command->position_rad * MOTOR_JOINT_RAD_TO_DEG;
    s_joint.target_rpm = command->velocity_rad_s * MOTOR_JOINT_RAD_S_TO_RPM;
    s_joint.torque_ff_a = (MOTOR_ENCODER_SPEED_VOLT_PER_AMP > 0.0f)
                             ? command->feedforward_v / MOTOR_ENCODER_SPEED_VOLT_PER_AMP
                             : 0.0f;
    s_joint.kp_v_per_deg = command->kp_v_per_rad * MOTOR_JOINT_DEG_TO_RAD;
    s_joint.kd_v_per_rpm = command->kd_v_per_rad_s * MOTOR_JOINT_RPM_TO_RAD_S;
    s_joint.ki_v_per_deg_s = command->ki_v_per_rad_s * MOTOR_JOINT_DEG_TO_RAD;
    s_joint.v_limit_v = command->output_limit_v;
    s_joint_torque_current_limit_a = (MOTOR_ENCODER_SPEED_VOLT_PER_AMP > 0.0f)
                                         ? command->output_limit_v / MOTOR_ENCODER_SPEED_VOLT_PER_AMP
                                         : 0.0f;
    s_joint.timeout_enabled = command->timeout_enable;
    s_joint_timeout_us = command->timeout_us == 0U ? MOTOR_JOINT_TIMEOUT_DEFAULT_US : command->timeout_us;
    if (!Motor_Joint_ApplyControlConfig() || !Motor_Joint_ApplyControlCommand()) {
        s_joint = old_state;
        s_joint_timeout_us = old_timeout_us;
        (void)Motor_Joint_ApplyControlConfig();
        (void)Motor_Joint_ApplyControlCommand();
        Motor_Crit_Exit(primask);
        return false;
    }
    s_joint_last_command_us = now_us;
    s_joint.command_age_us = 0U;
    s_joint.command_timeout = false;
    if (!s_joint.enabled) {
        Motor_Joint_ClearHold();
    }
    Motor_Crit_Exit(primask);
    return true;
}

/**
 * @description: 整体提交物理单位扭矩命令，并切换到 Iq 电流环后端。
 * @detail: torque = ff + kp*位置误差 + kd*速度误差；Iq=torque/(Kt*ratio*efficiency)。
 * @param {const motor_joint_torque_command_t *} command  位置 (rad)、转矩 (N·m) 和 Kt (N·m/A)
 * @return {bool} 参数、编码器状态与限幅全部有效时返回 true
 * @author: lkc
 */
bool Motor_Joint_SubmitTorqueCommand(const motor_joint_torque_command_t *command)
{
    motor_joint_control_config_t config;
    motor_joint_control_command_t core_command;
    uint32_t primask;
    uint32_t now_us;
    float effective_kt;

    if (command == NULL || !isfinite(command->position_rad) || !isfinite(command->velocity_rad_s) ||
        !isfinite(command->torque_ff_nm) || !isfinite(command->kp_nm_per_rad) ||
        !isfinite(command->kd_nm_per_rad_s) || !isfinite(command->torque_limit_nm) ||
        !isfinite(command->current_limit_a) || !isfinite(command->torque_constant_nm_per_a) ||
        !isfinite(command->gear_ratio) || !isfinite(command->efficiency)) {
        return false;
    }
    if (command->kp_nm_per_rad < 0.0f || command->kp_nm_per_rad > 100.0f ||
        command->kd_nm_per_rad_s < 0.0f || command->kd_nm_per_rad_s > 10.0f ||
        command->torque_limit_nm <= 0.0f || command->torque_limit_nm > 100.0f ||
        fabsf(command->torque_ff_nm) > command->torque_limit_nm || command->current_limit_a <= 0.0f ||
        command->current_limit_a > MOTOR_FIXED_PEAK_CURRENT || command->torque_constant_nm_per_a < 0.001f ||
        command->torque_constant_nm_per_a > 10.0f || command->gear_ratio < 1.0f ||
        command->gear_ratio > 1000.0f || command->efficiency <= 0.0f || command->efficiency > 1.0f) {
        return false;
    }
    if (command->timeout_enable && command->timeout_us != 0U &&
        (command->timeout_us < MOTOR_JOINT_TIMEOUT_MIN_US || command->timeout_us > MOTOR_JOINT_TIMEOUT_MAX_US)) {
        return false;
    }
    if (command->enable && (!Motor_Encoder_IsZeroValid() || Motor_Encoder_IsSignalFault())) return false;
    effective_kt = command->torque_constant_nm_per_a * command->gear_ratio * command->efficiency;
    if (!isfinite(effective_kt) || effective_kt <= 0.0f) return false;

    config.kp_output_per_rad = command->kp_nm_per_rad;
    config.kd_output_per_rad_s = command->kd_nm_per_rad_s;
    config.ki_output_per_rad_s = 0.0f;
    config.output_limit = command->torque_limit_nm;
    config.integral_limit = 0.0f;
    config.integral_max_error_rad = 0.0f;
    config.integral_max_velocity_rad_s = 0.0f;
    config.integral_max_target_rad_s = 0.0f;
    config.target_reset_threshold_rad = MOTOR_JOINT_TARGET_CHANGE_DEG * MOTOR_JOINT_DEG_TO_RAD;
    config.dt_max_s = MOTOR_JOINT_DT_MAX_S;
    core_command.position_rad = command->position_rad;
    core_command.velocity_rad_s = command->velocity_rad_s;
    core_command.feedforward_output = command->torque_ff_nm;

    now_us = Hal_Motor_TimeUs();
    primask = Motor_Crit_Enter();
    if (!Motor_JointControl_Configure(&s_control, &config) ||
        !Motor_JointControl_SetCommand(&s_control, &core_command)) {
        Motor_Crit_Exit(primask);
        return false;
    }
    s_joint.enabled = command->enable;
    s_joint.backend = MOTOR_JOINT_BACKEND_TORQUE_CURRENT;
    s_joint_trajectory_enabled = false;
    s_joint.hybrid_current_active = true;
    s_joint.target_deg = command->position_rad * MOTOR_JOINT_RAD_TO_DEG;
    s_joint.target_rpm = command->velocity_rad_s * MOTOR_JOINT_RAD_S_TO_RPM;
    s_joint.torque_ff_a = 0.0f;
    s_joint.timeout_enabled = command->timeout_enable;
    s_joint_timeout_us = command->timeout_us == 0U ? MOTOR_JOINT_TIMEOUT_DEFAULT_US : command->timeout_us;
    s_joint_torque_kd_nm_per_rad_s = command->kd_nm_per_rad_s;
    s_joint_torque_current_limit_a = command->current_limit_a;
    s_joint_torque_to_iq = 1.0f / effective_kt;
    s_joint_last_command_us = now_us;
    s_joint.command_age_us = 0U;
    s_joint.command_timeout = false;
    s_joint.torque_command_nm = 0.0f;
    s_joint.iq_command_a = 0.0f;
    if (!s_joint.enabled) {
        Motor_Joint_ClearHold();
    }
    Motor_Crit_Exit(primask);
    return true;
}

/**
 * @description: 提交直接 Iq 命令，供受限的电流环验证使用。
 * @detail: 不做扭矩换算；必须有有效编码器，且电流不超过显式限制和电机峰值。
 * @param {const motor_joint_current_command_t *} command  目标电流 (A) 和电流上限 (A)
 * @return {bool} true=电流有限且已进入直接电流后端
 * @author: lkc
 */
bool Motor_Joint_SubmitCurrentCommand(const motor_joint_current_command_t *command)
{
    motor_joint_control_config_t config;
    motor_joint_control_command_t core_command;
    uint32_t primask;
    uint32_t now_us;

    if (command == NULL || !isfinite(command->current_a) || !isfinite(command->current_limit_a) ||
        command->current_limit_a <= 0.0f || command->current_limit_a > MOTOR_FIXED_PEAK_CURRENT ||
        fabsf(command->current_a) > command->current_limit_a) {
        return false;
    }
    if (command->timeout_enable && command->timeout_us != 0U &&
        (command->timeout_us < MOTOR_JOINT_TIMEOUT_MIN_US || command->timeout_us > MOTOR_JOINT_TIMEOUT_MAX_US)) {
        return false;
    }
    if (command->enable && (!Motor_Encoder_IsZeroValid() || Motor_Encoder_IsSignalFault())) return false;

    memset(&config, 0, sizeof(config));
    config.output_limit = command->current_limit_a;
    config.target_reset_threshold_rad = MOTOR_JOINT_TARGET_CHANGE_DEG * MOTOR_JOINT_DEG_TO_RAD;
    config.dt_max_s = MOTOR_JOINT_DT_MAX_S;
    memset(&core_command, 0, sizeof(core_command));
    core_command.feedforward_output = command->current_a;

    now_us = Hal_Motor_TimeUs();
    primask = Motor_Crit_Enter();
    if (!Motor_JointControl_Configure(&s_control, &config) ||
        !Motor_JointControl_SetCommand(&s_control, &core_command)) {
        Motor_Crit_Exit(primask);
        return false;
    }
    s_joint.enabled = command->enable;
    s_joint.backend = MOTOR_JOINT_BACKEND_CURRENT_DIRECT;
    s_joint_trajectory_enabled = false;
    s_joint.hybrid_current_active = true;
    s_joint.target_deg = s_joint.position_deg;
    s_joint.target_rpm = 0.0f;
    s_joint.torque_ff_a = command->current_a;
    s_joint.timeout_enabled = command->timeout_enable;
    s_joint_timeout_us = command->timeout_us == 0U ? MOTOR_JOINT_TIMEOUT_DEFAULT_US : command->timeout_us;
    s_joint_torque_current_limit_a = command->current_limit_a;
    s_joint_last_command_us = now_us;
    s_joint.command_age_us = 0U;
    s_joint.command_timeout = false;
    s_joint.torque_command_nm = 0.0f;
    s_joint.iq_command_a = command->current_a;
    if (!s_joint.enabled) {
        Motor_Joint_ClearHold();
    }
    Motor_Crit_Exit(primask);
    return true;
}

/**
 * @description: 设置刚度。
 * @param {float} kp_v_per_deg  刚度 (V/deg)，范围 (0, 2]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetKp(float kp_v_per_deg)
{
    uint32_t primask;
    bool result;

    if (!isfinite(kp_v_per_deg) || kp_v_per_deg <= 0.0f || kp_v_per_deg > MOTOR_JOINT_KP_MAX_V_PER_DEG) return false;
    primask = Motor_Crit_Enter();
    s_joint.backend = MOTOR_JOINT_BACKEND_POSITION_CURRENT;
    s_joint.kp_v_per_deg = kp_v_per_deg;
    result = Motor_Joint_ApplyControlConfig();
    Motor_Crit_Exit(primask);
    return result;
}

/**
 * @description: 设置阻尼。
 * @param {float} kd_v_per_rpm  阻尼 (V/RPM)，范围 [0, 0.2]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetKd(float kd_v_per_rpm)
{
    uint32_t primask;
    bool result;

    if (!isfinite(kd_v_per_rpm) || kd_v_per_rpm < 0.0f || kd_v_per_rpm > MOTOR_JOINT_KD_MAX_V_PER_RPM) return false;
    primask = Motor_Crit_Enter();
    s_joint.backend = MOTOR_JOINT_BACKEND_POSITION_CURRENT;
    s_joint.kd_v_per_rpm = kd_v_per_rpm;
    result = Motor_Joint_ApplyControlConfig();
    Motor_Crit_Exit(primask);
    return result;
}

/**
 * @description: 设置停住补偿的慢积分。
 * @param {float} ki_v_per_deg_s  积分增益 (V/deg/s)，范围 [0, 1]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetKi(float ki_v_per_deg_s)
{
    uint32_t primask;
    bool result;

    if (!isfinite(ki_v_per_deg_s) || ki_v_per_deg_s < 0.0f || ki_v_per_deg_s > MOTOR_JOINT_KI_MAX_V_PER_DEG_S) return false;
    primask = Motor_Crit_Enter();
    s_joint.backend = MOTOR_JOINT_BACKEND_POSITION_CURRENT;
    s_joint.ki_v_per_deg_s = ki_v_per_deg_s;
    if (ki_v_per_deg_s == 0.0f) {
        Motor_JointControl_Reset(&s_control);
    }
    s_joint.bias_v = s_control.integral_output;
    result = Motor_Joint_ApplyControlConfig();
    Motor_Crit_Exit(primask);
    return result;
}

/**
 * @description: 设置合成电压绝对值上限。
 * @param {float} limit_v  上限 (V)，范围 [0.2, 6]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetVoltageLimit(float limit_v)
{
    uint32_t primask;
    bool result;

    if (!isfinite(limit_v) || limit_v < MOTOR_JOINT_V_LIMIT_MIN_V || limit_v > MOTOR_JOINT_V_LIMIT_MAX_V) return false;
    primask = Motor_Crit_Enter();
    s_joint.backend = MOTOR_JOINT_BACKEND_POSITION_CURRENT;
    s_joint.v_limit_v = limit_v;
    s_joint_torque_current_limit_a = (MOTOR_ENCODER_SPEED_VOLT_PER_AMP > 0.0f)
                                         ? limit_v / MOTOR_ENCODER_SPEED_VOLT_PER_AMP
                                         : 0.0f;
    result = Motor_Joint_ApplyControlConfig();
    Motor_Crit_Exit(primask);
    return result;
}

/**
 * @description: 临时改关节轨迹最高转速，加减速按同样比例缩放。
 * @detail: rpm 小于等于 0 或非有限时恢复头文件默认。不写 Flash。
 * @param {float} rpm  最高转速 (RPM)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_SetTrajectoryRpm(float rpm)
{
    uint32_t primask;

    if (!isfinite(rpm) || rpm <= 0.0f) {
        rpm = 0.0f;
    }
    primask = Motor_Crit_Enter();
    s_joint_trajectory_rpm = rpm;
    Motor_Crit_Exit(primask);
}

/**
 * @description: 读取当前关节轨迹最高转速。
 * @return {float} 最高转速 (RPM)。未临时改过时返回头文件默认
 * @author: lkc
 */
float Motor_Joint_GetTrajectoryRpm(void)
{
    float rpm = s_joint_trajectory_rpm;

    if (rpm > 0.0f) return rpm;
    return MOTOR_JOINT_TRAJECTORY_VEL_DEG_S / 6.0f;
}

/**
 * @description: 推进一拍位置/速度外环，把受限Iq目标交给电流内环。
 * @param {float} dt  调用周期 (s)。慢积分用它累加，大于 0.05 s 时本拍不积分
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Run(float dt)
{
    motor_joint_control_feedback_t feedback;
    float measured_rpm = 0.0f;
    float outer_output;
    bool speed_valid;
    uint32_t now_us;

    s_joint.position_deg = Motor_Joint_ReadDeg();
    speed_valid = Motor_Encoder_IsSpeedValid();
    if (speed_valid) {
        measured_rpm = Motor_Encoder_GetMechRpm();
    }
    s_joint.measured_rpm = measured_rpm;
    s_joint.speed_valid = speed_valid;
    s_joint.encoder_fault = Motor_Encoder_IsSignalFault();
    now_us = Hal_Motor_TimeUs();
    s_joint.command_age_us = now_us - s_joint_last_command_us;
    s_joint.command_timeout = s_joint.timeout_enabled && s_joint.command_age_us > s_joint_timeout_us;
    if (!s_joint.enabled || !motor.ol_enable || Motor_Encoder_IsSignalFault()) {
        Motor_Joint_ClearHold();
        if (s_joint.enabled) {
            motor.ol_target_rpm = 0.0f;
        }
        return;
    }
    if (s_joint.command_timeout) {
        Motor_JointControl_Reset(&s_control);
        s_joint.bias_v = 0.0f;
        if (s_joint.backend == MOTOR_JOINT_BACKEND_TORQUE_CURRENT) {
            float torque = speed_valid ? -s_joint_torque_kd_nm_per_rad_s * measured_rpm * MOTOR_JOINT_RPM_TO_RAD_S : 0.0f;
            float iq = Motor_Joint_LimitOutput(torque * s_joint_torque_to_iq, s_joint_torque_current_limit_a);

            s_joint.torque_command_nm = torque;
            s_joint.iq_command_a = iq;
            s_joint.output_v = 0.0f;
            motor.joint_voltage_hold = false;
            motor.joint_hold_v = 0.0f;
            motor.joint_current_hold = true;
            motor.joint_hold_iq = iq;
        } else if (s_joint.backend == MOTOR_JOINT_BACKEND_CURRENT_DIRECT) {
            s_joint.torque_command_nm = 0.0f;
            s_joint.iq_command_a = 0.0f;
            s_joint.output_v = 0.0f;
            motor.joint_voltage_hold = false;
            motor.joint_hold_v = 0.0f;
            motor.joint_current_hold = true;
            motor.joint_hold_iq = 0.0f;
        } else {
            float vq = speed_valid ? -s_joint.kd_v_per_rpm * measured_rpm : 0.0f;

            vq = Motor_Joint_LimitOutput(vq, s_joint.v_limit_v);
            s_joint.hybrid_current_active = false;
            s_joint.output_v = vq;
            s_joint.iq_command_a = vq / MOTOR_ENCODER_SPEED_VOLT_PER_AMP;
            motor.joint_voltage_hold = true;
            motor.joint_hold_v = vq;
            motor.joint_current_hold = false;
            motor.joint_hold_iq = 0.0f;
        }
        motor.position_voltage_hold = false;
        motor.position_hold_v = 0.0f;
        motor.ol_target_rpm = 0.0f;
        return;
    }
    if ((MOTOR_JOINT_TRAJECTORY_ENABLE != 0U) &&
        s_joint.backend == MOTOR_JOINT_BACKEND_POSITION_CURRENT && s_joint_trajectory_enabled) {
        float trajectory_vel = MOTOR_JOINT_TRAJECTORY_VEL_DEG_S;
        float trajectory_accel = MOTOR_JOINT_TRAJECTORY_ACCEL_DEG_S2;
        float trajectory_decel = MOTOR_JOINT_TRAJECTORY_DECEL_DEG_S2;
        motor_trajectory_config_t trajectory_config;

        /* 测试转速按与默认最高速度的比例缩放加减速，加速时间仍约 0.5 s。 */
        if (s_joint_trajectory_rpm > 0.0f && trajectory_vel > 0.0f) {
            float scale = s_joint_trajectory_rpm * 6.0f / trajectory_vel;

            trajectory_vel *= scale;
            trajectory_accel *= scale;
            trajectory_decel *= scale;
        }
        trajectory_config.velocity_limit = trajectory_vel;
        trajectory_config.acceleration_limit = trajectory_accel;
        trajectory_config.deceleration_limit = trajectory_decel;
        trajectory_config.position_tolerance = 0.02f;
        trajectory_config.velocity_tolerance = 0.05f;
        trajectory_config.maximum_dt_s = MOTOR_JOINT_DT_MAX_S;

        if (!Motor_Trajectory_Step(&s_joint_trajectory, &trajectory_config, dt)) {
            Motor_Joint_ClearHold();
            return;
        }
        s_joint.planned_position_deg = s_joint_trajectory.position;
        s_joint.planned_rpm = s_joint_trajectory.velocity / 6.0f;
        s_joint.trajectory_active = s_joint_trajectory.active;
        if (!Motor_Joint_ApplyControlSetpoint(s_joint.planned_position_deg, s_joint.planned_rpm)) {
            Motor_Joint_ClearHold();
            return;
        }
    }
    feedback.position_rad = s_joint.position_deg * MOTOR_JOINT_DEG_TO_RAD;
    feedback.velocity_rad_s = measured_rpm * MOTOR_JOINT_RPM_TO_RAD_S;
    feedback.velocity_valid = speed_valid;
    outer_output = Motor_JointControl_Step(&s_control, &feedback, dt, true);
    s_joint.bias_v = s_control.integral_output;
    if (s_joint.backend == MOTOR_JOINT_BACKEND_TORQUE_CURRENT) {
        float iq = Motor_Joint_LimitOutput(outer_output * s_joint_torque_to_iq, s_joint_torque_current_limit_a);

        s_joint.torque_command_nm = outer_output;
        s_joint.iq_command_a = iq;
        s_joint.output_v = 0.0f;
        motor.joint_voltage_hold = false;
        motor.joint_hold_v = 0.0f;
        motor.joint_current_hold = true;
        motor.joint_hold_iq = iq;
    } else if (s_joint.backend == MOTOR_JOINT_BACKEND_CURRENT_DIRECT) {
        float iq = Motor_Joint_LimitOutput(outer_output, s_joint_torque_current_limit_a);

        s_joint.torque_command_nm = 0.0f;
        s_joint.iq_command_a = iq;
        s_joint.output_v = 0.0f;
        motor.joint_voltage_hold = false;
        motor.joint_hold_v = 0.0f;
        motor.joint_current_hold = true;
        motor.joint_hold_iq = iq;
    } else if (s_joint.backend == MOTOR_JOINT_BACKEND_POSITION_CURRENT) {
        float iq_current_limit = s_joint_torque_current_limit_a;
        float iq = outer_output;
        float vq;
        float motion_rpm = s_joint.trajectory_active ? s_joint.planned_rpm : s_joint.target_rpm;

        if (motor.speed_iq_limit > 0.0f && motor.speed_iq_limit < iq_current_limit) {
            iq_current_limit = motor.speed_iq_limit;
        }
        /*
         * 极低速位置斜坡中，线性PD输出常小于静摩擦电流，表现为误差积累后突然滑动并往返摆动。
         * 仅在明确的非零速度命令下加入方向摩擦前馈；到点 velocity=0 时立即撤掉，不影响静止保持。
         */
        if (motion_rpm > MOTOR_JOINT_MOTION_FF_MIN_RPM) {
            iq += MOTOR_JOINT_MOTION_FF_A;
        } else if (motion_rpm < -MOTOR_JOINT_MOTION_FF_MIN_RPM) {
            iq -= MOTOR_JOINT_MOTION_FF_A;
        }
        iq = Motor_Joint_LimitOutput(iq, s_joint_torque_current_limit_a);
        if (s_joint.hybrid_current_active) {
            if (fabsf(iq) <= MOTOR_JOINT_HYBRID_IQ_EXIT_A) {
                s_joint.hybrid_current_active = false;
                if (s_joint.hybrid_exit_count < UINT32_MAX) {
                    s_joint.hybrid_exit_count++;
                }
            }
        } else if (iq_current_limit >= MOTOR_JOINT_HYBRID_IQ_ENTER_A &&
                   fabsf(iq) >= MOTOR_JOINT_HYBRID_IQ_ENTER_A) {
            s_joint.hybrid_current_active = true;
            if (s_joint.hybrid_enter_count < UINT32_MAX) {
                s_joint.hybrid_enter_count++;
            }
        }
        s_joint.torque_command_nm = 0.0f;
        if (s_joint.hybrid_current_active) {
            iq = Motor_Joint_LimitOutput(iq, iq_current_limit);
            if (fabsf(iq) > s_joint.hybrid_peak_iq_a) {
                s_joint.hybrid_peak_iq_a = fabsf(iq);
            }
            s_joint.output_v = 0.0f;
            motor.joint_voltage_hold = false;
            motor.joint_hold_v = 0.0f;
            motor.joint_current_hold = true;
            motor.joint_hold_iq = iq;
        } else {
            vq = Motor_Joint_LimitOutput(iq * MOTOR_ENCODER_SPEED_VOLT_PER_AMP, s_joint.v_limit_v);
            s_joint.output_v = vq;
            motor.joint_voltage_hold = true;
            motor.joint_hold_v = vq;
            motor.joint_current_hold = false;
            motor.joint_hold_iq = 0.0f;
        }
        s_joint.iq_command_a = iq;
    }
    motor.position_voltage_hold = false;
    motor.position_hold_v = 0.0f;
    motor.ol_target_rpm = 0.0f;
}

/**
 * @description: 复制关节阻抗状态。
 * @param {motor_joint_state_t *} state  输出，为空则忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_GetState(motor_joint_state_t *state)
{
    uint32_t primask;
    uint32_t last_command_us;

    COMMON_ASSERT(state != NULL);
    primask = Motor_Crit_Enter();
    *state = s_joint;
    last_command_us = s_joint_last_command_us;
    Motor_Crit_Exit(primask);
    state->position_deg = Motor_Joint_ReadDeg();
    state->error_deg = state->target_deg - state->position_deg;
    state->command_age_us = Hal_Motor_TimeUs() - last_command_us;
}

/**
 * @description: 查询关节阻抗是否使能。
 * @return {bool} true=已使能
 * @author: lkc
 */
bool Motor_Joint_IsEnabled(void)
{
    return s_joint.enabled;
}
#endif /* MOTOR_CFG_JOINT */

#ifdef __cplusplus
}
#endif
