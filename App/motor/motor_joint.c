/*
 * @Description: 关节阻抗模式（位置、速度、力矩前馈合成电压）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stddef.h>
#include "motor_joint.h"
#include "motor_joint_control.h"
#include "motor_encoder.h"
#include "motor_mt6701.h"
#include "motor_param.h"
#include "motor_crit.h"
#include "hal_motor.h"
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
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_joint_state_t s_joint;
static int32_t s_joint_origin;
static motor_joint_control_t s_control;
static uint32_t s_joint_last_command_us;
static uint32_t s_joint_timeout_us;
/* Private function prototypes -----------------------------------------------*/
static float Motor_Joint_ReadDeg(void);
static void Motor_Joint_ClearHold(void);
static bool Motor_Joint_ApplyControlConfig(void);
static bool Motor_Joint_ApplyControlCommand(void);
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
    s_joint.output_v = 0.0f;
    s_joint.bias_v = 0.0f;
    motor.joint_voltage_hold = false;
    motor.joint_hold_v = 0.0f;
}

/**
 * @description: 把兼容Shell使用的deg/RPM/V参数换成纯算法的rad、rad/s和输出单位。
 * @return {bool} true=配置有效并已应用
 */
static bool Motor_Joint_ApplyControlConfig(void)
{
    motor_joint_control_config_t config;

    config.kp_output_per_rad = s_joint.kp_v_per_deg * MOTOR_JOINT_RAD_TO_DEG;
    config.kd_output_per_rad_s = s_joint.kd_v_per_rpm * MOTOR_JOINT_RAD_S_TO_RPM;
    config.ki_output_per_rad_s = s_joint.ki_v_per_deg_s * MOTOR_JOINT_RAD_TO_DEG;
    config.output_limit = s_joint.v_limit_v;
    config.integral_limit = MOTOR_JOINT_BIAS_MAX_V;
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
    motor_joint_control_command_t command;

    command.position_rad = s_joint.target_deg * MOTOR_JOINT_DEG_TO_RAD;
    command.velocity_rad_s = s_joint.target_rpm * MOTOR_JOINT_RPM_TO_RAD_S;
    command.feedforward_output = s_joint.torque_ff_a * MOTOR_ENCODER_SPEED_VOLT_PER_AMP;
    return Motor_JointControl_SetCommand(&s_control, &command);
}

static float Motor_Joint_LimitOutput(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
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
    s_joint.error_deg = 0.0f;
    s_joint.target_rpm = 0.0f;
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
    s_joint_timeout_us = MOTOR_JOINT_TIMEOUT_DEFAULT_US;
    s_joint_last_command_us = Hal_Motor_TimeUs();
    config.kp_output_per_rad = s_joint.kp_v_per_deg * MOTOR_JOINT_RAD_TO_DEG;
    config.kd_output_per_rad_s = s_joint.kd_v_per_rpm * MOTOR_JOINT_RAD_S_TO_RPM;
    config.ki_output_per_rad_s = s_joint.ki_v_per_deg_s * MOTOR_JOINT_RAD_TO_DEG;
    config.output_limit = s_joint.v_limit_v;
    config.integral_limit = MOTOR_JOINT_BIAS_MAX_V;
    config.integral_max_error_rad = MOTOR_JOINT_INTEGRAL_MAX_ERROR_DEG * MOTOR_JOINT_DEG_TO_RAD;
    config.integral_max_velocity_rad_s = MOTOR_JOINT_INTEGRAL_MAX_RPM * MOTOR_JOINT_RPM_TO_RAD_S;
    config.integral_max_target_rad_s = MOTOR_JOINT_INTEGRAL_MAX_TARGET_RPM * MOTOR_JOINT_RPM_TO_RAD_S;
    config.target_reset_threshold_rad = MOTOR_JOINT_TARGET_CHANGE_DEG * MOTOR_JOINT_DEG_TO_RAD;
    config.dt_max_s = MOTOR_JOINT_DT_MAX_S;
    (void)Motor_JointControl_Init(&s_control, &config);
    (void)Motor_Joint_ApplyControlCommand();
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
    s_joint.error_deg = 0.0f;
    s_joint.target_rpm = 0.0f;
    s_joint.torque_ff_a = 0.0f;
    s_joint.output_v = 0.0f;
    s_joint.bias_v = 0.0f;
    (void)Motor_Joint_ApplyControlCommand();
    Motor_JointControl_Reset(&s_control);
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

    if (enable && (!Motor_Encoder_IsZeroValid() || Motor_Encoder_IsSignalFault())) {
        return false;
    }
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

    if (!isfinite(target_deg) || !isfinite(target_rpm) || !isfinite(torque_ff_a)) {
        return false;
    }
    if (fabsf(torque_ff_a) > MOTOR_JOINT_TORQUE_FF_MAX_A) {
        return false;
    }
    primask = Motor_Crit_Enter();
    old_target_deg = s_joint.target_deg;
    old_target_rpm = s_joint.target_rpm;
    old_torque_ff_a = s_joint.torque_ff_a;
    s_joint.target_deg = target_deg;
    s_joint.target_rpm = target_rpm;
    s_joint.torque_ff_a = torque_ff_a;
    if (!Motor_Joint_ApplyControlCommand()) {
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
    if (command->enable && (!Motor_Encoder_IsZeroValid() || Motor_Encoder_IsSignalFault())) {
        return false;
    }

    now_us = Hal_Motor_TimeUs();
    primask = Motor_Crit_Enter();
    old_state = s_joint;
    old_timeout_us = s_joint_timeout_us;
    s_joint.enabled = command->enable;
    s_joint.target_deg = command->position_rad * MOTOR_JOINT_RAD_TO_DEG;
    s_joint.target_rpm = command->velocity_rad_s * MOTOR_JOINT_RAD_S_TO_RPM;
    s_joint.torque_ff_a = (MOTOR_ENCODER_SPEED_VOLT_PER_AMP > 0.0f)
                             ? command->feedforward_v / MOTOR_ENCODER_SPEED_VOLT_PER_AMP
                             : 0.0f;
    s_joint.kp_v_per_deg = command->kp_v_per_rad * MOTOR_JOINT_DEG_TO_RAD;
    s_joint.kd_v_per_rpm = command->kd_v_per_rad_s * MOTOR_JOINT_RPM_TO_RAD_S;
    s_joint.ki_v_per_deg_s = command->ki_v_per_rad_s * MOTOR_JOINT_DEG_TO_RAD;
    s_joint.v_limit_v = command->output_limit_v;
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
 * @description: 设置刚度。
 * @param {float} kp_v_per_deg  刚度 (V/deg)，范围 (0, 2]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetKp(float kp_v_per_deg)
{
    uint32_t primask;
    bool result;

    if (!isfinite(kp_v_per_deg) || kp_v_per_deg <= 0.0f || kp_v_per_deg > MOTOR_JOINT_KP_MAX_V_PER_DEG) {
        return false;
    }
    primask = Motor_Crit_Enter();
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

    if (!isfinite(kd_v_per_rpm) || kd_v_per_rpm < 0.0f || kd_v_per_rpm > MOTOR_JOINT_KD_MAX_V_PER_RPM) {
        return false;
    }
    primask = Motor_Crit_Enter();
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

    if (!isfinite(ki_v_per_deg_s) || ki_v_per_deg_s < 0.0f || ki_v_per_deg_s > MOTOR_JOINT_KI_MAX_V_PER_DEG_S) {
        return false;
    }
    primask = Motor_Crit_Enter();
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

    if (!isfinite(limit_v) || limit_v < MOTOR_JOINT_V_LIMIT_MIN_V || limit_v > MOTOR_JOINT_V_LIMIT_MAX_V) {
        return false;
    }
    primask = Motor_Crit_Enter();
    s_joint.v_limit_v = limit_v;
    result = Motor_Joint_ApplyControlConfig();
    Motor_Crit_Exit(primask);
    return result;
}

/**
 * @description: 推进一拍阻抗公式，电压写入 joint_hold_v。
 * @param {float} dt  调用周期 (s)。慢积分用它累加，大于 0.05 s 时本拍不积分
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Run(float dt)
{
    motor_joint_control_feedback_t feedback;
    float measured_rpm = 0.0f;
    float vq;
    bool speed_valid;
    uint32_t now_us;

    s_joint.position_deg = Motor_Joint_ReadDeg();
    s_joint.error_deg = s_joint.target_deg - s_joint.position_deg;
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
        vq = speed_valid ? -s_joint.kd_v_per_rpm * measured_rpm : 0.0f;
        vq = Motor_Joint_LimitOutput(vq, s_joint.v_limit_v);
        s_joint.output_v = vq;
        motor.joint_voltage_hold = true;
        motor.joint_hold_v = vq;
        motor.position_voltage_hold = false;
        motor.position_hold_v = 0.0f;
        motor.ol_target_rpm = 0.0f;
        return;
    }
    feedback.position_rad = s_joint.position_deg * MOTOR_JOINT_DEG_TO_RAD;
    feedback.velocity_rad_s = measured_rpm * MOTOR_JOINT_RPM_TO_RAD_S;
    feedback.velocity_valid = speed_valid;
    vq = Motor_JointControl_Step(&s_control, &feedback, dt, true);
    s_joint.bias_v = s_control.integral_output;
    s_joint.output_v = vq;
    motor.joint_voltage_hold = true;
    motor.joint_hold_v = vq;
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

    if (state == NULL) {
        return;
    }
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

#ifdef __cplusplus
}
#endif
