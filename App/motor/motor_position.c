/*
 * @Description: 编码器多圈位置环（位置 P 串速度 PI，输出电压）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "motor_position.h"
#include "motor_position_control.h"
#include "motor_encoder.h"
#include "motor_mt6701.h"
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_POSITION_TOLERANCE_DEG           (0.30f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_position_state_t s_position;
static motor_position_control_t s_control;
/* Private function prototypes -----------------------------------------------*/
static float Motor_Position_ReadDeg(void);
static void Motor_Position_ClearHold(void);
static void Motor_Position_FillControlConfig(motor_position_control_config_t *config);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 从应用位置状态生成纯控制模块配置。
 * @param {motor_position_control_config_t *} config 输出配置，不可为空
 * @return {void} 无
 */
static void Motor_Position_FillControlConfig(motor_position_control_config_t *config)
{
    config->position_kp_rpm_per_deg = s_position.kp_rpm_per_deg;
    config->maximum_rpm = s_position.max_rpm;
    config->velocity_kp_v_per_rpm = MOTOR_POSITION_VEL_KP_DEFAULT;
    config->velocity_ki_v_per_rpm_s = MOTOR_POSITION_VEL_KI_DEFAULT;
    config->voltage_limit_v = MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT;
    config->acceleration_limit_rpm_s = MOTOR_POSITION_ACCEL_RPM_S_DEFAULT;
    config->feedforward_v_per_rpm = MOTOR_POSITION_FF_V_PER_RPM_DEFAULT;
    config->maximum_dt_s = 0.05f;
}

/**
 * @description: 用累计机械计数换算相对零点的角度。
 * @return {float} 当前位置 (deg)
 * @author: lkc
 */
static float Motor_Position_ReadDeg(void)
{
    int32_t delta = (int32_t)(Motor_Encoder_GetAccumulatedCount() - (uint32_t)s_position.origin_count);
    return (float)delta * 360.0f / (float)MOTOR_MT6701_COUNTS_PER_REV;
}

/**
 * @description: 退出位置保持，清掉速度积分和电压输出。
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Position_ClearHold(void)
{
    Motor_PositionControl_Reset(&s_control);
    s_position.voltage_hold_mode = true;
    motor.position_hold = false;
    motor.position_voltage_hold = false;
    motor.position_hold_v = 0.0f;
}

/**
 * @description: 用当前累计计数作为零点，并装入默认比例与转速上限。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_Init(void)
{
    memset(&s_position, 0, sizeof(s_position));
    s_position.origin_count = (int32_t)Motor_Encoder_GetAccumulatedCount();
    s_position.kp_rpm_per_deg = MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG;
    s_position.max_rpm = MOTOR_POSITION_MAX_RPM_DEFAULT;
    s_position.tolerance_deg = MOTOR_POSITION_TOLERANCE_DEG;
    s_position.voltage_hold_mode = true;
    {
        motor_position_control_config_t config;
        Motor_Position_FillControlConfig(&config);
        (void)Motor_PositionControl_Init(&s_control, &config);
    }
}

/**
 * @description: 把当前位置记为零，并清掉目标与速度指令。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_Zero(void)
{
    s_position.origin_count = (int32_t)Motor_Encoder_GetAccumulatedCount();
    s_position.position_deg = 0.0f;
    s_position.target_deg = 0.0f;
    s_position.voltage_hold_mode = true;
    Motor_PositionControl_Reset(&s_control);
}

/**
 * @description: 开关位置环。
 * @detail: 打开前要求编码器零点有效且无信号故障。关闭时速度目标清零。
 * @param {bool} enable  true=使能
 * @return {bool} true=已设置；false=编码器未就绪
 * @author: lkc
 */
bool Motor_Position_Enable(bool enable)
{
    if (enable && (!Motor_Encoder_IsZeroValid() || Motor_Encoder_IsSignalFault())) {
        return false;
    }
    s_position.enabled = enable;
    if (!enable) {
        Motor_Position_ClearHold();
    }
    return true;
}

/**
 * @description: 设置绝对目标位置。
 * @param {float} target_deg  相对零点的目标 (deg)，须为有限值
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Position_SetTargetDeg(float target_deg)
{
    if (!isfinite(target_deg)) return false;
    if (target_deg != s_position.target_deg) {
        /* 新目标不继承旧目标为克服齿槽积出的电压，尤其避免大角度反向时先向旧方向冲击。 */
        Motor_PositionControl_Reset(&s_control);
    }
    s_position.target_deg = target_deg;
    return true;
}

/**
 * @description: 在当前目标上叠加一段位移。
 * @param {float} delta_deg  增量 (deg)，须为有限值
 * @return {bool} true=相加后目标仍为有限值
 * @author: lkc
 */
bool Motor_Position_MoveDeg(float delta_deg)
{
    float target_deg;

    if (!isfinite(delta_deg)) return false;
    target_deg = s_position.target_deg + delta_deg;
    if (!isfinite(target_deg)) return false;
    s_position.target_deg = target_deg;
    Motor_PositionControl_Reset(&s_control);
    return true;
}

/**
 * @description: 设置位置比例。
 * @param {float} kp_rpm_per_deg  比例 (RPM/deg)，范围 (0, 100]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Position_SetKp(float kp_rpm_per_deg)
{
    if (!isfinite(kp_rpm_per_deg) || kp_rpm_per_deg <= 0.0f || kp_rpm_per_deg > 100.0f) return false;
    s_position.kp_rpm_per_deg = kp_rpm_per_deg;
    return true;
}

/**
 * @description: 设置位置环给出的速度上限。
 * @param {float} max_rpm  限幅 (RPM)，范围 [1, 当前电机允许上限]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Position_SetMaxRpm(float max_rpm)
{
    if (!isfinite(max_rpm) || max_rpm < 1.0f || max_rpm > MOTOR_POSITION_MAX_RPM_ALLOWED) return false;
    s_position.max_rpm = max_rpm;
    return true;
}

/**
 * @description: 推进一拍位置环；大误差可用速度电流环移动，目标附近直接电压保持。
 * @detail: 未使能、电机未启动或编码器故障时退出位置保持并清积分。
 *          位置误差始终换成转速命令，再由速度 PI 出电压。转速写进 ol_target_rpm 的值保持 0，
 *          避免电压速度模式的起动前馈在终点被踢出来。到达目标后积分保留，用来抵消齿槽。
 * @param {float} dt  调用周期 (s)。非正或大于 0.05 s 时本拍不积分
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_Run(float dt)
{
    motor_position_control_config_t config;
    motor_position_control_input_t input;
    motor_position_control_output_t output;

    s_position.position_deg = Motor_Position_ReadDeg();
    /* 未使能时位置模块不拥有速度命令，不能把 olspeed 写入的目标反复清成 0。 */
    if (!s_position.enabled) {
        Motor_Position_ClearHold();
        return;
    }
    Motor_Position_FillControlConfig(&config);
    memset(&input, 0, sizeof(input));
    input.enabled = s_position.enabled;
    input.motor_running = motor.ol_enable;
    input.feedback_valid = !Motor_Encoder_IsSignalFault();
    input.speed_valid = Motor_Encoder_IsSpeedValid();
    input.target_deg = s_position.target_deg;
    input.position_deg = s_position.position_deg;
    input.measured_rpm = input.speed_valid ? Motor_Encoder_GetMechRpm() : 0.0f;
    input.dt = dt;
    if (!Motor_PositionControl_Step(&s_control, &config, &input, &output)) {
        Motor_Position_ClearHold();
        return;
    }
    s_position.error_deg = output.error_deg;
    s_position.target_rpm = output.target_rpm;
    motor.position_hold = output.hold_active;
    if ((MOTOR_POSITION_HYBRID_SPEED_ENABLE != 0U) && output.hold_active) {
        float error_abs = fabsf(output.error_deg);

        if (s_position.voltage_hold_mode) {
            if (error_abs > MOTOR_POSITION_HOLD_EXIT_DEG) s_position.voltage_hold_mode = false;
        } else if (error_abs <= MOTOR_POSITION_HOLD_ENTER_DEG) {
            s_position.voltage_hold_mode = true;
        }
    } else {
        s_position.voltage_hold_mode = true;
    }
    motor.position_voltage_hold = output.hold_active && s_position.voltage_hold_mode;
    if (motor.position_voltage_hold) {
        motor.position_hold_v = output.output_v;
        motor.ol_target_rpm = 0.0f;
    } else if (output.hold_active) {
        /* 移动阶段复用编码器速度电流环；直接电压 PI 不在后台积累。 */
        Motor_PositionControl_Reset(&s_control);
        motor.position_hold_v = 0.0f;
        motor.ol_target_rpm = output.target_rpm;
    } else {
        motor.position_hold_v = 0.0f;
        motor.ol_target_rpm = 0.0f;
    }
}

/**
 * @description: 复制位置环状态。
 * @param {motor_position_state_t} *state  输出，为空则忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_GetState(motor_position_state_t *state)
{
    if (state == NULL) return;

    *state = s_position;
    /*
     * Shell 可能刚改完目标、控制任务还没运行下一拍。只刷新返回快照中的观测量，
     * 不推进 PI、不改积分，避免立即打印上一拍的位置误差和速度命令。
     */
    state->position_deg = Motor_Position_ReadDeg();
    state->error_deg = state->target_deg - state->position_deg;
    state->target_rpm = Motor_Encoder_IsSignalFault()
                            ? 0.0f
                            : fmaxf(-state->max_rpm,
                                    fminf(state->kp_rpm_per_deg * state->error_deg, state->max_rpm));
}

/**
 * @description: 查询位置环是否使能。
 * @return {bool} true=已使能
 * @author: lkc
 */
bool Motor_Position_IsEnabled(void)
{
    return s_position.enabled;
}
#ifdef __cplusplus
}
#endif
