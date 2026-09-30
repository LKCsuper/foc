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
#include "motor_encoder.h"
#include "motor_mt6701.h"
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* ODrive 默认 pos_gain=20 (turn/s)/turn，换成 RPM/deg。 */
#define MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG (20.0f * 60.0f / 360.0f)
#define MOTOR_POSITION_MAX_RPM_DEFAULT         (60.0f)
#define MOTOR_POSITION_TOLERANCE_DEG           (0.30f)
/* 速度 PI 直接出电压。比例靠近原速度环 0.008 A/RPM × 5 V/A，并略加强阻尼。 */
#define MOTOR_POSITION_VEL_KP_V_PER_RPM   (0.050f)
#define MOTOR_POSITION_VEL_KI_V_PER_RPM_S (0.20f)
#define MOTOR_POSITION_VEL_V_LIMIT_V      (2.0f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_position_state_t s_position;
static float s_vel_integrator_v;
/* Private function prototypes -----------------------------------------------*/
static float Motor_Position_Clamp(float value, float limit);
static float Motor_Position_ReadDeg(void);
static void Motor_Position_ClearHold(void);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把数值限制在对称区间内。
 * @param {float} value  输入
 * @param {float} limit  绝对值上限，须为正
 * @return {float} 限幅后的值
 * @author: lkc
 */
static float Motor_Position_Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
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
    s_position.target_rpm = 0.0f;
    s_vel_integrator_v = 0.0f;
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
    s_vel_integrator_v = 0.0f;
    s_position.origin_count = (int32_t)Motor_Encoder_GetAccumulatedCount();
    s_position.kp_rpm_per_deg = MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG;
    s_position.max_rpm = MOTOR_POSITION_MAX_RPM_DEFAULT;
    s_position.tolerance_deg = MOTOR_POSITION_TOLERANCE_DEG;
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
    s_position.error_deg = 0.0f;
    s_position.target_rpm = 0.0f;
    s_vel_integrator_v = 0.0f;
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
    if (!isfinite(delta_deg)) return false;
    s_position.target_deg += delta_deg;
    return isfinite(s_position.target_deg) != 0;
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
 * @param {float} max_rpm  限幅 (RPM)，范围 [1, 1800]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Position_SetMaxRpm(float max_rpm)
{
    if (!isfinite(max_rpm) || max_rpm < 1.0f || max_rpm > 1800.0f) return false;
    s_position.max_rpm = max_rpm;
    return true;
}

/**
 * @description: 推进一拍位置环，速度 PI 的电压写入 position_hold_v。
 * @detail: 未使能、电机未启动或编码器故障时退出位置保持并清积分。
 *          位置误差始终换成转速命令，再由速度 PI 出电压。转速写进 ol_target_rpm 的值保持 0，
 *          避免电压速度模式的起动前馈在终点被踢出来。到达目标后积分保留，用来抵消齿槽。
 * @param {float} dt  调用周期 (s)。非正或大于 0.05 s 时本拍不积分
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_Run(float dt)
{
    float measured_rpm = 0.0f;
    float vel_des;
    float vel_err;
    float proportional;
    float integral_candidate;
    float unsaturated;
    bool winding_up;

    s_position.position_deg = Motor_Position_ReadDeg();
    s_position.error_deg = s_position.target_deg - s_position.position_deg;
    if (!s_position.enabled || !motor.ol_enable || Motor_Encoder_IsSignalFault()) {
        Motor_Position_ClearHold();
        if (s_position.enabled) {
            motor.ol_target_rpm = 0.0f;
        }
        return;
    }
    if (!(dt > 0.0f) || dt > 0.05f) {
        dt = 0.0f;
    }

    vel_des = Motor_Position_Clamp(s_position.kp_rpm_per_deg * s_position.error_deg, s_position.max_rpm);
    if (Motor_Encoder_IsSpeedValid()) {
        measured_rpm = Motor_Encoder_GetMechRpm();
    }
    vel_err = vel_des - measured_rpm;
    proportional = MOTOR_POSITION_VEL_KP_V_PER_RPM * vel_err;
    integral_candidate = s_vel_integrator_v;
    if (dt > 0.0f) {
        integral_candidate += MOTOR_POSITION_VEL_KI_V_PER_RPM_S * vel_err * dt;
    }
    integral_candidate = Motor_Position_Clamp(integral_candidate, MOTOR_POSITION_VEL_V_LIMIT_V);
    unsaturated = proportional + integral_candidate;
    winding_up = (unsaturated > MOTOR_POSITION_VEL_V_LIMIT_V && vel_err > 0.0f) ||
                 (unsaturated < -MOTOR_POSITION_VEL_V_LIMIT_V && vel_err < 0.0f);
    if (!winding_up) {
        s_vel_integrator_v = integral_candidate;
    }
    s_position.target_rpm = vel_des;
    motor.position_hold = true;
    motor.position_voltage_hold = true;
    motor.position_hold_v = Motor_Position_Clamp(proportional + s_vel_integrator_v, MOTOR_POSITION_VEL_V_LIMIT_V);
    motor.ol_target_rpm = 0.0f;
}

/**
 * @description: 复制位置环状态。
 * @param {motor_position_state_t} *state  输出，为空则忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_GetState(motor_position_state_t *state)
{
    if (state != NULL) *state = s_position;
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
