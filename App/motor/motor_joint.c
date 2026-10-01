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
#include "motor_encoder.h"
#include "motor_mt6701.h"
#include "motor_param.h"
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
#define MOTOR_JOINT_TARGET_CHANGE_DEG     (0.01f)
#define MOTOR_JOINT_INTEGRAL_MAX_ERROR_DEG (2.0f)
#define MOTOR_JOINT_INTEGRAL_MAX_RPM      (1.0f)
#define MOTOR_JOINT_INTEGRAL_MAX_TARGET_RPM (0.01f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_joint_state_t s_joint;
static int32_t s_joint_origin;
/* Private function prototypes -----------------------------------------------*/
static float Motor_Joint_Clamp(float value, float limit);
static float Motor_Joint_ReadDeg(void);
static void Motor_Joint_ClearHold(void);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把数值限制在对称区间内。
 * @param {float} value  输入
 * @param {float} limit  绝对值上限，须为正
 * @return {float} 限幅后的值
 * @author: lkc
 */
static float Motor_Joint_Clamp(float value, float limit)
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
    s_joint.output_v = 0.0f;
    s_joint.bias_v = 0.0f;
    motor.joint_voltage_hold = false;
    motor.joint_hold_v = 0.0f;
}

/**
 * @description: 用当前累计计数作为零点，并装入默认刚度、阻尼和电压上限。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Init(void)
{
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
    Motor_Joint_ClearHold();
}

/**
 * @description: 把当前位置记为零，并清掉目标与前馈。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Zero(void)
{
    s_joint_origin = (int32_t)Motor_Encoder_GetAccumulatedCount();
    s_joint.target_deg = 0.0f;
    s_joint.position_deg = 0.0f;
    s_joint.error_deg = 0.0f;
    s_joint.target_rpm = 0.0f;
    s_joint.torque_ff_a = 0.0f;
    s_joint.output_v = 0.0f;
    s_joint.bias_v = 0.0f;
}

/**
 * @description: 开关关节阻抗。
 * @param {bool} enable  true=使能
 * @return {bool} true=已设置；false=编码器未就绪
 * @author: lkc
 */
bool Motor_Joint_Enable(bool enable)
{
    if (enable && (!Motor_Encoder_IsZeroValid() || Motor_Encoder_IsSignalFault())) {
        return false;
    }
    s_joint.enabled = enable;
    if (!enable) {
        Motor_Joint_ClearHold();
    }
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
    if (!isfinite(target_deg) || !isfinite(target_rpm) || !isfinite(torque_ff_a)) {
        return false;
    }
    if (fabsf(torque_ff_a) > MOTOR_JOINT_TORQUE_FF_MAX_A) {
        return false;
    }
    /* 新位置目标不能继承上一落点的静摩擦补偿，否则反向阶跃时旧积分会先推错方向。 */
    if (fabsf(target_deg - s_joint.target_deg) >= MOTOR_JOINT_TARGET_CHANGE_DEG) {
        s_joint.bias_v = 0.0f;
    }
    s_joint.target_deg = target_deg;
    s_joint.target_rpm = target_rpm;
    s_joint.torque_ff_a = torque_ff_a;
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
    if (!isfinite(kp_v_per_deg) || kp_v_per_deg <= 0.0f || kp_v_per_deg > MOTOR_JOINT_KP_MAX_V_PER_DEG) {
        return false;
    }
    s_joint.kp_v_per_deg = kp_v_per_deg;
    return true;
}

/**
 * @description: 设置阻尼。
 * @param {float} kd_v_per_rpm  阻尼 (V/RPM)，范围 [0, 0.2]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetKd(float kd_v_per_rpm)
{
    if (!isfinite(kd_v_per_rpm) || kd_v_per_rpm < 0.0f || kd_v_per_rpm > MOTOR_JOINT_KD_MAX_V_PER_RPM) {
        return false;
    }
    s_joint.kd_v_per_rpm = kd_v_per_rpm;
    return true;
}

/**
 * @description: 设置停住补偿的慢积分。
 * @param {float} ki_v_per_deg_s  积分增益 (V/deg/s)，范围 [0, 1]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetKi(float ki_v_per_deg_s)
{
    if (!isfinite(ki_v_per_deg_s) || ki_v_per_deg_s < 0.0f || ki_v_per_deg_s > MOTOR_JOINT_KI_MAX_V_PER_DEG_S) {
        return false;
    }
    s_joint.ki_v_per_deg_s = ki_v_per_deg_s;
    if (ki_v_per_deg_s == 0.0f) {
        s_joint.bias_v = 0.0f;
    }
    return true;
}

/**
 * @description: 设置合成电压绝对值上限。
 * @param {float} limit_v  上限 (V)，范围 [0.2, 6]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetVoltageLimit(float limit_v)
{
    if (!isfinite(limit_v) || limit_v < MOTOR_JOINT_V_LIMIT_MIN_V || limit_v > MOTOR_JOINT_V_LIMIT_MAX_V) {
        return false;
    }
    s_joint.v_limit_v = limit_v;
    return true;
}

/**
 * @description: 推进一拍阻抗公式，电压写入 joint_hold_v。
 * @param {float} dt  调用周期 (s)。慢积分用它累加，大于 0.05 s 时本拍不积分
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Run(float dt)
{
    float measured_rpm = 0.0f;
    float vq;
    float torque_v;
    bool winding_up;
    bool integral_allowed;

    s_joint.position_deg = Motor_Joint_ReadDeg();
    s_joint.error_deg = s_joint.target_deg - s_joint.position_deg;
    if (Motor_Encoder_IsSpeedValid()) {
        measured_rpm = Motor_Encoder_GetMechRpm();
    }
    s_joint.measured_rpm = measured_rpm;
    if (!s_joint.enabled || !motor.ol_enable || Motor_Encoder_IsSignalFault()) {
        Motor_Joint_ClearHold();
        if (s_joint.enabled) {
            motor.ol_target_rpm = 0.0f;
        }
        return;
    }
    if (!(dt > 0.0f) || dt > MOTOR_JOINT_DT_MAX_S) {
        dt = 0.0f;
    }

    torque_v = s_joint.torque_ff_a * MOTOR_ENCODER_SPEED_VOLT_PER_AMP;
    vq = s_joint.kp_v_per_deg * s_joint.error_deg;
    vq += s_joint.kd_v_per_rpm * (s_joint.target_rpm - measured_rpm);
    vq += torque_v;
    winding_up = (vq + s_joint.bias_v >= s_joint.v_limit_v && s_joint.error_deg > 0.0f) ||
                 (vq + s_joint.bias_v <= -s_joint.v_limit_v && s_joint.error_deg < 0.0f);
    integral_allowed = fabsf(s_joint.target_rpm) <= MOTOR_JOINT_INTEGRAL_MAX_TARGET_RPM &&
                       fabsf(measured_rpm) <= MOTOR_JOINT_INTEGRAL_MAX_RPM &&
                       fabsf(s_joint.error_deg) <= MOTOR_JOINT_INTEGRAL_MAX_ERROR_DEG;
    if (integral_allowed && !winding_up && dt > 0.0f) {
        s_joint.bias_v += s_joint.ki_v_per_deg_s * s_joint.error_deg * dt;
        s_joint.bias_v = Motor_Joint_Clamp(s_joint.bias_v, MOTOR_JOINT_BIAS_MAX_V);
    }
    vq += s_joint.bias_v;
    vq = Motor_Joint_Clamp(vq, s_joint.v_limit_v);
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
    if (state == NULL) {
        return;
    }
    *state = s_joint;
    state->position_deg = Motor_Joint_ReadDeg();
    state->error_deg = state->target_deg - state->position_deg;
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
