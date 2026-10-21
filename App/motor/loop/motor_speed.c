/*
 * @Description: 通用速度 PI 外环实现（支持状态实例；默认单实例包装保留）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "common_assert.h"
#include <stddef.h>

#include "motor_param.h"
#include "motor_speed.h"
#include "motor_protect.h"
#include "motor_config.h"
#if MOTOR_CFG_SPEED
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 默认单实例包装，步骤 2 过渡期保留。 */
static motor_speed_state_t s_speed;
/* ISR 在速度 PI 接管后置 true；foc pid 任务据此决定是否 Run。 */
static volatile bool s_pi_run_allowed;
bool Motor_SpeedLoop_IsRunAllowed(void) { return s_pi_run_allowed; }
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** 项目兼容入口：只映射输入，控制算法唯一实现位于core。 */
static void Motor_SpeedLoop_RunInternal(motor_speed_state_t *state, MOTOR_PARAM_S *motor, float dt, bool scheduled)
{
    motor_speed_input_t input;
    COMMON_ASSERT(state != NULL && motor != NULL);
    input.enabled = motor->speed_enable && motor->ol_mode == MOTOR_OL_MODE_IF && motor->ol_enable;
    input.inhibited = motor->position_voltage_hold || motor->joint_voltage_hold || motor->joint_current_hold;
    input.allow_braking = motor->speed_allow_braking;
    input.position_hold = motor->position_hold;
    input.command_rpm = motor->ol_target_rpm;
    input.target_rpm = motor->ol_current_rpm;
    input.loop_hz = motor->speed_loop_hz;
    input.kp = motor->speed_kp;
    input.ki = motor->speed_ki;
    input.iq_limit = motor->speed_iq_limit;
    if ((scheduled ? Motor_SpeedLoop_StepScheduledState(state, &input, dt) :
                     Motor_SpeedLoop_StepState(state, &input, dt)) && motor->speed_loop_hz < 1.0f) {
        motor->speed_loop_hz = 1.0f; /* 保留旧入口仅进入分频后归一化的行为。 */
    }
}

void Motor_SpeedLoop_RunState(motor_speed_state_t *state, MOTOR_PARAM_S *motor, float dt)
{
    Motor_SpeedLoop_RunInternal(state, motor, dt, false);
}

void Motor_SpeedLoop_RunScheduled(MOTOR_PARAM_S *motor, float dt)
{
    if (!s_pi_run_allowed) return;
    Motor_SpeedLoop_RunInternal(&s_speed, motor, dt, true);
}

/**
 * @description: 上电初始化默认单实例速度环，并向保护模块登记无效反馈
 * @return {*}
 * @author: lkc
 */
void Motor_Speed_Init(void)
{
    Motor_Speed_InitState(&s_speed);
    s_pi_run_allowed = false;
    Motor_Protect_SetSpeedFeedback(0.0f, false);
}

/**
 * @description: 更新默认实例的真实机械转速反馈
 * @detail: 同时把转速/有效性同步给保护模块用于失速判定
 * @param {float} rpm 机械转速 (RPM)，反转为负数
 * @param {bool} valid true=反馈可信；false=当前不能用于速度闭环
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedFeedback_Set(float rpm, bool valid)
{
    Motor_SpeedFeedback_SetState(&s_speed, rpm, valid);
    Motor_Protect_SetSpeedFeedback(s_speed.measured_rpm, valid);
}

/**
 * @description: 清除默认实例速度 PI 状态
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_Reset(void)
{
    s_pi_run_allowed = false;
    Motor_SpeedLoop_ResetState(&s_speed);
}

/**
 * @description: 允许或禁止默认实例速度 PI
 * @param {bool} allowed true=任务可运行 PI
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_SetRunAllowed(bool allowed)
{
    s_pi_run_allowed = allowed;
    if (!allowed) {
        Motor_SpeedLoop_ResetState(&s_speed);
    }
}

/**
 * @description: 用当前等效 Iq 预置默认实例速度 PI
 * @param {float} iq 预置 Iq 目标 (A)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_PrimeIq(float iq)
{
    Motor_SpeedLoop_PrimeIqState(&s_speed, iq);
}

/**
 * @description: 运行默认实例速度 PI 外环（委托 State 版本）
 * @param {MOTOR_PARAM_S} *motor 电机实时参数
 * @param {float} dt 控制周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_Run(MOTOR_PARAM_S *motor, float dt)
{
    if (!s_pi_run_allowed) return;
    Motor_SpeedLoop_RunState(&s_speed, motor, dt);
}

/**
 * @description: 读取默认实例当前 Iq 目标
 * @return {float} Iq 目标 (A)
 * @author: lkc
 */
float Motor_SpeedLoop_GetIqTarget(void)
{
    return Motor_SpeedLoop_GetIqTargetState(&s_speed);
}

/**
 * @description: 读取默认实例实测机械转速
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetMeasuredRpm(void)
{
    return Motor_SpeedLoop_GetMeasuredRpmState(&s_speed);
}

/**
 * @description: 读取默认实例速度误差
 * @return {float} 速度误差 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetError(void)
{
    return Motor_SpeedLoop_GetErrorState(&s_speed);
}

/**
 * @description: 查询默认实例速度反馈是否可信
 * @return {bool} true=反馈可信
 * @author: lkc
 */
bool Motor_SpeedLoop_IsFeedbackValid(void)
{
    return Motor_SpeedLoop_IsFeedbackValidState(&s_speed);
}

/**
 * @description: 读取最近一次速度 PI 使用的同一帧目标、反馈、误差和 Iq。
 * @param {motor_speed_snapshot_t *} snapshot  输出帧，不可为空
 * @return {bool} true=已写出；snapshot 为空返回 false
 * @author: lkc
 */
bool Motor_SpeedLoop_GetSnapshot(motor_speed_snapshot_t *snapshot)
{
    if (snapshot == NULL) return false;

    snapshot->target_rpm = s_speed.control_target_rpm;
    snapshot->update_count = s_speed.update_count;
    snapshot->measured_rpm = s_speed.control_measured_rpm;
    snapshot->error = s_speed.error;
    snapshot->iq_target = s_speed.iq_target;
    return true;
}

/**
 * @description: 返回默认速度环更新序号，避免诊断前复制无用浮点字段。
 * @return {uint32_t} 当前PI更新计数
 */
uint32_t Motor_SpeedLoop_GetUpdateCount(void)
{
    return s_speed.update_count;
}

#endif /* MOTOR_CFG_SPEED */
#ifdef __cplusplus
}
#endif
