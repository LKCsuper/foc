/*
 * @Description: 通用速度 PI 外环实现（支持状态实例；默认单实例包装保留）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motor_param.h"
#include "motor_speed.h"
#include "motor_protect.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 默认单实例包装，步骤 2 过渡期保留。 */
static motor_speed_state_t s_speed;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将 value 限幅到 [minimum, maximum]
 * @param {float} value 待限幅值
 * @param {float} minimum 下限
 * @param {float} maximum 上限
 * @return {float} 限幅结果
 * @author: lkc
 */
static float Motor_Speed_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static void Motor_Speed_ClearPi(motor_speed_state_t *state)
{
    state->loop_time = 0.0f;
    state->integral = 0.0f;
    state->error = 0.0f;
    state->iq_target = 0.0f;
}

/**
 * @description: 初始化指定速度环状态对象，清零反馈、PI 与分频时间
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_Speed_InitState(motor_speed_state_t *state)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
}

/**
 * @description: 更新指定状态对象的机械转速反馈
 * @detail: 反馈无效时清零转速并复位 PI，避免用不可信值闭环
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @param {float} rpm 机械转速 (RPM)
 * @param {bool} valid true=反馈可信
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedFeedback_SetState(motor_speed_state_t *state, float rpm, bool valid)
{
    if (state == NULL) {
        return;
    }
    state->measured_rpm = valid ? rpm : 0.0f;
    state->feedback_valid = valid;
    if (!valid) {
        Motor_SpeedLoop_ResetState(state);
    }
}

/**
 * @description: 清除指定状态对象的 PI（保留反馈转速）
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_ResetState(motor_speed_state_t *state)
{
    if (state == NULL) {
        return;
    }
    Motor_Speed_ClearPi(state);
    state->zero_brake_active = false;
    state->zero_brake_released = false;
    state->zero_brake_direction = 0;
}

/**
 * @description: 预置指定状态对象的 Iq 积分与输出，清分频时间
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @param {float} iq 预置 Iq (A)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_PrimeIqState(motor_speed_state_t *state, float iq)
{
    if (state == NULL) {
        return;
    }
    state->integral = iq;
    state->iq_target = iq;
    state->loop_time = 0.0f;
}

/**
 * @description: 运行指定状态对象的速度 PI，输出 Iq 目标
 * @detail: 仅 I/F+使能且反馈有效时分频执行；输出限幅到 speed_iq_limit 并抗积分饱和，
 *          禁止制动时按转速方向削负输出
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @param {MOTOR_PARAM_S} *motor 配置与目标来源，不可为空
 * @param {float} dt 距上一拍时间 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_RunState(motor_speed_state_t *state, MOTOR_PARAM_S *motor, float dt)
{
    float loop_period;
    float control_dt;
    float iq_limit;
    float proportional;
    float integral_candidate;
    float unsaturated_output;
    float output;

    if (state == NULL || motor == NULL) {
        return;
    }

    if (!motor->speed_enable || motor->ol_mode != MOTOR_OL_MODE_IF || !motor->ol_enable || !state->feedback_valid) {
        Motor_SpeedLoop_ResetState(state);
        return;
    }

    /* 零速且禁止制动表示自然滑停：立即释放历史正向积分，避免停住后仍持续施加转矩。 */
    if (!motor->speed_allow_braking && motor->ol_target_rpm == 0.0f) {
        Motor_SpeedLoop_ResetState(state);
        return;
    }
    if (motor->ol_target_rpm != 0.0f) {
        state->zero_brake_active = false;
        state->zero_brake_released = false;
        state->zero_brake_direction = 0;
    } else {
        int8_t direction = state->measured_rpm > 1.0f ? 1 : (state->measured_rpm < -1.0f ? -1 : 0);

        if (!state->zero_brake_active && !state->zero_brake_released) {
            if (direction == 0) {
                state->zero_brake_released = true;
            } else {
                state->zero_brake_active = true;
                state->zero_brake_direction = direction;
            }
        } else if (state->zero_brake_active &&
                   (direction == 0 || direction != state->zero_brake_direction)) {
            /* Hall 低速区可能超时归零，也可能越过零点后才给出反向边沿；两种情况都必须释放。 */
            state->zero_brake_active = false;
            state->zero_brake_released = true;
        }
        if (state->zero_brake_released) {
            Motor_Speed_ClearPi(state);
            return;
        }
    }

    if (motor->speed_loop_hz < 1.0f) {
        motor->speed_loop_hz = 1.0f;
    }
    loop_period = 1.0f / motor->speed_loop_hz;
    state->loop_time += dt;
    if (state->loop_time < loop_period) {
        return;
    }

    control_dt = state->loop_time;
    state->loop_time = 0.0f;
    iq_limit = fabsf(motor->speed_iq_limit);

    state->error = motor->ol_current_rpm - state->measured_rpm;
    proportional = motor->speed_kp * state->error;
    integral_candidate = state->integral + motor->speed_ki * state->error * control_dt;
    unsaturated_output = proportional + integral_candidate;
    output = Motor_Speed_Clamp(unsaturated_output, -iq_limit, iq_limit);

    if (!((unsaturated_output > iq_limit && state->error > 0.0f) ||
          (unsaturated_output < -iq_limit && state->error < 0.0f))) {
        state->integral = Motor_Speed_Clamp(integral_candidate, -iq_limit, iq_limit);
    } else {
        output = Motor_Speed_Clamp(proportional + state->integral, -iq_limit, iq_limit);
    }

    if (!motor->speed_allow_braking) {
        if (motor->ol_current_rpm > 0.0f && output < 0.0f) {
            output = 0.0f;
        } else if (motor->ol_current_rpm < 0.0f && output > 0.0f) {
            output = 0.0f;
        }
    }

    state->iq_target = output;
}

/**
 * @description: 读取指定状态当前 Iq 目标
 * @param {const motor_speed_state_t} *state 状态对象；NULL 返回 0
 * @return {float} Iq 目标 (A)
 * @author: lkc
 */
float Motor_SpeedLoop_GetIqTargetState(const motor_speed_state_t *state)
{
    return state == NULL ? 0.0f : state->iq_target;
}

/**
 * @description: 读取指定状态实测机械转速
 * @param {const motor_speed_state_t} *state 状态对象；NULL 返回 0
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetMeasuredRpmState(const motor_speed_state_t *state)
{
    return state == NULL ? 0.0f : state->measured_rpm;
}

/**
 * @description: 读取指定状态速度误差（目标−实测）
 * @param {const motor_speed_state_t} *state 状态对象；NULL 返回 0
 * @return {float} 速度误差 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetErrorState(const motor_speed_state_t *state)
{
    return state == NULL ? 0.0f : state->error;
}

/**
 * @description: 查询指定状态速度反馈是否可信
 * @param {const motor_speed_state_t} *state 状态对象；NULL 返回 false
 * @return {bool} true=反馈可信，可用于闭环
 * @author: lkc
 */
bool Motor_SpeedLoop_IsFeedbackValidState(const motor_speed_state_t *state)
{
    return state != NULL && state->feedback_valid;
}

/**
 * @description: 上电初始化默认单实例速度环，并向保护模块登记无效反馈
 * @return {*}
 * @author: lkc
 */
void Motor_Speed_Init(void)
{
    Motor_Speed_InitState(&s_speed);
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
    Motor_SpeedLoop_ResetState(&s_speed);
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

#ifdef __cplusplus
}
#endif
