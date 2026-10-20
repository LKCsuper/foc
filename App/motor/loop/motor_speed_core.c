/* 速度控制核心：标准C，无硬件或全局电机依赖。 */
#include <math.h>
#include "common_assert.h"
#include <string.h>
#include "motor_speed_core.h"
#define MOTOR_SPEED_LIMIT_DECAY_S 0.100f

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
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

/**
 * @description: 清零速度 PI 的积分、误差和 Iq，保留反馈转速。
 * @param {motor_speed_state_t *} state  速度环状态，调用方保证非空
 * @return {void} 无
 * @author: lkc
 */
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
    COMMON_ASSERT(state != NULL);
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
    COMMON_ASSERT(state != NULL);
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
    COMMON_ASSERT(state != NULL);
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
    COMMON_ASSERT(state != NULL);
    state->integral = iq;
    state->iq_target = iq;
    state->loop_time = 0.0f;
}

/**
 * @description: 运行指定状态对象的速度 PI，输出 Iq 目标
 * @detail: 仅输入使能且反馈有效时分频执行；输出限幅到 input->iq_limit。
 *          输出与误差同向顶满时积分按 MOTOR_SPEED_LIMIT_DECAY_S 泄放，不再累加。
 *          负误差可以出负 Iq，用来抵消前馈。禁止制动的反向限制不在这里做。
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @param {motor_speed_input_t} *input 配置与目标快照，不可为空
 * @param {float} dt 距上一拍时间 (s)
 * @return {*}
 * @author: lkc
 */
static bool Motor_SpeedLoop_StepInternal(motor_speed_state_t *state, const motor_speed_input_t *input, float dt,
                                       bool scheduled)
{
    float loop_period;
    float control_dt;
    float iq_limit;
    float proportional;
    float integral_candidate;
    float unsaturated_output;
    float output;

    if (state == NULL || input == NULL) return false;

    if (!input->enabled || !state->feedback_valid) {
        Motor_SpeedLoop_ResetState(state);
        return false;
    }

    /* 位置环自己出电压，原速度 PI 的低速噪声和起动前馈不能再叠进去。 */
    if (input->inhibited) {
        Motor_SpeedLoop_ResetState(state);
        return false;
    }

    /*
     * 零速且禁止制动表示自然滑停：立即释放历史正向积分，避免停住后仍持续施加转矩。
     * 位置环保持时零速是要定住的目标，不能走这条松开路径。
     */
    if (!input->allow_braking && !input->position_hold && input->command_rpm == 0.0f) {
        Motor_SpeedLoop_ResetState(state);
        return false;
    }
    if (input->position_hold || input->command_rpm != 0.0f) {
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
            return false;
        }
    }

    state->loop_time += dt;
    if (!scheduled) {
        loop_period = 1.0f / (input->loop_hz < 1.0f ? 1.0f : input->loop_hz);
        if (state->loop_time < loop_period) return true;
    }

    control_dt = state->loop_time;
    state->loop_time = 0.0f;
    iq_limit = fabsf(input->iq_limit);

    state->control_target_rpm = input->target_rpm;
    state->control_measured_rpm = state->measured_rpm;
    state->error = state->control_target_rpm - state->control_measured_rpm;
    proportional = input->kp * state->error;
    integral_candidate = state->integral + input->ki * state->error * control_dt;
    unsaturated_output = proportional + integral_candidate;

    if ((unsaturated_output > iq_limit && state->error > 0.0f) ||
        (unsaturated_output < -iq_limit && state->error < 0.0f)) {
        float decay = control_dt / MOTOR_SPEED_LIMIT_DECAY_S;

        if (decay > 1.0f) {
            decay = 1.0f;
        }
        /* 同向顶满时不再累加，并按时间常数把已有积分泄掉，避免转速过了目标还满电流推。 */
        state->integral *= (1.0f - decay);
    } else {
        state->integral = Motor_Speed_Clamp(integral_candidate, -iq_limit, iq_limit);
    }
    output = Motor_Speed_Clamp(proportional + state->integral, -iq_limit, iq_limit);
    state->iq_target = output;
    state->update_count++;
    return true;
}

bool Motor_SpeedLoop_StepState(motor_speed_state_t *state, const motor_speed_input_t *input, float dt)
{
    return Motor_SpeedLoop_StepInternal(state, input, dt, false);
}

bool Motor_SpeedLoop_StepScheduledState(motor_speed_state_t *state, const motor_speed_input_t *input, float dt)
{
    return Motor_SpeedLoop_StepInternal(state, input, dt, true);
}

/**
 * @description: 禁用制动时，把最终电压或电流限制成不与转速命令反向。
 * @detail: 速度 PI 本身可以出负修正，用来抵消前馈。本函数用在前馈与 PI 相加之后。
 *          允许制动，或命令转速为 0 时，原样返回。
 * @param {float} command_rpm  转速命令 (RPM)，符号表示驱动方向
 * @param {float} output       即将执行的电压 (V) 或电流 (A)
 * @param {bool} allow_braking true=允许反向制动
 * @return {float} 限幅后的输出，单位与 output 相同
 * @author: lkc
 */
float Motor_Speed_LimitNoBrakeOutput(float command_rpm, float output, bool allow_braking)
{
    if (allow_braking) return output;
    if (command_rpm > 0.0f && output < 0.0f) return 0.0f;
    if (command_rpm < 0.0f && output > 0.0f) return 0.0f;
    return output;
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
