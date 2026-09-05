#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motor_param.h"
#include "motor_speed.h"
#include "motor_protect.h"

/* 默认单实例包装，步骤 2 过渡期保留。 */
static motor_speed_state_t s_speed;

/**
 * @description: 将 value 限幅到 [minimum, maximum]。
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

void Motor_Speed_InitState(motor_speed_state_t *state)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
}

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

void Motor_SpeedLoop_ResetState(motor_speed_state_t *state)
{
    if (state == NULL) {
        return;
    }
    state->loop_time = 0.0f;
    state->integral = 0.0f;
    state->error = 0.0f;
    state->iq_target = 0.0f;
}

void Motor_SpeedLoop_PrimeIqState(motor_speed_state_t *state, float iq)
{
    if (state == NULL) {
        return;
    }
    state->integral = iq;
    state->iq_target = iq;
    state->loop_time = 0.0f;
}

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

    if (!motor->speed_enable || motor->ol_mode != MOTOR_OL_MODE_IF || !motor->ol_enable ||
        !state->feedback_valid) {
        Motor_SpeedLoop_ResetState(state);
        return;
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

float Motor_SpeedLoop_GetIqTargetState(const motor_speed_state_t *state)
{
    return state == NULL ? 0.0f : state->iq_target;
}

float Motor_SpeedLoop_GetMeasuredRpmState(const motor_speed_state_t *state)
{
    return state == NULL ? 0.0f : state->measured_rpm;
}

float Motor_SpeedLoop_GetErrorState(const motor_speed_state_t *state)
{
    return state == NULL ? 0.0f : state->error;
}

bool Motor_SpeedLoop_IsFeedbackValidState(const motor_speed_state_t *state)
{
    return state != NULL && state->feedback_valid;
}

void Motor_Speed_Init(void)
{
    Motor_Speed_InitState(&s_speed);
    Motor_Protect_SetSpeedFeedback(0.0f, false);
}

void Motor_SpeedFeedback_Set(float rpm, bool valid)
{
    Motor_SpeedFeedback_SetState(&s_speed, rpm, valid);
    Motor_Protect_SetSpeedFeedback(s_speed.measured_rpm, valid);
}

void Motor_SpeedLoop_Reset(void)
{
    Motor_SpeedLoop_ResetState(&s_speed);
}

void Motor_SpeedLoop_PrimeIq(float iq)
{
    Motor_SpeedLoop_PrimeIqState(&s_speed, iq);
}

void Motor_SpeedLoop_Run(MOTOR_PARAM_S *motor, float dt)
{
    Motor_SpeedLoop_RunState(&s_speed, motor, dt);
}

float Motor_SpeedLoop_GetIqTarget(void)
{
    return Motor_SpeedLoop_GetIqTargetState(&s_speed);
}

float Motor_SpeedLoop_GetMeasuredRpm(void)
{
    return Motor_SpeedLoop_GetMeasuredRpmState(&s_speed);
}

float Motor_SpeedLoop_GetError(void)
{
    return Motor_SpeedLoop_GetErrorState(&s_speed);
}

bool Motor_SpeedLoop_IsFeedbackValid(void)
{
    return Motor_SpeedLoop_IsFeedbackValidState(&s_speed);
}

#ifdef __cplusplus
}
#endif
