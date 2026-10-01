#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_joint.h"
#include "motor_param.h"

MOTOR_PARAM_S motor;
static uint32_t s_count;
static bool s_zero_valid = true;
static bool s_fault;
static float s_rpm;
static bool s_speed_valid;

uint32_t Motor_Encoder_GetAccumulatedCount(void) { return s_count; }
bool Motor_Encoder_IsZeroValid(void) { return s_zero_valid; }
bool Motor_Encoder_IsSignalFault(void) { return s_fault; }
float Motor_Encoder_GetMechRpm(void) { return s_rpm; }
bool Motor_Encoder_IsSpeedValid(void) { return s_speed_valid; }

int main(void)
{
    motor_joint_state_t state;

    memset(&motor, 0, sizeof(motor));
    Motor_Joint_Init();
    Motor_Joint_Zero();
    assert(Motor_Joint_SetKp(0.02f));
    assert(Motor_Joint_SetKd(0.0f));
    assert(Motor_Joint_SetKi(0.0f));
    assert(Motor_Joint_SetVoltageLimit(2.0f));
    assert(Motor_Joint_SetTarget(90.0f, 0.0f, 0.0f));
    assert(Motor_Joint_Enable(true));
    motor.ol_enable = true;
    s_speed_valid = true;
    s_rpm = 0.0f;
    Motor_Joint_Run(0.001f);
    assert(motor.joint_voltage_hold);
    assert(fabsf(motor.joint_hold_v - 1.8f) < 1e-4f);
    assert(fabsf(motor.ol_target_rpm) < 1e-5f);
    assert(!motor.position_voltage_hold);

    assert(Motor_Joint_SetKd(0.01f));
    assert(Motor_Joint_SetTarget(90.0f, 0.0f, 0.0f));
    s_rpm = 100.0f;
    Motor_Joint_Run(0.001f);
    assert(fabsf(motor.joint_hold_v - 0.8f) < 1e-4f);

    assert(Motor_Joint_SetTarget(90.0f, 0.0f, 0.1f));
    s_rpm = 0.0f;
    Motor_Joint_Run(0.001f);
    assert(fabsf(motor.joint_hold_v - 2.0f) < 1e-4f);

    s_count = 4096U;
    assert(Motor_Joint_SetKd(0.0f));
    assert(Motor_Joint_SetTarget(90.0f, 0.0f, 0.0f));
    Motor_Joint_Run(0.001f);
    Motor_Joint_GetState(&state);
    assert(fabsf(state.error_deg) < 1e-3f);
    assert(fabsf(motor.joint_hold_v) < 1e-4f);

    s_count = 0U;
    assert(Motor_Joint_SetKp(0.05f));
    assert(Motor_Joint_SetKd(0.0f));
    assert(Motor_Joint_SetKi(0.10f));
    assert(Motor_Joint_SetTarget(2.0f, 0.0f, 0.0f));
    assert(Motor_Joint_Enable(true));
    Motor_Joint_Run(0.05f);
    Motor_Joint_GetState(&state);
    assert(fabsf(state.bias_v - 0.01f) < 1e-4f);
    assert(fabsf(motor.joint_hold_v - 0.11f) < 1e-4f);
    {
        int step;
        for (step = 0; step < 40; step++) {
            Motor_Joint_Run(0.05f);
        }
    }
    Motor_Joint_GetState(&state);
    assert(fabsf(state.bias_v - 0.30f) < 1e-4f);

    /* 运动阶段冻结慢积分，位置目标跳变时清掉上一落点的补偿。 */
    s_rpm = 2.0f;
    Motor_Joint_Run(0.05f);
    Motor_Joint_GetState(&state);
    assert(fabsf(state.bias_v - 0.30f) < 1e-4f);
    assert(Motor_Joint_SetTarget(3.0f, 0.0f, 0.0f));
    Motor_Joint_GetState(&state);
    assert(fabsf(state.bias_v) < 1e-5f);
    s_rpm = 0.0f;
    Motor_Joint_Run(0.05f);
    Motor_Joint_GetState(&state);
    assert(fabsf(state.bias_v) < 1e-5f);
    assert(Motor_Joint_SetTarget(2.0f, 0.0f, 0.0f));
    Motor_Joint_Run(0.05f);
    Motor_Joint_GetState(&state);
    assert(fabsf(state.bias_v - 0.01f) < 1e-4f);

    assert(Motor_Joint_SetKi(0.0f));
    assert(Motor_Joint_SetVoltageLimit(0.20f));
    assert(Motor_Joint_SetKi(0.10f));
    {
        int step;
        for (step = 0; step < 20; step++) {
            Motor_Joint_Run(0.05f);
        }
    }
    Motor_Joint_GetState(&state);
    assert(fabsf(state.bias_v - 0.10f) < 1e-3f);
    assert(fabsf(motor.joint_hold_v - 0.20f) < 1e-4f);

    s_fault = true;
    Motor_Joint_Run(0.001f);
    assert(!motor.joint_voltage_hold);
    assert(fabsf(motor.joint_hold_v) < 1e-5f);

    puts("test_motor_joint: pass");
    return 0;
}
