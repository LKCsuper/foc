#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_position.h"
#include "motor_param.h"

MOTOR_PARAM_S motor;
static uint32_t s_count;
static bool s_zero_valid = true;
static bool s_fault;

uint32_t Motor_Encoder_GetAccumulatedCount(void) { return s_count; }
bool Motor_Encoder_IsZeroValid(void) { return s_zero_valid; }
bool Motor_Encoder_IsSignalFault(void) { return s_fault; }
static float s_rpm;
static bool s_speed_valid;
float Motor_Encoder_GetMechRpm(void) { return s_rpm; }
bool Motor_Encoder_IsSpeedValid(void) { return s_speed_valid; }

int main(void)
{
    motor_position_state_t state;
    memset(&motor, 0, sizeof(motor));
    Motor_Position_Init();
    Motor_Position_Zero();
    assert(Motor_Position_SetTargetDeg(90.0f));
    assert(Motor_Position_SetKp(0.5f));
    assert(Motor_Position_SetMaxRpm(30.0f));
    assert(Motor_Position_Enable(true));
    motor.ol_enable = true;
    Motor_Position_Run(0.001f);
    Motor_Position_GetState(&state);
    assert(fabsf(state.target_rpm - 30.0f) < 1e-4f);
    assert(fabsf(motor.ol_target_rpm) < 1e-5f);
    assert(motor.position_voltage_hold);
    assert(motor.position_hold_v > 0.5f);
    s_count = 2048U;
    Motor_Position_Run(0.001f);
    Motor_Position_GetState(&state);
    assert(fabsf(state.target_rpm - 22.5f) < 1e-3f);
    s_count = 4096U;
    Motor_Position_Run(0.001f);
    Motor_Position_GetState(&state);
    assert(fabsf(state.target_rpm) < 1e-4f);
    assert(motor.position_hold_v > 0.0f);
    assert(Motor_Position_MoveDeg(-180.0f));
    Motor_Position_Run(0.001f);
    Motor_Position_GetState(&state);
    assert(fabsf(state.target_rpm + 30.0f) < 1e-3f);
    assert(motor.position_hold_v < -0.5f);
    s_fault = true;
    Motor_Position_Run(0.001f);
    Motor_Position_GetState(&state);
    assert(fabsf(state.target_rpm) < 1e-5f);
    assert(!motor.position_hold);
    assert(fabsf(motor.position_hold_v) < 1e-5f);

    /* 停在目标外侧时积分会积出电压；到达后仍留着。轴还在转时比例项负责阻尼。 */
    s_fault = false;
    s_count = 0U;
    Motor_Position_Zero();
    assert(Motor_Position_SetKp(20.0f * 60.0f / 360.0f));
    assert(Motor_Position_SetMaxRpm(60.0f));
    assert(Motor_Position_SetTargetDeg(0.20f));
    assert(Motor_Position_Enable(true));
    motor.ol_enable = true;
    s_speed_valid = true;
    s_rpm = 0.0f;
    for (int i = 0; i < 200; i++) {
        Motor_Position_Run(0.001f);
    }
    assert(motor.position_voltage_hold);
    assert(fabsf(motor.ol_target_rpm) < 1e-5f);
    assert(motor.position_hold_v > 0.04f);
    Motor_Position_Zero();
    assert(Motor_Position_SetTargetDeg(0.0f));
    s_rpm = 10.0f;
    Motor_Position_Run(0.001f);
    assert(motor.position_hold_v < -0.40f);
    puts("test_motor_position: pass");
    return 0;
}
