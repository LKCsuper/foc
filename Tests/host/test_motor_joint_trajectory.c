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
static uint32_t s_time_us;

uint32_t Motor_Encoder_GetAccumulatedCount(void) { return s_count; }
bool Motor_Encoder_IsZeroValid(void) { return true; }
bool Motor_Encoder_IsSignalFault(void) { return false; }
float Motor_Encoder_GetMechRpm(void) { return 0.0f; }
bool Motor_Encoder_IsSpeedValid(void) { return true; }
uint32_t Hal_Motor_TimeUs(void) { return s_time_us; }

int main(void)
{
    motor_joint_state_t state;
    int i;

    memset(&motor, 0, sizeof(motor));
    motor.speed_iq_limit = 0.30f;
    motor.ol_enable = true;
    Motor_Joint_Init();
    Motor_Joint_Zero();
    assert(Motor_Joint_SetKp(0.10f));
    assert(Motor_Joint_SetKd(0.02f));
    assert(Motor_Joint_SetKi(0.0f));
    assert(Motor_Joint_SetVoltageLimit(1.0f));
    assert(Motor_Joint_Enable(true));
    assert(Motor_Joint_SetTarget(90.0f, 0.0f, 0.0f));

    Motor_Joint_Run(0.001f);
    Motor_Joint_GetState(&state);
    assert(state.trajectory_active);
    assert(fabsf(state.target_deg - 90.0f) < 1e-5f);
    assert(state.planned_position_deg > 0.0f && state.planned_position_deg < 0.01f);
    assert(state.planned_rpm > 0.0f && state.planned_rpm <= MOTOR_JOINT_TRAJECTORY_VEL_DEG_S / 6.0f);

    for (i = 0; i < 20; i++) {
        s_time_us += 1000U;
        Motor_Joint_Run(0.001f);
    }
    Motor_Joint_GetState(&state);
    assert(state.planned_rpm > 0.0f);
    assert(state.iq_command_a >= 0.03f);

    for (i = 0; i < 5000 && state.trajectory_active; i++) {
        s_time_us += 1000U;
        Motor_Joint_Run(0.001f);
        Motor_Joint_GetState(&state);
    }
    assert(!state.trajectory_active);
    assert(fabsf(state.planned_position_deg - 90.0f) < 1e-4f);
    assert(fabsf(state.planned_rpm) < 1e-4f);
    puts("test_motor_joint_trajectory: pass");
    return 0;
}
