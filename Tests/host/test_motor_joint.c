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
static uint32_t s_time_us;

uint32_t Motor_Encoder_GetAccumulatedCount(void) { return s_count; }
bool Motor_Encoder_IsZeroValid(void) { return s_zero_valid; }
bool Motor_Encoder_IsSignalFault(void) { return s_fault; }
float Motor_Encoder_GetMechRpm(void) { return s_rpm; }
bool Motor_Encoder_IsSpeedValid(void) { return s_speed_valid; }
uint32_t Hal_Motor_TimeUs(void) { return s_time_us; }

int main(void)
{
    motor_joint_state_t state;

    memset(&motor, 0, sizeof(motor));
    motor.speed_iq_limit = 0.20f;
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
    assert(!motor.joint_current_hold);
    assert(motor.joint_voltage_hold);
    assert(fabsf(motor.joint_hold_v - 1.80f) < 1e-4f);
    assert(fabsf(motor.ol_target_rpm) < 1e-5f);
    assert(!motor.position_voltage_hold);

    /* 非零速度轨迹加入方向摩擦前馈，到点速度为零时不保留。 */
    s_count = 0U;
    assert(Motor_Joint_SetKp(0.05f));
    assert(Motor_Joint_SetKd(0.0f));
    assert(Motor_Joint_SetTarget(0.0f, 1.0f, 0.0f));
    Motor_Joint_Run(0.001f);
    assert(fabsf(motor.joint_hold_v - 0.15f) < 1e-4f);
    assert(Motor_Joint_SetTarget(0.0f, 0.0f, 0.0f));
    Motor_Joint_Run(0.001f);
    assert(fabsf(motor.joint_hold_v) < 1e-4f);

    assert(Motor_Joint_SetKp(0.02f));
    assert(Motor_Joint_SetKd(0.01f));
    assert(Motor_Joint_SetTarget(90.0f, 0.0f, 0.0f));
    s_rpm = 100.0f;
    Motor_Joint_Run(0.001f);
    assert(fabsf(motor.joint_hold_v - 0.80f) < 1e-4f);

    assert(Motor_Joint_SetTarget(90.0f, 0.0f, 0.1f));
    s_rpm = 0.0f;
    Motor_Joint_Run(0.001f);
    assert(fabsf(motor.joint_hold_v - 2.00f) < 1e-4f);

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
    assert(fabsf(state.bias_v - 0.002f) < 1e-4f);
    assert(fabsf(motor.joint_hold_v - 0.11f) < 1e-4f);
    {
        int step;
        for (step = 0; step < 40; step++) {
            Motor_Joint_Run(0.05f);
        }
    }
    Motor_Joint_GetState(&state);
    assert(fabsf(state.bias_v - 0.06f) < 1e-4f);

    /* 运动阶段冻结慢积分，位置目标跳变时清掉上一落点的补偿。 */
    s_rpm = 2.0f;
    Motor_Joint_Run(0.05f);
    Motor_Joint_GetState(&state);
    assert(fabsf(state.bias_v - 0.06f) < 1e-4f);
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
    assert(fabsf(state.bias_v - 0.002f) < 1e-4f);

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
    assert(state.bias_v >= 0.018f && state.bias_v <= 0.0201f);
    assert(motor.joint_hold_v >= 0.19f && motor.joint_hold_v <= 0.2001f);

    /* 完整SI命令整体提交；超时后只保留反向速度阻尼。 */
    {
        motor_joint_command_si_t command = {
            .enable = true,
            .timeout_enable = true,
            .timeout_us = 100000U,
            .position_rad = 0.0f,
            .velocity_rad_s = 0.0f,
            .feedforward_v = 0.0f,
            .kp_v_per_rad = 8.5943669f,
            .kd_v_per_rad_s = 0.1909859f,
            .ki_v_per_rad_s = 8.5943669f,
            .output_limit_v = 2.0f,
        };

        s_time_us = 1000U;
        assert(Motor_Joint_SubmitCommand(&command));
        s_rpm = 10.0f;
        s_time_us = 102000U;
        Motor_Joint_Run(0.001f);
        Motor_Joint_GetState(&state);
        assert(state.command_timeout);
        assert(state.timeout_enabled);
        assert(state.command_age_us == 101000U);
        assert(fabsf(motor.joint_hold_v + 0.20f) < 1e-4f);
    }

    /* 混合后端：允许高于0.25 A时大误差进入Iq，回落到0.20 A以下切回Vq。 */
    motor.speed_iq_limit = 0.30f;
    assert(Motor_Joint_SetVoltageLimit(2.0f));
    assert(Motor_Joint_SetKp(0.02f));
    assert(Motor_Joint_SetKd(0.0f));
    s_count = 0U;
    assert(Motor_Joint_SetTarget(90.0f, 0.0f, 0.0f));
    Motor_Joint_Run(0.001f);
    Motor_Joint_GetState(&state);
    assert(state.hybrid_current_active);
    assert(state.hybrid_enter_count == 1U);
    assert(state.hybrid_exit_count == 0U);
    assert(fabsf(state.hybrid_peak_iq_a - 0.30f) < 1e-4f);
    assert(motor.joint_current_hold);
    assert(fabsf(motor.joint_hold_iq - 0.30f) < 1e-4f);
    assert(Motor_Joint_SetTarget(0.0f, 0.0f, 0.0f));
    Motor_Joint_Run(0.001f);
    Motor_Joint_GetState(&state);
    assert(!state.hybrid_current_active);
    assert(state.hybrid_enter_count == 1U);
    assert(state.hybrid_exit_count == 1U);
    assert(motor.joint_voltage_hold);
    assert(fabsf(motor.joint_hold_v) < 1e-4f);
    motor.speed_iq_limit = 0.20f;

    /* 直接Iq测试命令受显式电流限幅和100 ms刷新超时约束。 */
    {
        motor_joint_current_command_t command = {
            .enable = true,
            .timeout_enable = true,
            .timeout_us = 100000U,
            .current_a = 0.10f,
            .current_limit_a = 0.20f,
        };

        s_time_us = 150000U;
        assert(Motor_Joint_SubmitCurrentCommand(&command));
        Motor_Joint_Run(0.001f);
        Motor_Joint_GetState(&state);
        assert(state.backend == MOTOR_JOINT_BACKEND_CURRENT_DIRECT);
        assert(fabsf(motor.joint_hold_iq - 0.10f) < 1e-5f);
        s_time_us = 251000U;
        Motor_Joint_Run(0.001f);
        assert(fabsf(motor.joint_hold_iq) < 1e-5f);
        assert(Motor_Joint_SetTarget(0.0f, 0.0f, 0.0f));
        Motor_Joint_GetState(&state);
        assert(!state.timeout_enabled);
        assert(!state.command_timeout);
    }

    /* 显式Kt与限流齐全时，物理扭矩命令转换成Iq并走电流后端。 */
    {
        motor_joint_torque_command_t command = {
            .enable = true,
            .timeout_enable = false,
            .position_rad = 0.1f,
            .velocity_rad_s = 0.0f,
            .torque_ff_nm = 0.0f,
            .kp_nm_per_rad = 1.0f,
            .kd_nm_per_rad_s = 0.0f,
            .torque_limit_nm = 0.5f,
            .current_limit_a = 1.0f,
            .torque_constant_nm_per_a = 0.1f,
            .gear_ratio = 1.0f,
            .efficiency = 1.0f,
        };

        s_time_us = 200000U;
        s_rpm = 0.0f;
        assert(Motor_Joint_SubmitTorqueCommand(&command));
        Motor_Joint_Run(0.001f);
        Motor_Joint_GetState(&state);
        assert(state.backend == MOTOR_JOINT_BACKEND_TORQUE_CURRENT);
        assert(motor.joint_current_hold);
        assert(!motor.joint_voltage_hold);
        assert(fabsf(state.torque_command_nm - 0.1f) < 1e-4f);
        assert(fabsf(motor.joint_hold_iq - 1.0f) < 1e-4f);
    }

    s_fault = true;
    Motor_Joint_Run(0.001f);
    assert(!motor.joint_voltage_hold);
    assert(fabsf(motor.joint_hold_v) < 1e-5f);

    puts("test_motor_joint: pass");
    return 0;
}
