#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_position.h"
#include "motor_position_control.h"
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
    /* 位置环关闭时不得覆盖普通速度环命令。 */
    motor.ol_target_rpm = 150.0f;
    Motor_Position_Run(0.001f);
    assert(fabsf(motor.ol_target_rpm - 150.0f) < 1e-5f);
    motor.ol_target_rpm = 0.0f;
#ifdef MOTOR_POSITION_HYBRID_ONLY
    assert(Motor_Position_SetKp(1.0f));
    assert(Motor_Position_SetMaxRpm(30.0f));
    assert(Motor_Position_SetTargetDeg(30.0f));
    assert(Motor_Position_Enable(true));
    motor.ol_enable = true;
    s_speed_valid = true;
    Motor_Position_Run(0.001f);
    Motor_Position_GetState(&state);
    assert(state.enabled);
    assert(!state.voltage_hold_mode);
    assert(motor.position_hold);
    assert(!motor.position_voltage_hold);
    assert(motor.ol_target_rpm > 29.0f);
    s_count = 1320U; /* 约 29 deg，进入目标附近直接电压保持。 */
    Motor_Position_Run(0.001f);
    Motor_Position_GetState(&state);
    assert(state.voltage_hold_mode);
    assert(motor.position_voltage_hold);
    assert(fabsf(motor.ol_target_rpm) < 1e-5f);
    puts("test_motor_position_hybrid_2804: pass");
    return 0;
#endif
    assert(Motor_Position_SetTargetDeg(90.0f));
    assert(Motor_Position_SetKp(0.5f));
    assert(Motor_Position_SetMaxRpm(30.0f));
    assert(Motor_Position_Enable(true));
    /* 改目标后即使控制任务尚未运行，命令行快照也必须反映新误差。 */
    Motor_Position_GetState(&state);
    assert(fabsf(state.error_deg - 90.0f) < 1e-4f);
    assert(fabsf(state.target_rpm - 30.0f) < 1e-4f);
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

    /* 纯控制模块由调用者持有状态，两实例交替步进不得互相污染积分。 */
    {
        motor_position_control_config_t config = {1.0f, 30.0f, 0.05f, 0.20f, 2.0f, 30000.0f, 0.0f, 0.05f};
        motor_position_control_t control_a;
        motor_position_control_t control_b;
        motor_position_control_input_t input = {true, true, true, true, 10.0f, 0.0f, 0.0f, 0.01f};
        motor_position_control_output_t output_a;
        motor_position_control_output_t output_b;

        assert(Motor_PositionControl_Init(&control_a, &config));
        assert(Motor_PositionControl_Init(&control_b, &config));
        assert(Motor_PositionControl_Step(&control_a, &config, &input, &output_a));
        assert(Motor_PositionControl_Step(&control_a, &config, &input, &output_a));
        input.target_deg = -10.0f;
        assert(Motor_PositionControl_Step(&control_b, &config, &input, &output_b));
        assert(output_a.output_v > 0.0f);
        assert(output_b.output_v < 0.0f);
        assert(control_a.velocity_integrator_v > 0.0f);
        assert(control_b.velocity_integrator_v < 0.0f);
        assert(fabsf(control_a.velocity_integrator_v + control_b.velocity_integrator_v) > 1e-4f);
    }

    /* 小惯量电机的速度命令必须按加速度斜坡推进，不能由 0 瞬跳到限速值。 */
    {
        motor_position_control_config_t config = {1.0f, 30.0f, 0.05f, 0.20f, 2.0f, 5.0f, 0.0f, 0.05f};
        motor_position_control_t control;
        motor_position_control_input_t input = {true, true, true, true, 30.0f, 0.0f, 0.0f, 0.01f};
        motor_position_control_output_t output;

        assert(Motor_PositionControl_Init(&control, &config));
        assert(Motor_PositionControl_Step(&control, &config, &input, &output));
        assert(fabsf(output.target_rpm - 0.05f) < 1e-5f);
        assert(Motor_PositionControl_Step(&control, &config, &input, &output));
        assert(fabsf(output.target_rpm - 0.10f) < 1e-5f);
        config.feedforward_v_per_rpm = 0.02f;
        Motor_PositionControl_Reset(&control);
        assert(Motor_PositionControl_Step(&control, &config, &input, &output));
        assert(output.output_v > 0.0f);
        assert(output.output_v > config.velocity_kp_v_per_rpm * output.target_rpm);
    }
    puts("test_motor_position: pass");
    return 0;
}
