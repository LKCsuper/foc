#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_position_core.h"

int main(void)
{
    motor_position_config_t config = {3600U, true, 2.0f, 5.0f,
        {1.0f, 100.0f, 0.1f, 0.2f, 2.0f, 1000.0f, 0.0f, 0.05f}};
    motor_position_runtime_t a, b;
    motor_position_state_t snapshot;
    motor_position_input_t input = {10U, true, true, true, 0.0f, 0.01f};
    motor_position_output_t output;
    assert(Motor_PositionCore_Init(&a, &config, UINT32_MAX - 9U, 0.3f));
    assert(Motor_PositionCore_Init(&b, &config, 10U, 0.3f));
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(!output.owns_speed_command && !output.hold_active);
    assert(fabsf(a.state.position_deg - 2.0f) < 1e-6f);
    assert(!Motor_PositionCore_Enable(&a, true, false));
    assert(Motor_PositionCore_Enable(&a, true, true));
    assert(Motor_PositionCore_SetTarget(&a, 30.0f));
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(output.owns_speed_command && output.hold_active && !output.voltage_hold);
    assert(output.speed_rpm == 10.0f && a.control.velocity_integrator_v == 0.0f); /* 斜坡首拍10RPM */
    assert(!b.state.enabled && b.state.target_deg == 0.0f);
    input.accumulated_count = 285U; /* 相对零点29.5deg，进入电压保持 */
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(output.voltage_hold && output.speed_rpm == 0.0f);
    input.accumulated_count = 260U; /* 误差3deg，位于滞环内 */
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(output.voltage_hold);
    Motor_PositionCore_Snapshot(&a, &config, 10U, true, &snapshot);
    assert(snapshot.error_deg == 28.0f && a.state.position_deg == 27.0f);
    input.accumulated_count = 200U; /* 误差9deg，退出电压保持 */
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(!output.voltage_hold && output.speed_rpm > 0.0f);
    input.feedback_valid = false;
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(output.owns_speed_command && !output.hold_active && output.speed_rpm == 0.0f);
    Motor_PositionCore_Zero(&a, 100U);
    Motor_PositionCore_Snapshot(&a, &config, 90U, true, &snapshot);
    assert(snapshot.position_deg == -1.0f);
    assert(!Motor_PositionCore_Move(&a, NAN));
    assert(!Motor_PositionCore_SetMaxRpm(&a, 101.0f, 100.0f));
    config.counts_per_rev = 0U;
    assert(!Motor_PositionCore_Init(&a, &config, 0U, 0.3f));
    assert(a.state.origin_count == 100);
    puts("Position command/mode standalone tests passed");
    return 0;
}
