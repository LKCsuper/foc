/* 保护与位置控制独立核心：过流/欠压/失速/丢速故障与位置保持输出。 */
#include <assert.h>
#include <stdio.h>
#include "motor_protect_core.h"
#include "motor_position_control.h"

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_protect_state_t a, b;
    motor_protect_input_t input = {0};
    motor_protect_snapshot_t snapshot;
    motor_protect_aux_t aux = {0};
    motor_position_control_t position, other;
    motor_position_control_config_t config = {2.0f, 100.0f, 0.1f, 0.2f, 2.0f, 1000.0f, 0.0f, 0.05f};
    motor_position_control_input_t pos_input = {true, true, true, true, 30.0f, 0.0f, 0.0f, 0.01f};
    motor_position_control_output_t output;

    Motor_Protect_InitState(&a, NULL);
    Motor_Protect_InitState(&b, NULL);
    input.ia = -12.0f;
    input.v_bus = 5.0f;
    input.ol_enable = true;
    input.ol_target_rpm = input.ol_current_rpm = 600.0f;
    aux.iq_command = 0.7f;
    assert(!Motor_Protect_RunInputState(&a, &input, 0.0005f, &aux));
    assert(Motor_Protect_RunInputState(&a, &input, 0.0005f, &aux));
    assert(Motor_Protect_GetFaultState(&a) == FAULT_CODE_ABS_OVER_CURRENT);
    assert(!Motor_Protect_IsFaultLatchedState(&b));
    input.ia = 0.0f;
    Motor_Protect_TriggerFaultInputState(&a, FAULT_CODE_UNDER_VOLTAGE, &input, NULL);
    Motor_Protect_GetSnapshotState(&a, &snapshot);
    assert(snapshot.current_peak == 12.0f && snapshot.iq_command == 0.7f);
    Motor_Protect_ClearFaultState(&a);
    assert(Motor_Protect_RunInputState(&a, &input, 0.1f, NULL));
    assert(Motor_Protect_GetFaultState(&a) == FAULT_CODE_UNDER_VOLTAGE);
    Motor_Protect_ClearFaultState(&a);
    input.v_bus = 16.0f;
    input.ia = 1.0f;
    Motor_Protect_SetSpeedFeedbackState(&a, 0.0f, true);
    assert(Motor_Protect_RunInputState(&a, &input, 1.0f, NULL));
    assert(Motor_Protect_GetFaultState(&a) == FAULT_CODE_MOTOR_STALL);
    Motor_Protect_ClearFaultState(&a);
    Motor_Protect_SetSpeedFeedbackState(&a, 600.0f, true);
    assert(!Motor_Protect_RunInputState(&a, &input, 0.01f, NULL));
    Motor_Protect_SetSpeedFeedbackState(&a, 100.0f, true);
    assert(Motor_Protect_RunInputState(&a, &input, 0.3f, NULL));
    assert(Motor_Protect_GetFaultState(&a) == FAULT_CODE_MOTOR_SPEED_LOSS);
    Motor_Protect_ClearFaultState(&a);
    Motor_Protect_SetSpeedFeedbackState(&a, 0.0f, false);
    assert(!Motor_Protect_RunInputState(&a, &input, 2.0f, NULL));

    assert(Motor_PositionControl_Init(&position, &config));
    assert(Motor_PositionControl_Init(&other, &config));
    assert(Motor_PositionControl_Step(&position, &config, &pos_input, &output));
    assert(output.hold_active && output.error_deg == 30.0f);
    assert(output.output_v <= 2.0f && output.output_v > 0.0f);
    assert(other.commanded_rpm == 0.0f && other.velocity_integrator_v == 0.0f);
    pos_input.feedback_valid = false;
    assert(Motor_PositionControl_Step(&position, &config, &pos_input, &output));
    assert(!output.hold_active && output.output_v == 0.0f);
    assert(position.velocity_integrator_v == 0.0f);
    puts("Protection and position standalone tests passed");
    return 0;
}
