#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_joint_control.h"

int main(void)
{
    motor_joint_control_t control;
    motor_joint_control_config_t config = {
        .kp_output_per_rad = 2.0f,
        .kd_output_per_rad_s = 3.0f,
        .ki_output_per_rad_s = 4.0f,
        .output_limit = 10.0f,
        .integral_limit = 2.0f,
        .integral_max_error_rad = 1.0f,
        .integral_max_velocity_rad_s = 2.0f,
        .integral_max_target_rad_s = 3.0f,
        .target_reset_threshold_rad = 0.01f,
        .dt_max_s = 0.2f,
    };
    motor_joint_control_command_t command = {
        .position_rad = 1.0f,
        .velocity_rad_s = 2.0f,
        .feedforward_output = 0.5f,
    };
    motor_joint_control_feedback_t feedback = {
        .position_rad = 0.5f,
        .velocity_rad_s = 1.0f,
        .velocity_valid = true,
    };
    float output;

    assert(Motor_JointControl_Init(&control, &config));
    assert(Motor_JointControl_SetCommand(&control, &command));
    output = Motor_JointControl_Step(&control, &feedback, 0.1f, true);
    assert(fabsf(output - 4.7f) < 1e-5f);
    assert(control.integral_active);
    assert(fabsf(control.integral_output - 0.2f) < 1e-5f);

    /* 速度无效时不使用阻尼，也不能继续积分。 */
    feedback.velocity_valid = false;
    output = Motor_JointControl_Step(&control, &feedback, 0.1f, true);
    assert(fabsf(output - 1.7f) < 1e-5f);
    assert(!control.integral_active);
    assert(fabsf(control.integral_output - 0.2f) < 1e-5f);

    /* 位置目标跳变清除旧落点积分。 */
    command.position_rad = 1.02f;
    assert(Motor_JointControl_SetCommand(&control, &command));
    assert(fabsf(control.integral_output) < 1e-6f);

    /* 输出饱和时不允许积分继续向饱和方向累积。 */
    config.output_limit = 1.0f;
    config.integral_limit = 1.0f;
    assert(Motor_JointControl_Configure(&control, &config));
    feedback.position_rad = 0.0f;
    feedback.velocity_rad_s = 0.0f;
    feedback.velocity_valid = true;
    output = Motor_JointControl_Step(&control, &feedback, 0.1f, true);
    assert(fabsf(output - 1.0f) < 1e-6f);
    assert(fabsf(control.integral_output) < 1e-6f);

    output = Motor_JointControl_Step(&control, &feedback, 0.1f, false);
    assert(fabsf(output) < 1e-6f);
    assert(fabsf(control.integral_output) < 1e-6f);

    puts("test_motor_joint_control: pass");
    return 0;
}
