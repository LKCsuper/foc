#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_speed.h"
#include "motor_param.h"

/* 默认包装路径会引用保护反馈；本测试只走 State API，提供空桩。 */
void Motor_Protect_SetSpeedFeedback(float rpm, bool valid)
{
    (void)rpm;
    (void)valid;
}

int main(void)
{
    motor_speed_state_t a;
    motor_speed_state_t b;
    MOTOR_PARAM_S motor_a;
    MOTOR_PARAM_S motor_b;
    float dt = 0.001f;
    unsigned i;
    motor_speed_snapshot_t speed_snapshot;

    memset(&motor_a, 0, sizeof(motor_a));
    memset(&motor_b, 0, sizeof(motor_b));
    motor_a.ol_enable = true;
    motor_a.ol_mode = MOTOR_OL_MODE_IF;
    motor_a.speed_enable = true;
    motor_a.speed_loop_hz = 1000.0f;
    motor_a.speed_kp = 0.01f;
    motor_a.speed_ki = 0.0f;
    motor_a.speed_iq_limit = 5.0f;
    motor_a.ol_current_rpm = 1000.0f;
    motor_a.speed_allow_braking = true;
    motor_b = motor_a;
    motor_b.ol_current_rpm = -800.0f;

    Motor_Speed_InitState(&a);
    Motor_Speed_InitState(&b);
    Motor_SpeedFeedback_SetState(&a, 900.0f, true);
    Motor_SpeedFeedback_SetState(&b, -700.0f, true);

    for (i = 0; i < 5U; ++i) {
        Motor_SpeedLoop_RunState(&a, &motor_a, dt);
        Motor_SpeedLoop_RunState(&b, &motor_b, dt);
    }

    /* 两实例误差与输出符号应各自独立，互不覆盖。 */
    assert(Motor_SpeedLoop_IsFeedbackValidState(&a));
    assert(Motor_SpeedLoop_IsFeedbackValidState(&b));
    assert(Motor_SpeedLoop_GetMeasuredRpmState(&a) == 900.0f);
    assert(Motor_SpeedLoop_GetMeasuredRpmState(&b) == -700.0f);
    assert(Motor_SpeedLoop_GetErrorState(&a) > 0.0f);
    assert(Motor_SpeedLoop_GetErrorState(&b) < 0.0f);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) > 0.0f);
    assert(Motor_SpeedLoop_GetIqTargetState(&b) < 0.0f);

    Motor_SpeedLoop_PrimeIqState(&a, 1.5f);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) == 1.5f);
    assert(Motor_SpeedLoop_GetIqTargetState(&b) < 0.0f);

    Motor_SpeedLoop_ResetState(&b);
    assert(Motor_SpeedLoop_GetIqTargetState(&b) == 0.0f);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) == 1.5f);
    assert(Motor_SpeedLoop_GetMeasuredRpmState(&b) == -700.0f);

    /* 零速命令且禁用制动时必须释放历史积分，不继续输出正向保持转矩。 */
    Motor_SpeedLoop_PrimeIqState(&a, 1.5f);
    motor_a.ol_target_rpm = 0.0f;
    motor_a.speed_allow_braking = false;
    Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) == 0.0f);

    /* 位置保持时零速是定住目标，禁止制动也不能把保持转矩清掉。 */
    Motor_SpeedLoop_PrimeIqState(&a, 1.5f);
    motor_a.position_hold = true;
    motor_a.ol_current_rpm = 0.0f;
    Motor_SpeedFeedback_SetState(&a, 0.0f, true);
    Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) == 1.5f);
    motor_a.position_hold = false;

    /* Hall 制动越过零点后必须锁定释放，不得反向满电流形成往返振荡。 */
    Motor_SpeedLoop_ResetState(&a);
    motor_a.ol_target_rpm = 0.0f;
    motor_a.ol_current_rpm = 0.0f;
    motor_a.speed_allow_braking = true;
    Motor_SpeedFeedback_SetState(&a, 100.0f, true);
    Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) < 0.0f);
    Motor_SpeedFeedback_SetState(&a, -100.0f, true);
    Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) == 0.0f);
    Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) == 0.0f);

    /* Hall 无新边沿而超时归零时也要释放制动电流。 */
    Motor_SpeedLoop_ResetState(&a);
    Motor_SpeedFeedback_SetState(&a, 100.0f, true);
    Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) < 0.0f);
    Motor_SpeedFeedback_SetState(&a, 0.0f, true);
    Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) == 0.0f);

    /* 新的非零转速指令解除零速释放锁定。 */
    motor_a.ol_target_rpm = 100.0f;
    motor_a.ol_current_rpm = 100.0f;
    Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    assert(!a.zero_brake_released);

    /* 允许制动时，确认机械速度已为零也要释放减速阶段残留积分。 */
    Motor_SpeedLoop_PrimeIqState(&a, 1.5f);
    motor_a.ol_target_rpm = 0.0f;
    motor_a.ol_current_rpm = 0.0f;
    motor_a.speed_allow_braking = true;
    Motor_SpeedFeedback_SetState(&a, 0.0f, true);
    Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) == 0.0f);

    /* 默认包装：未允许时 Run 为空操作；允许后才积分。 */
    Motor_Speed_Init();
    Motor_SpeedFeedback_Set(900.0f, true);
    Motor_SpeedLoop_SetRunAllowed(false);
    Motor_SpeedLoop_RunScheduled(&motor_a, 0.00099f);
    assert(Motor_SpeedLoop_GetSnapshot(&speed_snapshot) && speed_snapshot.update_count == 0U);
    Motor_SpeedLoop_Run(&motor_a, dt);
    assert(Motor_SpeedLoop_GetIqTarget() == 0.0f);
    Motor_SpeedLoop_SetRunAllowed(true);
    Motor_SpeedLoop_RunScheduled(&motor_a, 0.00099f);
    assert(Motor_SpeedLoop_GetSnapshot(&speed_snapshot) && speed_snapshot.update_count == 1U);
    Motor_SpeedLoop_PrimeIq(0.5f);
    Motor_SpeedLoop_Run(&motor_a, dt);
    assert(Motor_SpeedLoop_GetIqTarget() != 0.0f);
    assert(Motor_SpeedLoop_GetSnapshot(&speed_snapshot));
    assert(fabsf(speed_snapshot.error -
                 (speed_snapshot.target_rpm - speed_snapshot.measured_rpm)) < 1e-6f);
    Motor_SpeedLoop_Reset();
    assert(Motor_SpeedLoop_GetIqTarget() == 0.0f);

    /* 输出顶满时积分要泄放，不能停在限幅上把转速继续推过目标。 */
    Motor_SpeedLoop_ResetState(&a);
    motor_a.ol_target_rpm = 150.0f;
    motor_a.ol_current_rpm = 150.0f;
    motor_a.speed_kp = 0.002f;
    motor_a.speed_ki = 0.004f;
    motor_a.speed_iq_limit = 0.08f;
    motor_a.speed_allow_braking = true;
    motor_a.speed_loop_hz = 1000.0f;
    Motor_SpeedFeedback_SetState(&a, 80.0f, true);
    Motor_SpeedLoop_PrimeIqState(&a, 0.08f);
    for (i = 0; i < 30U; ++i) {
        Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    }
    /* 1 kHz 下约每拍乘 0.99；30 ms 后应保留约 0.059 A。 */
    assert(a.integral < 0.065f);
    assert(a.integral > 0.055f);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) > 0.0f);

    /* 禁用制动时，负速度误差的 PI 仍给出负修正，不在 PI 内部钳成 0。 */
    Motor_SpeedLoop_ResetState(&a);
    motor_a.ol_target_rpm = 30.0f;
    motor_a.ol_current_rpm = 30.0f;
    motor_a.speed_kp = 0.01f;
    motor_a.speed_ki = 0.0f;
    motor_a.speed_iq_limit = 5.0f;
    motor_a.speed_allow_braking = false;
    motor_a.position_hold = false;
    motor_a.position_voltage_hold = false;
    Motor_SpeedFeedback_SetState(&a, 40.0f, true);
    Motor_SpeedLoop_RunState(&a, &motor_a, dt);
    assert(Motor_SpeedLoop_GetErrorState(&a) < 0.0f);
    assert(Motor_SpeedLoop_GetIqTargetState(&a) < 0.0f);

    /* 负修正小于前馈时最终电压仍为正；大于前馈时收到 0，不允许制动时不反向。 */
    assert(Motor_Speed_LimitNoBrakeOutput(30.0f, 0.37f - 0.10f, false) > 0.20f);
    assert(Motor_Speed_LimitNoBrakeOutput(30.0f, 0.37f - 0.10f, false) < 0.37f);
    assert(fabsf(Motor_Speed_LimitNoBrakeOutput(30.0f, 0.37f - 1.0f, false)) < 1e-6f);
    assert(fabsf(Motor_Speed_LimitNoBrakeOutput(-30.0f, -0.37f + 1.0f, false)) < 1e-6f);
    assert(Motor_Speed_LimitNoBrakeOutput(30.0f, 0.37f - 1.0f, true) < 0.0f);

    printf("test_motor_speed: pass\n");
    return 0;
}
