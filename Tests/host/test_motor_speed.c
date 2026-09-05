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

    printf("test_motor_speed: pass\n");
    return 0;
}
