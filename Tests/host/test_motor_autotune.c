#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include "motor_autotune.h"
#include "motor_pid.h"

static motor_autotune_config_t config(void)
{
    motor_autotune_config_t c = {0.5f, 0.001f, 0.002f, 1.0f / 15000.0f, 150.0f,
                                1.5f / 15000.0f, 60.0f};
    return c;
}

/* 用真实 PI 与连续 RL 的精确 ZOH 响应闭环验证，含一拍输出延迟。
 * 不使用 Kp/L 代数关系作为唯一测试，检查跟踪与过冲。 */
static void check_step(float r, float l, motor_autotune_pi_t gains, float dt)
{
    float current = 0.0f, integral = 0.0f, applied = 0.0f, peak = 0.0f;
    float a = expf(-r * dt / l);
    unsigned k;
    for (k = 0; k < 3000; k++) {
        float next = Motor_Current_PI_Run(0.2f, current, &integral, gains.kp, gains.ki, dt, 12.0f);
        current = a * current + (1.0f - a) / r * applied;
        applied = next;
        assert(isfinite(current));
        peak = fmaxf(peak, current);
    }
    assert(fabsf(current - 0.2f) < 0.0001f);
    assert(peak < 0.24f);
}

int main(void)
{
    motor_autotune_config_t c = config();
    motor_autotune_result_t result;
    assert(Motor_Autotune_CurrentPi(&c, &result) == MOTOR_AUTOTUNE_OK);
    assert(fabsf(result.d.kp - 0.9424778f) < 0.00001f);
    assert(fabsf(result.d.ki - 471.2389f) < 0.01f);
    assert(result.shared.kp == result.d.kp);
    check_step(c.resistance_ohm, c.ld_h, result.d, c.control_dt_s);
    check_step(c.resistance_ohm, c.lq_h, result.q, c.control_dt_s);
    check_step(c.resistance_ohm, c.lq_h, result.shared, c.control_dt_s);
    c.bandwidth_hz = 1000.0f;
    assert(Motor_Autotune_CurrentPi(&c, &result) == MOTOR_AUTOTUNE_SAMPLE_LIMIT);
    assert(result.d.kp == 0.0f);
    c = config(); c.total_delay_s = 0.001f;
    assert(Motor_Autotune_CurrentPi(&c, &result) == MOTOR_AUTOTUNE_DELAY_LIMIT);
    c = config(); c.total_delay_s = 0.0f;
    assert(Motor_Autotune_CurrentPi(&c, &result) == MOTOR_AUTOTUNE_INVALID_INPUT);
    c = config(); c.ld_h = NAN;
    assert(Motor_Autotune_CurrentPi(&c, &result) == MOTOR_AUTOTUNE_INVALID_INPUT);
    c = config(); c.resistance_ohm = -1.0f;
    assert(Motor_Autotune_CurrentPi(&c, &result) == MOTOR_AUTOTUNE_INVALID_INPUT);
    c = config(); c.bandwidth_hz = INFINITY;
    assert(Motor_Autotune_CurrentPi(&c, &result) == MOTOR_AUTOTUNE_INVALID_INPUT);
    c = config(); c.ld_h = FLT_MAX;
    assert(Motor_Autotune_CurrentPi(&c, &result) == MOTOR_AUTOTUNE_NUMERIC_ERROR);
    assert(Motor_Autotune_CurrentPi(NULL, &result) == MOTOR_AUTOTUNE_INVALID_INPUT);
    assert(result.d.kp == 0.0f);
    assert(Motor_Autotune_CurrentPi(&c, NULL) == MOTOR_AUTOTUNE_INVALID_INPUT);
    puts("motor autotune: candidate validation and real PI/RL step passed");
    return 0;
}
