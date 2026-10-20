#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_speed_core.h"
#include "motor_task_clock.h"

int main(void)
{
    {
        motor_task_period_cache_t cache = {0};
        const float frequencies[] = {1.0f, 10.0f, 333.0f, 500.0f, 1000.0f, 1500.0f, 20000.0f};
        unsigned index;
        for (index = 0U; index < sizeof(frequencies) / sizeof(frequencies[0]); index++) {
            uint32_t expected = (uint32_t)(10000.0f / frequencies[index]);
            if (expected == 0U) expected = 1U;
            assert(Motor_Task_PeriodTicks(&cache, 10000U, frequencies[index]) == expected);
            assert(Motor_Task_PeriodTicks(&cache, 10000U, frequencies[index]) == expected);
        }
        assert(Motor_Task_PeriodTicks(&cache, 1000U, 500.0f) == 2U);
        assert(Motor_Task_PeriodTicks(&cache, 10000U, 500.0f) == 20U);
        assert(Motor_Task_PeriodTicks(&cache, 10000U, 1000.0f) == 10U);
        assert(Motor_Task_PeriodTicks(&cache, 0U, 1000.0f) == 1U);
        assert(Motor_Task_PeriodTicks(&cache, 10000U, -1.0f) == 10000U);
        assert(Motor_Task_PeriodTicks(&cache, 10000U, NAN) == 10000U);
        assert(Motor_Task_PeriodTicks(&cache, 10000U, INFINITY) == 1U);
        assert(Motor_Task_PeriodTicks(&cache, UINT32_MAX, 1.0f) == UINT32_MAX);
    }
    assert(Motor_Task_NextDeadline(10U, 12U, 10U) == 20U);
    assert(Motor_Task_NextDeadline(10U, 20U, 10U) == 30U);
    assert(Motor_Task_NextDeadline(10U, 52U, 10U) == 60U);
    assert(Motor_Task_NextDeadline(UINT32_MAX - 4U, UINT32_MAX - 2U, 10U) == 5U);
    assert(Motor_Task_NextDeadline(UINT32_MAX - 4U, 6U, 10U) == 15U);
    assert(Motor_Task_NextDeadline(10U, 12U, 0U) == 13U);
    {
        motor_speed_state_t scheduled;
        motor_speed_input_t config = {true, false, true, false, 100.0f, 100.0f, 1000.0f, 0.0f, 1.0f, 2.0f};
        Motor_Speed_InitState(&scheduled);
        Motor_SpeedFeedback_SetState(&scheduled, 0.0f, true);
        assert(Motor_SpeedLoop_StepScheduledState(&scheduled, &config, 0.00099f));
        assert(scheduled.update_count == 1U);
        assert(fabsf(scheduled.integral - 0.099f) < 1e-6f);
        assert(Motor_SpeedLoop_StepScheduledState(&scheduled, &config, 0.00101f));
        assert(scheduled.update_count == 2U);
        assert(fabsf(scheduled.integral - 0.2f) < 1e-6f);
        config.inhibited = true;
        assert(!Motor_SpeedLoop_StepScheduledState(&scheduled, &config, 0.001f));
        assert(scheduled.update_count == 2U && scheduled.iq_target == 0.0f);
        config.inhibited = false;
        Motor_SpeedFeedback_SetState(&scheduled, 0.0f, false);
        assert(!Motor_SpeedLoop_StepScheduledState(&scheduled, &config, 0.001f));
        assert(scheduled.update_count == 2U);
    }
    motor_speed_state_t a, b;
    motor_speed_input_t input = {true, false, true, false, 100.0f, 80.0f, 100.0f, 0.01f, 0.1f, 2.0f};
    Motor_Speed_InitState(&a);
    Motor_Speed_InitState(&b);
    Motor_SpeedFeedback_SetState(&a, 60.0f, true);
    Motor_SpeedFeedback_SetState(&b, 80.0f, true);
    assert(Motor_SpeedLoop_StepState(&a, &input, 0.005f));
    assert(a.iq_target == 0.0f);
    assert(a.update_count == 0U);
    assert(Motor_SpeedLoop_StepState(&a, &input, 0.005f));
    assert(fabsf(a.iq_target - 0.22f) < 1e-6f);
    assert(a.update_count == 1U);
    assert(a.control_target_rpm == 80.0f);
    assert(Motor_SpeedLoop_StepState(&b, &input, 0.01f));
    assert(b.iq_target == 0.0f && a.iq_target > 0.0f);
    input.inhibited = true;
    assert(!Motor_SpeedLoop_StepState(&a, &input, 0.01f));
    assert(a.iq_target == 0.0f && a.feedback_valid);
    assert(a.update_count == 1U);
    input.inhibited = false;
    input.command_rpm = input.target_rpm = 0.0f;
    assert(Motor_SpeedLoop_StepState(&a, &input, 0.01f));
    assert(a.iq_target < 0.0f && a.zero_brake_active);
    Motor_SpeedFeedback_SetState(&a, -1.1f, true);
    assert(!Motor_SpeedLoop_StepState(&a, &input, 0.01f));
    assert(a.iq_target == 0.0f && a.zero_brake_released);
    input.command_rpm = input.target_rpm = 100.0f;
    input.loop_hz = 0.0f;
    assert(Motor_SpeedLoop_StepState(&a, &input, 0.1f));
    assert(input.loop_hz == 0.0f && a.iq_target == 0.0f);
    Motor_SpeedFeedback_SetState(&a, 0.0f, false);
    assert(!Motor_SpeedLoop_StepState(&a, &input, 0.01f));
    puts("Speed standalone core tests passed");
    return 0;
}
