#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "motor_fast_vf.h"

static bool s_should_run;
static uint32_t s_time_us;
static float s_control_hz;
static unsigned s_sample_calls;
static unsigned s_step_calls;
static unsigned s_runtime_calls;
static uint32_t s_runtime_start;

uint32_t Hal_Motor_TimeUs(void) { return s_time_us; }
bool Motor_FocIsr_ShouldRun(void) { return s_should_run; }
void Motor_Sense_Update(MOTOR_PARAM_S *motor)
{
    s_sample_calls++;
    motor->v_bus = s_sample_calls == 1U ? 0.5f : 24.0f;
}
float Motor_DmaInt_GetControlFrequencyHz(void) { return s_control_hz; }
void Motor_FastVf_Step(MOTOR_PARAM_S *motor, float control_hz)
{
    (void)motor;
    assert(control_hz == s_control_hz);
    s_step_calls++;
}
void Motor_Timing_RecordRuntime(uint32_t start_time_us)
{
    s_runtime_calls++;
    s_runtime_start = start_time_us;
}

int main(void)
{
    MOTOR_PARAM_S motor;

    memset(&motor, 0, sizeof(motor));
    Motor_FastVf_RunIsr(NULL);
    assert(s_runtime_calls == 0U);

    s_time_us = 77U;
    s_should_run = false;
    Motor_FastVf_RunIsr(&motor);
    assert(s_sample_calls == 0U && s_step_calls == 0U);
    assert(s_runtime_calls == 1U && s_runtime_start == 77U);

    s_should_run = true;
    s_control_hz = 15000.0f;
    Motor_FastVf_RunIsr(&motor);
    assert(s_sample_calls == 1U && s_step_calls == 0U);
    assert(s_runtime_calls == 2U);

    Motor_FastVf_RunIsr(&motor);
    assert(s_sample_calls == 2U && s_step_calls == 1U);
    assert(s_runtime_calls == 3U);
    puts("test_motor_fast_vf_isr: pass");
    return 0;
}
