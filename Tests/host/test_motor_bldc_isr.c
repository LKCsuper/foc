#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bldc_isr.h"

static bool s_fault;
static bool s_sample_ready;
static unsigned s_sense_calls;
static unsigned s_stop_calls;
static unsigned s_step_calls;
static unsigned s_record_calls;
static bool s_last_sample_upward;
static float s_phase[3];
static float s_bus;
static float s_dt;

uint32_t Hal_Motor_TimeUs(void) { return 100U; }
void Motor_Sense_Update(MOTOR_PARAM_S *motor)
{
    (void)motor;
    s_sense_calls++;
}
float Motor_Timing_GetAdcDt(MOTOR_PARAM_S *motor)
{
    (void)motor;
    return 0.001f;
}
void Motor_Timing_RecordRuntime(uint32_t start_time)
{
    assert(start_time == 100U);
    s_record_calls++;
}
void Motor_BldcSensorless_Stop(motor_bldc_sensorless_t *control)
{
    (void)control;
    s_stop_calls++;
}
bool Motor_Protect_Run(const MOTOR_PARAM_S *motor, float dt)
{
    (void)motor;
    assert(dt == 0.001f);
    return s_fault;
}
bool Hal_Motor_BldcBemfSampleReady(bool upward)
{
    s_last_sample_upward = upward;
    return s_sample_ready;
}
bool Motor_BldcSensorless_Step(motor_bldc_sensorless_t *control,
                               const float phase[3], float bus, float dt)
{
    (void)control;
    s_step_calls++;
    memcpy(s_phase, phase, sizeof(s_phase));
    s_bus = bus;
    s_dt = dt;
    return true;
}

int main(void)
{
    MOTOR_PARAM_S motor;
    motor_bldc_sensorless_t sensorless;

    memset(&motor, 0, sizeof(motor));
    memset(&sensorless, 0, sizeof(sensorless));
    motor.va = 1.0f;
    motor.vb = 2.0f;
    motor.vc = 3.0f;
    motor.v_bus = 24.0f;

    s_fault = true;
    Motor_BldcIsr_Step(&sensorless, &motor, true);
    assert(s_sense_calls == 1U && s_stop_calls == 1U && s_step_calls == 0U);
    assert(s_record_calls == 1U);

    s_fault = false;
    s_sample_ready = false;
    Motor_BldcIsr_Step(&sensorless, &motor, false);
    assert(!s_last_sample_upward && s_step_calls == 0U && s_record_calls == 2U);

    s_sample_ready = true;
    Motor_BldcIsr_Step(&sensorless, &motor, true);
    assert(s_last_sample_upward && s_step_calls == 1U && s_record_calls == 3U);
    assert(s_phase[0] == 1.0f && s_phase[1] == 3.0f && s_phase[2] == 2.0f);
    assert(s_bus == 24.0f && s_dt == 0.002f);

    puts("test_motor_bldc_isr: pass");
    return 0;
}
