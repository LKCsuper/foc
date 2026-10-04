#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_timing.h"

MOTOR_PARAM_S motor;
MOTOR_PARAM_S *gpstMotor = &motor;

static float s_adc_hz;
static uint32_t s_time_us;

float Hal_Motor_AdcSampleRateHz(void) { return s_adc_hz; }
uint32_t Hal_Motor_TimeUs(void) { return s_time_us; }

static int Near(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

int main(void)
{
    MOTOR_PARAM_S local;

    memset(&motor, 0, sizeof(motor));
    memset(&local, 0, sizeof(local));

    s_adc_hz = 30000.0f;
    assert(Near(Motor_Timing_GetAdcDt(&local), 1.0f / 30000.0f, 1.0e-9f));
    assert(Near(Motor_Timing_GetControlDt(&local), 2.0f / 30000.0f, 1.0e-9f));
    assert(Near(local.ol_foc_freq, 30000.0f, 0.1f));

    s_adc_hz = 0.0f;
    local.ol_foc_freq = 15000.0f;
    assert(Near(Motor_Timing_GetAdcDt(&local), 1.0f / 15000.0f, 1.0e-9f));
    local.ol_foc_freq = 0.0f;
    assert(Near(Motor_Timing_GetAdcDt(&local), 1.0f, 1.0e-6f));

    motor.ol_foc_freq = 12345.0f;
    assert(Near(Motor_DmaInt_GetAdcFrequencyHz(), 12345.0f, 0.1f));
    assert(Near(Motor_DmaInt_GetControlFrequencyHz(), 6172.5f, 0.1f));

    s_time_us = 125U;
    Motor_Timing_RecordRuntime(100U);
    assert(Near(Motor_DmaInt_GetAverageRuntimeUs(), 25.0f, 0.01f));
    assert(Near(Motor_DmaInt_GetMaximumRuntimeUs(), 25.0f, 0.01f));
    s_time_us = 275U;
    Motor_Timing_RecordRuntime(200U);
    assert(Near(Motor_DmaInt_GetAverageRuntimeUs(), 26.0f, 0.01f));
    assert(Near(Motor_DmaInt_GetMaximumRuntimeUs(), 75.0f, 0.01f));
    s_time_us = 2200U;
    Motor_Timing_RecordRuntime(1000U);
    assert(Near(Motor_DmaInt_GetAverageRuntimeUs(), 26.0f, 0.01f));
    assert(Near(Motor_DmaInt_GetMaximumRuntimeUs(), 75.0f, 0.01f));

    {
        uint32_t up = 0U;
        uint32_t down = 0U;

        Motor_Timing_RecordSampleDirection(true);
        Motor_Timing_RecordSampleDirection(false);
        Motor_Timing_RecordSampleDirection(false);
        Motor_DmaInt_GetSampleDirectionCounts(&up, &down);
        assert(up == 1U && down == 2U);
        Motor_DmaInt_GetSampleDirectionCounts(NULL, NULL);
    }

    puts("test_motor_timing: pass");
    return 0;
}
