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

    {
        motor_fault_trace_sample_t input = {0}, output;
        unsigned index;
        Motor_Timing_FaultTraceArm();
        assert(Motor_Timing_TraceIsFault());
        assert(g_motor_fault_debug.magic == MOTOR_FAULT_DEBUG_MAGIC);
        assert(g_motor_fault_debug.version == MOTOR_FAULT_DEBUG_VERSION);
        assert(g_motor_fault_debug.descriptor_bytes == 56U && g_motor_fault_debug.sample_bytes == 84U);
        assert(g_motor_fault_debug.state == MOTOR_FAULT_DEBUG_ARMED);
        assert((g_motor_fault_debug.sequence & 1U) == 0U);
        assert(!Motor_Timing_TraceIsCompact());
        assert(!Motor_Timing_TraceCompactDue(0U, 0U, 0U, true));
        assert(!Motor_Timing_TraceDue(0U));
        assert(!Motor_Timing_FaultTraceGet(0U, &output));
        for (index = 0U; index < MOTOR_TRACE_CAPACITY + 5U; index++) {
            input.time_us = index;
            input.adc[0] = 2100U; input.ia = 4.0f; input.flux = 0.002f;
            Motor_Timing_FaultTracePush(&input);
        }
        assert(Motor_Timing_TraceCount() == MOTOR_TRACE_CAPACITY);
        Motor_Timing_FaultTraceFreeze(4U);
        assert(!Motor_Timing_FaultTraceIsActive());
        assert(g_motor_fault_debug.state == MOTOR_FAULT_DEBUG_FROZEN);
        assert((g_motor_fault_debug.sequence & 1U) == 0U);
        assert(g_motor_fault_debug.head == 5U && g_motor_fault_debug.count == MOTOR_TRACE_CAPACITY);
        assert(g_motor_fault_debug.fault == 4U && g_motor_fault_debug.first_us == 5U);
        assert(g_motor_fault_debug.last_us == MOTOR_TRACE_CAPACITY + 4U);
        assert(Motor_Timing_FaultTraceGet(0U, &output) && output.time_us == 5U);
        assert(Motor_Timing_FaultTraceGet(MOTOR_TRACE_CAPACITY - 1U, &output));
        assert(output.time_us == MOTOR_TRACE_CAPACITY + 4U && output.flags == (4U << 16));
        assert(output.adc[0] == 2100U && output.ia == 4.0f && output.flux == 0.002f);
        Motor_Timing_FaultTracePush(&input);
        Motor_Timing_FaultTraceFreeze(5U);
        assert(Motor_Timing_FaultTraceGet(MOTOR_TRACE_CAPACITY - 1U, &output));
        assert(output.flags == (4U << 16));
        assert(!Motor_Timing_FaultTraceGet(MOTOR_TRACE_CAPACITY, &output));
        assert(!Motor_Timing_FaultTraceGet(0U, NULL));
        Motor_Timing_FaultTraceArm();
        assert(Motor_Timing_TraceCount() == 0U);
        Motor_Timing_FaultTracePush(&input);
        Motor_Timing_FaultTraceFreeze(0U);
        assert(Motor_Timing_FaultTraceGet(0U, &output) && output.flags == 0U);
        Motor_Timing_TraceArm();
        assert(!Motor_Timing_TraceIsFault());
        assert(g_motor_fault_debug.state == MOTOR_FAULT_DEBUG_INVALID);
        assert(!Motor_Timing_FaultTraceGet(0U, &output));
    }
    {
        motor_fault_trace_sample_t sample = {0}, output;
        unsigned index;
        Motor_Timing_FaultTraceArm();
        assert(!Motor_Timing_FaultTraceDue(0U, false, false));
        assert(Motor_Timing_FaultTraceIsActive());
        for (index = 1U; index <= MOTOR_FAULT_TRACE_STRIDE; index++) {
            bool due = Motor_Timing_FaultTraceDue(index * 66U, true, false);
            assert(due == (index == MOTOR_FAULT_TRACE_STRIDE));
            if (due) {
                sample.time_us = index * 66U;
                Motor_Timing_FaultTracePush(&sample);
            }
        }
        assert(!Motor_Timing_FaultTraceDue(sample.time_us, true, true));
        assert(Motor_Timing_TraceCount() == 1U);
        assert(Motor_Timing_FaultTraceDue(sample.time_us + 66U, true, true));
        sample.time_us += 66U;
        Motor_Timing_FaultTracePush(&sample);
        Motor_Timing_FaultTraceFreeze(4U);
        assert(Motor_Timing_TraceCount() == 2U);
        assert(Motor_Timing_FaultTraceGet(1U, &output));
        assert(output.time_us == sample.time_us && output.flags == (4U << 16));
        assert(!Motor_Timing_FaultTraceDue(sample.time_us + 66U, true, true));
        Motor_Timing_FaultTraceArm();
        assert(Motor_Timing_FaultTraceDue(0xffffffffU, true, true));
        sample.time_us = 0xffffffffU; sample.flags = 0U;
        Motor_Timing_FaultTracePush(&sample);
        assert(Motor_Timing_FaultTraceDue(0U, true, true));
        sample.time_us = 0U;
        Motor_Timing_FaultTracePush(&sample);
        assert(!Motor_Timing_FaultTraceDue(66U, false, false));
        assert(!Motor_Timing_FaultTraceIsActive());
        assert(Motor_Timing_FaultTraceGet(1U, &output) && output.time_us == 0U);
    }
    {
        motor_fault_trace_sample_t sample = {0};
        uint32_t sequence;
        Motor_Timing_FaultTraceArm();
        sequence = g_motor_fault_debug.sequence;
        sample.time_us = 123U;
        Motor_Timing_FaultTracePush(&sample);
        assert(g_motor_fault_debug.sequence == sequence);
        assert(g_motor_fault_debug.count == 0U);
        Motor_Timing_TraceStop();
        assert(g_motor_fault_debug.state == MOTOR_FAULT_DEBUG_FROZEN && g_motor_fault_debug.count == 1U);
        assert(g_motor_fault_debug.first_us == 123U && g_motor_fault_debug.last_us == 123U);
        sequence = g_motor_fault_debug.sequence;
        Motor_Timing_TraceStop();
        Motor_Timing_FaultTraceFreeze(9U);
        assert(g_motor_fault_debug.sequence == sequence && g_motor_fault_debug.fault == 0U);
        Motor_Timing_TraceArmCompact();
        assert(g_motor_fault_debug.state == MOTOR_FAULT_DEBUG_INVALID);
        Motor_Timing_FaultTraceArm();
        assert(g_motor_fault_debug.state == MOTOR_FAULT_DEBUG_ARMED && g_motor_fault_debug.count == 0U);
        Motor_Timing_TraceStop();
        assert(g_motor_fault_debug.state == MOTOR_FAULT_DEBUG_FROZEN && g_motor_fault_debug.count == 0U);
        g_motor_fault_debug.sequence = 0xfffffffeU;
        Motor_Timing_FaultTraceArm();
        assert(g_motor_fault_debug.sequence == 2U);
        Motor_Timing_FaultTracePush(&sample);
        Motor_Timing_FaultTraceFreeze(0x10004U);
        assert(g_motor_fault_debug.fault == 4U);
    }
    puts("test_motor_timing: pass");
    return 0;
}
