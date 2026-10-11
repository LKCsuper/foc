#include <assert.h>
#include <stdio.h>
#include "motor_timing.h"

MOTOR_PARAM_S motor;
static uint32_t now_us;
uint32_t Hal_Motor_TimeUs(void) { return now_us; }
float Hal_Motor_AdcSampleRateHz(void) { return 30000.0f; }

int main(void)
{
    float average[MOTOR_TIMING_STAGE_COUNT];
    uint32_t samples;
    unsigned i, n;
    {
        motor_callback_t cb;
        uint32_t t = UINT32_MAX - 20U;
        Motor_Timing_CadenceReset();
        Motor_Timing_CallbackBegin(t);
        Motor_Timing_CadenceNote(t, false);
        Motor_Timing_CallbackEnd(t + 70U, true);
        Motor_Timing_CallbackBegin(t + 72U);
        Motor_Timing_CallbackEnd(t + 76U, false);
        Motor_Timing_CallbackBegin(t + 100U);
        Motor_Timing_CadenceNote(t + 100U, true);
        Motor_Timing_CallbackEnd(t + 160U, true);
        Motor_Timing_CallbackGet(&cb);
        assert(cb.count == 3U && cb.full == 2U && cb.full_sum == 130U && cb.full_max == 70U);
        assert(cb.skip_sum == 4U && cb.skip_max == 4U && cb.pending == 2U);
        assert(cb.over33 == 2U && cb.over66 == 1U);
        assert(cb.interval_count == 2U && cb.interval_sum == 100U && cb.interval_max == 72U);
        assert(cb.bins[3] == 1U && cb.bins[1] == 1U);
        Motor_Timing_CadenceStop();
        Motor_Timing_CallbackBegin(t + 200U);
        Motor_Timing_CadenceNote(t + 200U, false);
        Motor_Timing_CallbackEnd(t + 270U, true);
        Motor_Timing_CallbackGet(&cb);
        assert(cb.count == 3U && cb.pending == 2U);
        Motor_Timing_CallbackGet(NULL);
    }
    {
        motor_cadence_t c;
        uint32_t t = UINT32_MAX - 20U;
        const uint32_t steps[] = {66U, 67U, 100U, 133U, 250U, 1001U, 66U};
        Motor_Timing_CadenceReset();
        Motor_Timing_CadenceNote(t, false);
        for (i = 0; i < 7; i++) {
            t += steps[i];
            Motor_Timing_CadenceNote(t, (i & 1U) != 0U);
        }
        Motor_Timing_CadenceGet(&c);
        assert(c.count == 6U && c.sum_us == 682U && c.min_us == 66U && c.max_us == 250U);
        assert(c.gaps == 1U && c.up == 2U && c.down == 4U);
        assert(c.bins[2] == 3U && c.bins[4] == 1U && c.bins[5] == 1U && c.bins[6] == 1U);
        Motor_Timing_CadenceGet(NULL);
        Motor_Timing_CadenceReset();
        Motor_Timing_CadenceGet(&c);
        assert(c.count == 0U && c.gaps == 0U);
    }
    for (i = 0; i < 255; ++i) assert(!Motor_Timing_ProfileBegin());
    assert(Motor_Timing_ProfileBegin());
    now_us = 7U;
    Motor_Timing_ProfileRecord(MOTOR_TIMING_CONTROL, UINT32_MAX - 2U);
    Motor_Timing_ProfileEnd();
    Motor_Timing_ProfileRecord(MOTOR_TIMING_CONTROL, 0U);
    Motor_Timing_ProfileGet(average, &samples);
    assert(samples == 1U && average[MOTOR_TIMING_CONTROL] == 10.0f);
    assert(average[MOTOR_TIMING_OBSERVER] == 0.0f);
    for (n = 0; n < 256; ++n) {
        for (i = 0; i < 255; ++i) assert(!Motor_Timing_ProfileBegin());
        assert(Motor_Timing_ProfileBegin());
        now_us = 20U;
        Motor_Timing_ProfileRecord(MOTOR_TIMING_CONTROL, 0U);
        Motor_Timing_ProfileEnd();
    }
    Motor_Timing_ProfileGet(average, &samples);
    assert(samples == 1U && average[MOTOR_TIMING_CONTROL] == 20.0f);
    {
        motor_trace_sample_t t = {0}, out;
        uint32_t entry = UINT32_MAX - 20U;
        Motor_Timing_CadenceReset();
        Motor_Timing_TraceArm();
        Motor_Timing_CallbackBegin(entry);
        Motor_Timing_CadenceNote(entry, false);
        Motor_Timing_CallbackEnd(entry + 70U, true);
        Motor_Timing_CallbackBegin(entry + 100U);
        Motor_Timing_CadenceNote(entry + 100U, true);
        Motor_Timing_TracePush(&t);
        /* 当前回调在Push后结束，必须归入下一窗口。 */
        Motor_Timing_CallbackEnd(entry + 180U, true);
        Motor_Timing_CallbackBegin(entry + 200U);
        Motor_Timing_CadenceNote(entry + 200U, false);
        Motor_Timing_TracePush(&t);
        Motor_Timing_TraceStop();
        assert(Motor_Timing_TraceGet(0U, &out));
        assert(out.gap_max_us == 100U && out.full_max_us == 70U);
        assert(out.pending_count == 1U && out.up_count == 1U);
        assert(Motor_Timing_TraceGet(1U, &out));
        assert(out.gap_max_us == 100U && out.full_max_us == 80U);
        assert(out.pending_count == 1U && out.up_count == 0U);
        Motor_Timing_TraceArm();
        assert(!Motor_Timing_TraceGet(0U, &out));
        for (n = 0; n < MOTOR_TRACE_CAPACITY + 10U; ++n) {
            for (i = 0; i < 29U; ++i) assert(!Motor_Timing_TraceDue(2U));
            assert(Motor_Timing_TraceDue(2U));
            t.time_us = n;
            Motor_Timing_TracePush(&t);
        }
        assert(Motor_Timing_TraceDue(0U));
        t.time_us++;
        Motor_Timing_TracePush(&t);
        for (n = 0; n < 15U; ++n) {
            for (i = 0; i < 29U; ++i) assert(!Motor_Timing_TraceDue(0U));
            assert(Motor_Timing_TraceDue(0U));
            t.time_us++;
            Motor_Timing_TracePush(&t);
        }
        assert(!Motor_Timing_TraceDue(2U));
        assert(Motor_Timing_TraceCount() == MOTOR_TRACE_CAPACITY);
        assert(Motor_Timing_TraceGet(0U, &out) && out.time_us == 26U);
        assert(Motor_Timing_TraceGet(MOTOR_TRACE_CAPACITY - 1U, &out));
        assert(out.time_us == t.time_us && (out.flags & 128U));
    }
    {
        uint32_t count, maximum;
        Motor_Timing_CadenceReset();
        while (!Motor_Timing_ProfileBegin()) { }
        now_us = 9U;
        Motor_Timing_ProfileRecord(MOTOR_TIMING_TRANSPORT, 2U);
        Motor_Timing_ProfileEnd();
        Motor_Timing_ProfileDetail(MOTOR_TIMING_TRANSPORT, &count, &maximum);
        assert(count == 1U && maximum == 7U);
        Motor_Timing_ProfileGet(average, NULL);
        assert(average[MOTOR_TIMING_TRANSPORT] == 7.0f);
        Motor_Timing_CadenceReset();
        Motor_Timing_ProfileDetail(MOTOR_TIMING_TRANSPORT, &count, &maximum);
        assert(count == 0U && maximum == 0U);
    }
    puts("test_motor_timing_profile: pass");
    {
        motor_trace_sample_t t = {0}, out;
        Motor_Timing_TraceArmEntry();
        for (n = 0; n < MOTOR_TRACE_CAPACITY; n++) {
            for (i = 0; i < 29U; i++) assert(!Motor_Timing_TraceDue(1U));
            assert(Motor_Timing_TraceDue(1U));
            t.time_us = n;
            Motor_Timing_TracePush(&t);
        }
        /* 首次CLOSED立即记录，即使尚未到30拍；进入标记不能冒充失锁。 */
        assert(Motor_Timing_TraceDue(2U));
        t.time_us++;
        Motor_Timing_TracePush(&t);
        for (n = 1; n < 384U; n++) {
            for (i = 0; i < 29U; i++) assert(!Motor_Timing_TraceDue(2U));
            assert(Motor_Timing_TraceDue(2U));
            t.time_us++;
            Motor_Timing_TracePush(&t);
        }
        assert(!Motor_Timing_TraceDue(2U));
        assert(Motor_Timing_TraceGet(127U, &out) && !(out.flags & 256U));
        assert(Motor_Timing_TraceGet(128U, &out) && (out.flags & 256U) && !(out.flags & 128U));
        assert(Motor_Timing_TraceGet(511U, &out) && out.time_us == t.time_us);
        t.time_us++;
        Motor_Timing_TracePush(&t);
        assert(Motor_Timing_TraceGet(511U, &out) && out.time_us + 1U == t.time_us);
        Motor_Timing_TraceArmEntry();
        assert(Motor_Timing_TraceDue(2U));
        Motor_Timing_TracePush(&t);
        assert(Motor_Timing_TraceDue(0U));
        Motor_Timing_TracePush(&t);
        Motor_Timing_TraceStop();
        assert(Motor_Timing_TraceGet(1U, &out) && (out.flags & 384U) == 384U);
    }
    return 0;
}
