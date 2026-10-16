#include "test_common_assert.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
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
        motor_pair_frame_t frame = {0};
        motor_pair_sample_t out;
        Motor_Timing_PairHalfArm();
        assert(Motor_Timing_PairHalfMode());
        for (i = 0U; i < 40000U; i++) {
            now_us = 1000U + i * 33U;
            frame.entry_us = now_us;
            frame.callback_cr1 = i % 2U ? 16U : 0U;
            frame.reconstructed = 4U;
            if (i == 14401U) Motor_Timing_HandoffMark();
            if (Motor_Timing_PairFrame(&frame)) Motor_Timing_PairPush(0.0f, 0.0f);
        }
        assert(Motor_Timing_HandoffCount() == 32U && !Motor_Timing_PairActive());
        assert(Motor_Timing_PairGet(4U, &out) && out.flags == 1U);
        assert(out.current.entry_us == 1000U + 14401U * 33U);
        assert(out.current.entry_us - out.previous.entry_us == 33U);
        assert(out.current.callback_cr1 != out.previous.callback_cr1 && out.current.reconstructed == 4U);
        assert(Motor_Timing_PairGet(5U, &out) && out.current.callback_cr1 == 0U);
        assert(Motor_Timing_PairGet(6U, &out) && out.current.callback_cr1 == 16U);
        Motor_Timing_PairArm();
        assert(!Motor_Timing_PairHalfMode());
        Motor_Timing_HandoffStop();
    }
    {
        motor_pair_frame_t frame = {0};
        motor_pair_sample_t out;
        motor_phase_sample_t phase;
        Motor_Timing_PairArm();
        assert(sizeof(out) <= sizeof(motor_handoff_sample_t));
        for (i = 0U; i < 20000U; i++) {
            now_us = 1000U + i * 67U;
            frame.pwm_sequence = i; frame.entry_us = now_us;
            frame.adc[0] = (uint16_t)(i % 4096U);
            frame.ia = (float)i;
            if (i == 7201U) Motor_Timing_HandoffMark();
            assert(!Motor_Timing_HandoffDue() && !Motor_Timing_PhaseDue(false));
            if (Motor_Timing_PairFrame(&frame)) Motor_Timing_PairPush(1.0f, 2.0f);
        }
        assert(!Motor_Timing_PairActive() && Motor_Timing_HandoffCount() == 32U);
        assert(Motor_Timing_PairGet(4U, &out) && out.flags == 1U);
        assert(out.current.pwm_sequence == 7201U && out.previous.pwm_sequence == 7200U);
        assert(out.current.sample_sequence == out.previous.sample_sequence + 1U);
        assert(out.current.adc[0] == 7201U % 4096U && out.previous.ia == 7200.0f);
        assert(out.v_alpha == 1.0f && out.v_beta == 2.0f);
        assert(!Motor_Timing_PhaseGet(0U, &phase));
        assert(!Motor_Timing_PairGet(32U, &out) && !Motor_Timing_PairGet(0U, NULL));
        Motor_Timing_PhaseArm();
        assert(!Motor_Timing_PairMode() && !Motor_Timing_PairFrame(&frame));
        Motor_Timing_HandoffStop();
    }
    {
        motor_observer_step_sample_t step = {0}, out;
        motor_phase_sample_t phase;
        Motor_Timing_ObserverStepArm();
        assert(sizeof(step) <= sizeof(motor_handoff_sample_t));
        for (i = 0U; i < 20000U; i++) {
            now_us = 1000U + i * 67U;
            assert(!Motor_Timing_PhaseDue(false) && !Motor_Timing_HandoffDue());
            if (i == 7201U) Motor_Timing_HandoffMark();
            if (!Motor_Timing_ObserverStepDue()) continue;
            step.gain = (float)i;
            Motor_Timing_ObserverStepPush(&step);
        }
        assert(!Motor_Timing_ObserverStepActive());
        assert(Motor_Timing_HandoffCount() == 32U);
        assert(Motor_Timing_ObserverStepGet(4U, &out) && out.flags == 1U && out.gain == 7201.0f);
        assert(!Motor_Timing_PhaseGet(0U, &phase));
        assert(!Motor_Timing_ObserverStepGet(32U, &out));
        assert(!Motor_Timing_ObserverStepGet(0U, NULL));
        Motor_Timing_PhaseArm();
        assert(!Motor_Timing_ObserverStepMode() && !Motor_Timing_ObserverStepDue());
        Motor_Timing_HandoffStop();
    }
    {
        motor_phase_sample_t phase = {0}, out;
        unsigned captured = 0U;
        Motor_Timing_PhaseArmLong();
        for (i = 0U; i < 20000U; i++) {
            now_us = 0xffff0000U + i * 67U;
            if (!Motor_Timing_PhaseDue(i == 7201U)) continue;
            captured++;
            phase.observer = (float)i;
            Motor_Timing_PhasePush(&phase);
        }
        assert(captured == 44U && Motor_Timing_HandoffCount() == 32U);
        assert(Motor_Timing_PhaseGet(4U, &out) && out.flags == 1U && out.observer == 7201.0f);
        assert(out.time_us == (uint32_t)(0xffff0000U + 7201U * 67U));
        assert(Motor_Timing_PhaseGet(31U, &out) && out.flags == 2U && out.observer == 19351.0f);
        assert(!Motor_Timing_PhaseDue(true));
        Motor_Timing_PhaseArmLong();
        Motor_Timing_HandoffStop();
        assert(Motor_Timing_HandoffCount() == 0U && !Motor_Timing_PhaseDue(true));
    }
    {
        motor_phase_sample_t phase = {0}, out;
        motor_handoff_sample_t handoff;
        Motor_Timing_PhaseArm();
        assert(Motor_Timing_PhaseMode());
        assert(sizeof(phase) <= sizeof(handoff));
        for (i = 0U; i < 700U; i++) {
            now_us = 1000U + i * 67U;
            assert(!Motor_Timing_HandoffDue());
            if (!Motor_Timing_PhaseDue(i == 400U)) continue;
            phase.observer = (float)i;
            Motor_Timing_PhasePush(&phase);
        }
        assert(Motor_Timing_HandoffCount() == 32U);
        assert(!Motor_Timing_HandoffGet(0U, &handoff));
        assert(Motor_Timing_PhaseGet(16U, &out));
        assert(out.flags == 1U && out.time_us == 27800U && out.observer == 400.0f);
        assert(Motor_Timing_PhaseGet(31U, &out) && out.flags == 2U);
        assert(!Motor_Timing_PhaseGet(32U, &out));
        assert(!Motor_Timing_PhaseGet(0U, NULL));
        assert(!Motor_Timing_PhaseDue(true));
        Motor_Timing_HandoffArm();
        assert(!Motor_Timing_PhaseMode() && !Motor_Timing_PhaseDue(true));
        assert(!Motor_Timing_PhaseGet(0U, &out));
        Motor_Timing_HandoffStop();
    }
    {
        MOTOR_PARAM_S handoff_motor = {0};
        motor_handoff_sample_t sample;
        unsigned step;
        assert(!Motor_Timing_HandoffDue());
        Motor_Timing_HandoffArm();
        for (step = 0U; step < 600U; step++) {
            now_us = 1000U + step * 67U;
            if (step == 400U) Motor_Timing_HandoffMark();
            if (!Motor_Timing_HandoffDue()) continue;
            handoff_motor.ol_vd_int = 1.0f;
            handoff_motor.currentAdc[0] = 2048U;
            Motor_Timing_HandoffBefore(&handoff_motor, 0.1f, 0.02f, 0.0f, 0.5f, 0.000067f);
            handoff_motor.ol_vd_int = 2.0f;
            Motor_Timing_HandoffTransport(&handoff_motor);
            handoff_motor.ol_vd_int = 3.0f;
            Motor_Timing_HandoffAfter(&handoff_motor, 0.4f, 0.5f);
        }
        assert(!Motor_Timing_HandoffGet(0U, &sample));
        for (; step < 700U; step++) {
            now_us = 1000U + step * 67U;
            if (!Motor_Timing_HandoffDue()) continue;
            Motor_Timing_HandoffBefore(&handoff_motor, 0.1f, 0.0f, 0.0f, 0.5f, 0.000067f);
            Motor_Timing_HandoffTransport(&handoff_motor);
            Motor_Timing_HandoffAfter(&handoff_motor, 0.4f, 0.5f);
        }
        assert(Motor_Timing_HandoffCount() == 32U);
        assert(Motor_Timing_HandoffGet(16U, &sample) && sample.flags == 1U);
        assert(sample.time_us == 27800U && sample.delta == 0.02f);
        assert(sample.vd_before == 1.0f && sample.vd_transport == 2.0f && sample.vd_after == 3.0f);
        assert(sample.adc[0] == 2048U);
        assert(Motor_Timing_HandoffGet(31U, &sample) && sample.flags == 2U);
        assert(!Motor_Timing_HandoffGet(32U, &sample));
        assert(!Motor_Timing_HandoffGet(0U, NULL));
        Motor_Timing_HandoffArm();
        Motor_Timing_HandoffStop();
        assert(Motor_Timing_HandoffCount() == 0U && !Motor_Timing_HandoffDue());
    }
    {
        motor_profile_snapshot_t snapshot;
        Motor_Timing_CadenceReset();
        while (!Motor_Timing_ProfileBegin()) { }
        now_us = 110U;
        Motor_Timing_ProfileRecord(MOTOR_TIMING_SENSE_FRAME, 100U);
        Motor_Timing_ProfileRecord(MOTOR_TIMING_SENSOR_FEEDBACK, 105U);
        Motor_Timing_ProfileRecord(MOTOR_TIMING_PROTECTION, 108U);
        Motor_Timing_CadenceStop();
        Motor_Timing_ProfileSnapshot(&snapshot);
        assert(snapshot.count[MOTOR_TIMING_SENSE_FRAME] == 1U);
        assert(snapshot.sum_us[MOTOR_TIMING_SENSE_FRAME] == 10U);
        assert(snapshot.max_us[MOTOR_TIMING_SENSOR_FEEDBACK] == 5U);
        assert(snapshot.sum_us[MOTOR_TIMING_PROTECTION] == 2U);
        now_us = 210U;
        Motor_Timing_ProfileRecord(MOTOR_TIMING_PROTECTION, 100U);
        for (i = 0U; i < 256U; i++) assert(!Motor_Timing_ProfileBegin());
        Motor_Timing_ProfileSnapshot(&snapshot);
        assert(snapshot.count[MOTOR_TIMING_PROTECTION] == 1U);
        assert(snapshot.max_us[MOTOR_TIMING_PROTECTION] == 2U);
        EXPECT_COMMON_ASSERT(Motor_Timing_ProfileSnapshot(NULL));
        Motor_Timing_CadenceReset();
        Motor_Timing_ProfileSnapshot(&snapshot);
        assert(snapshot.count[MOTOR_TIMING_PROTECTION] == 0U);
    }
    {
        motor_speed_task_diag_t task;
        Motor_Timing_WakeProbeReset();
        Motor_Timing_WakeProbeArm(UINT32_MAX);
        assert(Motor_Timing_WakeProbeArmed() && !Motor_Timing_WakeProbeDmaDue());
        Motor_Timing_WakeProbeTick(UINT32_MAX - 1U);
        assert(!Motor_Timing_WakeProbeDmaDue());
        now_us = UINT32_MAX - 99U;
        Motor_Timing_WakeProbeTick(UINT32_MAX);
        assert(Motor_Timing_WakeProbeDmaDue());
        Motor_Timing_WakeProbeDmaNote(UINT32_MAX - 49U, 10U);
        now_us = 100U;
        Motor_Timing_SpeedTaskWake(100U, UINT32_MAX, UINT32_MAX);
        assert(!Motor_Timing_WakeProbeDmaDue());
        Motor_Timing_SpeedPiNote(100U, true);
        Motor_Timing_WakeProbeArm(0U);
        now_us = 1000U;
        Motor_Timing_WakeProbeTick(0U);
        Motor_Timing_WakeProbeDmaNote(1010U, 1040U);
        now_us = 1100U;
        Motor_Timing_SpeedTaskWake(1100U, 0U, 0U);
        Motor_Timing_SpeedPiNote(1200U, true);
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.worst_wake.valid && task.worst_wake.delay_us == 200U);
        assert(task.worst_wake.dma_count == 1U && task.worst_wake.dma_us == 60U);
        assert(task.pi_max_gap_us == 1100U && task.worst_pi_wake.valid);
        assert(task.worst_pi_wake.delay_us == 100U && task.worst_pi_wake.dma_us == 30U);
        Motor_Timing_CadenceStop();
        Motor_Timing_WakeProbeArm(1U);
        assert(!Motor_Timing_WakeProbeArmed());
        Motor_Timing_SpeedTaskReset();
        Motor_Timing_WakeProbeArm(1U);
        assert(!Motor_Timing_WakeProbeArmed());
    }
    {
        motor_speed_task_diag_t task;
        const uint32_t gaps[] = {1000U, 1001U, 1100U, 1101U, 1250U, 1251U, 1500U, 1501U, 2000U, 2001U, 999U};
        uint32_t stamp = UINT32_MAX - 100U;
        Motor_Timing_SpeedTaskReset();
        Motor_Timing_SpeedPiNote(stamp, true);
        for (i = 0U; i < sizeof(gaps) / sizeof(gaps[0]); i++) {
            stamp += gaps[i];
            Motor_Timing_SpeedPiNote(stamp, true);
        }
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.pi_count == 12U && task.pi_over_1ms == 9U);
        assert(task.pi_over_streak == 0U && task.pi_over_streak_max == 9U);
        assert(task.pi_gap_bins[0] == 2U && task.pi_gap_bins[1] == 2U);
        assert(task.pi_gap_bins[2] == 2U && task.pi_gap_bins[3] == 2U);
        assert(task.pi_gap_bins[4] == 2U && task.pi_gap_bins[5] == 1U);
        Motor_Timing_CadenceStop();
        Motor_Timing_SpeedPiNote(stamp + 3000U, true);
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.pi_gap_bins[5] == 1U);
        Motor_Timing_SpeedTaskReset();
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.pi_over_1ms == 0U && task.pi_over_streak_max == 0U && task.pi_gap_bins[5] == 0U);
    }
    {
        motor_speed_task_diag_t task;
        motor_irq_diag_t irq;
        motor_cadence_t cadence;
        Motor_Timing_SpeedTaskReset();
        assert(Motor_Timing_SpeedTaskIsActive());
        assert(!Motor_Timing_CallbackIsActive());
        assert(!Motor_Timing_IrqDue(0U) && !Motor_Timing_IrqDue(1U));
        Motor_Timing_IrqNote(0U, 100U, 200U, 3U);
        Motor_Timing_IrqGet(0U, &irq);
        assert(irq.count == 0U && irq.seen == 0U);
        assert(!Motor_Timing_ProfileBegin());
        Motor_Timing_CadenceNote(100U, false);
        Motor_Timing_CadenceNote(167U, false);
        Motor_Timing_CadenceGet(&cadence);
        assert(cadence.count == 0U);
        Motor_Timing_SpeedPiNote(1000U, true);
        Motor_Timing_SpeedPiNote(2000U, true);
        Motor_Timing_SpeedTaskNote(2000U, true, 600.0f, 600.0f, 0.1f);
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.pi_count == 2U && task.pi_max_gap_us == 1000U && task.count == 1U);
        Motor_Timing_CadenceStop();
        assert(!Motor_Timing_SpeedTaskIsActive());
        Motor_Timing_CadenceReset();
        assert(Motor_Timing_IrqDue(0U) && Motor_Timing_CallbackIsActive());
    }
    {
        motor_speed_task_diag_t task;
        Motor_Timing_CadenceReset();
        assert(Motor_Timing_SpeedTaskIsActive());
        Motor_Timing_SpeedPiNote(UINT32_MAX - 99U, true);
        Motor_Timing_SpeedPiNote(0U, false);
        Motor_Timing_SpeedPiNote(100U, true);
        Motor_Timing_SpeedPiNote(1100U, true);
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.pi_count == 3U && task.pi_no_update == 1U);
        assert(task.pi_max_gap_us == 1000U && task.pi_sum_gap_us == 1200U);
        Motor_Timing_CadenceStop();
        assert(!Motor_Timing_SpeedTaskIsActive());
        Motor_Timing_SpeedTaskHeartbeat(2100U);
        Motor_Timing_SpeedPiNote(3100U, true);
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.pi_count == 3U);
        Motor_Timing_CadenceReset();
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.pi_count == 0U && task.pi_sum_gap_us == 0U);
    }
    {
        motor_speed_task_diag_t task;
        Motor_Timing_CadenceReset();
        Motor_Timing_SpeedTaskStage(UINT32_MAX - 99U, UINT32_MAX - 2U, 1U);
        Motor_Timing_SpeedTaskWake(100U, 2U, 1U);
        Motor_Timing_SpeedTaskStage(130U, 3U, 3U);
        Motor_Timing_SpeedTaskStage(170U, 4U, 4U);
        Motor_Timing_SpeedTaskStage(220U, 5U, 1U);
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.phase_max_us[0] == 200U && task.phase_max_ticks[0] == 5U);
        assert(task.phase_max_us[1] == 30U && task.phase_max_us[2] == 40U);
        assert(task.phase_max_us[3] == 50U && task.wake_late_max_ticks == 1U);
        Motor_Timing_SpeedTaskWake(240U, 6U, 8U);
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.wake_late_max_ticks == 1U && task.deadline_tick == 8U);
        Motor_Timing_SpeedTaskStage(300U, 9U, 0U);
        Motor_Timing_SpeedTaskStage(300U, 9U, 5U);
        Motor_Timing_CadenceStop();
        Motor_Timing_SpeedTaskWake(999U, 99U, 0U);
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.phase == 2U && task.phase_us == 240U);
        Motor_Timing_CadenceReset();
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.phase_max_us[0] == 0U && task.wake_late_max_ticks == 0U);
    }
    {
        motor_irq_diag_t irq;
        Motor_Timing_CadenceReset();
        for (i = 0U; i < 310U; i++) {
            bool due = Motor_Timing_IrqDue(0U);
            assert(due == (i % 31U == 0U));
            if (due) Motor_Timing_IrqNote(0U, i * 33U, i * 33U + 20U, 1U);
        }
        assert(Motor_Timing_IrqDue(1U));
        Motor_Timing_IrqGet(0U, &irq);
        assert(irq.seen == 310U && irq.count == 10U && irq.sum_us == 200U);
        assert(irq.tick_pending == 10U);
        Motor_Timing_CadenceStop();
        assert(!Motor_Timing_IrqDue(0U) && !Motor_Timing_IrqDue(2U));
        Motor_Timing_IrqGet(0U, &irq);
        assert(irq.seen == 310U);
        Motor_Timing_CadenceReset();
        assert(!Motor_Timing_IrqDue(2U));
        assert(Motor_Timing_IrqDue(0U));
        Motor_Timing_IrqGet(0U, &irq);
        assert(irq.seen == 1U && irq.count == 0U);
    }
    {
        motor_callback_t callback;
        motor_irq_diag_t irq;
        Motor_Timing_TraceStop();
        Motor_Timing_CadenceReset();
        assert(Motor_Timing_CallbackIsActive());
        Motor_Timing_CallbackStop();
        assert(!Motor_Timing_CallbackIsActive());
        Motor_Timing_CallbackBegin(10U);
        Motor_Timing_CallbackEnd(30U, true);
        Motor_Timing_CallbackGet(&callback);
        assert(callback.count == 0U && callback.pending == 0U);
        Motor_Timing_IrqNote(0U, 10U, 30U, 1U);
        Motor_Timing_IrqGet(0U, &irq);
        assert(irq.count == 1U && irq.sum_us == 20U && irq.tick_pending == 1U);
        Motor_Timing_TraceArm();
        assert(Motor_Timing_CallbackIsActive());
        Motor_Timing_TraceStop();
        assert(!Motor_Timing_CallbackIsActive());
        Motor_Timing_CadenceReset();
        assert(Motor_Timing_CallbackIsActive());
    }
    {
        float reference = 100.0f;
        now_us = 100U;
        Motor_Timing_RecordRuntime(0U);
        assert(Motor_DmaInt_GetAverageRuntimeUs() == reference);
        for (i = 0U; i < 10000U; i++) {
            uint32_t elapsed = i % 2U == 0U ? 1000U : 0U;
            now_us = elapsed;
            Motor_Timing_RecordRuntime(0U);
            reference += 0.02f * ((float)elapsed - reference);
            assert(fabsf(Motor_DmaInt_GetAverageRuntimeUs() - reference) < 0.001f);
        }
        assert(Motor_DmaInt_GetMaximumRuntimeUs() == 1000.0f);
        reference = Motor_DmaInt_GetAverageRuntimeUs();
        now_us = 1001U;
        Motor_Timing_RecordRuntime(0U);
        assert(Motor_DmaInt_GetAverageRuntimeUs() == reference);
        assert(Motor_DmaInt_GetMaximumRuntimeUs() == 1000.0f);
        now_us = 49U;
        Motor_Timing_RecordRuntime(UINT32_MAX - 50U);
        assert(Motor_DmaInt_GetAverageRuntimeUs() < reference);
    }
    {
        motor_irq_diag_t irq;
        Motor_Timing_CadenceReset();
        Motor_Timing_IrqNote(0U, UINT32_MAX - 20U, 39U, 3U);
        Motor_Timing_IrqNote(0U, 100U, 150U, 1U);
        Motor_Timing_IrqNote(1U, 0U, 90U, 0U);
        Motor_Timing_IrqGet(0U, &irq);
        assert(irq.count == 2U && irq.sum_us == 110U && irq.max_us == 60U);
        assert(irq.tick_pending == 2U && irq.switch_pending == 1U);
        Motor_Timing_IrqGet(1U, &irq);
        assert(irq.count == 1U && irq.sum_us == 90U);
        Motor_Timing_CadenceStop();
        Motor_Timing_IrqNote(1U, 0U, 999U, 0U);
        Motor_Timing_IrqGet(1U, &irq);
        assert(irq.count == 1U);
        Motor_Timing_IrqNote(2U, 0U, 999U, 3U);
        EXPECT_COMMON_ASSERT(Motor_Timing_IrqGet(0U, NULL));
        Motor_Timing_CadenceReset();
        Motor_Timing_IrqGet(0U, &irq);
        assert(irq.count == 0U);
    }
    {
        motor_speed_task_diag_t task;
        Motor_Timing_CadenceReset();
        Motor_Timing_SpeedTaskStage(UINT32_MAX - 99U, UINT32_MAX - 1U, 4U);
        Motor_Timing_SpeedTaskNote(UINT32_MAX - 99U, false, 600.0f, 500.0f, 0.5f);
        Motor_Timing_SpeedTaskStage(100U, 1U, 4U);
        Motor_Timing_SpeedTaskNote(100U, true, 600.0f, 700.0f, -0.3f);
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.count == 2U && task.max_gap_us == 200U && task.last_us == 100U && task.allowed);
        assert(task.target == 600.0f && task.feedback == 700.0f && task.iq == -0.3f);
        assert(task.max_gap_ticks == 3U && task.tick == 1U && task.phase == 4U);
        Motor_Timing_CadenceStop();
        Motor_Timing_SpeedTaskNote(200U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.count == 2U && task.allowed);
        Motor_Timing_CadenceReset();
        Motor_Timing_SpeedTaskGet(&task);
        assert(task.count == 0U && task.max_gap_us == 0U);
        EXPECT_COMMON_ASSERT(Motor_Timing_SpeedTaskGet(NULL));
    }
    {
        motor_control_clock_t clock = {0};
        float dt, total = 0.0f;
        const float nominal = 1.0f / 15000.0f;
        uint32_t t = UINT32_MAX - 20U;
        assert(Motor_Timing_ControlClockStep(&clock, true, t, nominal, &dt) && dt == nominal);
        /* 模拟每五拍一次 100 us 间隔，积分时间须与入口累计时间一致。 */
        for (i = 0; i < 100U; i++) {
            unsigned step = i % 5U == 4U ? 100U : 67U;
            t += step;
            assert(Motor_Timing_ControlClockStep(&clock, true, t, nominal, &dt));
            assert(fabsf(dt - step * 1.0e-6f) < 1.0e-9f);
            total += dt;
        }
        assert(fabsf(total - 0.00736f) < 1.0e-7f);
        assert(!Motor_Timing_ControlClockStep(&clock, true, t, nominal, &dt) && dt == 0.0f);
        t += 20U;
        assert(!Motor_Timing_ControlClockStep(&clock, true, t, nominal, &dt) && dt == 0.0f);
        t += 201U;
        assert(!Motor_Timing_ControlClockStep(&clock, true, t, nominal, &dt) && dt == 0.0f);
        /* 停机/退出无感清除历史，重新起动不带入长停机间隔。 */
        assert(Motor_Timing_ControlClockStep(&clock, false, t, nominal, &dt) && !clock.seeded);
        t += 1000000U;
        assert(Motor_Timing_ControlClockStep(&clock, true, t, nominal, &dt) && dt == nominal);
        assert(!Motor_Timing_ControlClockStep(&clock, true, t, NAN, &dt) && !clock.seeded);
        assert(!Motor_Timing_ControlClockStep(&clock, true, t, INFINITY, &dt));
        assert(!Motor_Timing_ControlClockStep(&clock, true, t, 0.0f, &dt));
        assert(!Motor_Timing_ControlClockStep(&clock, true, t, -nominal, &dt));
        assert(!Motor_Timing_ControlClockStep(NULL, true, t, nominal, &dt));
        assert(!Motor_Timing_ControlClockStep(&clock, true, t, nominal, NULL));
    }
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
        EXPECT_COMMON_ASSERT(Motor_Timing_CallbackGet(NULL));
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
        EXPECT_COMMON_ASSERT(Motor_Timing_CadenceGet(NULL));
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
    {
        motor_trace_sample_t t = {0}, out;
        Motor_Timing_TraceArmEntryFast();
        for (i = 0; i < 29U; i++) assert(!Motor_Timing_TraceDue(1U));
        assert(Motor_Timing_TraceDue(1U));
        Motor_Timing_TracePush(&t);
        for (n = 0; n < 384U; n++) {
            assert(Motor_Timing_TraceDue(2U));
            t.time_us = n + 1U;
            Motor_Timing_TracePush(&t);
        }
        assert(!Motor_Timing_TraceDue(2U));
        assert(Motor_Timing_TraceCount() == 385U);
        assert(Motor_Timing_TraceGet(0U, &out) && !(out.flags & 256U));
        assert(Motor_Timing_TraceGet(1U, &out) && (out.flags & 256U));
        assert(Motor_Timing_TraceGet(384U, &out) && out.time_us == 384U);
        Motor_Timing_TraceArm();
        for (i = 0; i < 29U; i++) assert(!Motor_Timing_TraceDue(2U));
        assert(Motor_Timing_TraceDue(2U));
        Motor_Timing_TraceStop();
    }
    {
        motor_trace_light_sample_t t = {0}, out;
        motor_trace_sample_t full = {0};
        assert(sizeof(t) == 96U && sizeof(t) < sizeof(full));
        Motor_Timing_TraceArmLight();
        assert(Motor_Timing_TraceIsLight());
        /* 错误格式的Push/Get不能读写共用缓冲区。 */
        Motor_Timing_TracePush(&full);
        assert(Motor_Timing_TraceCount() == 0U);
        assert(!Motor_Timing_TraceGetLight(0U, &out));
        for (n = 0; n < MOTOR_TRACE_CAPACITY; n++) {
            for (i = 0; i < 29U; i++) assert(!Motor_Timing_TraceDue(1U));
            assert(Motor_Timing_TraceDue(1U));
            t.time_us = n;
            t.iq_cmd = 0.5f;
            Motor_Timing_TracePushLight(&t);
        }
        assert(Motor_Timing_TraceDue(2U));
        t.time_us++;
        Motor_Timing_TracePushLight(&t);
        for (n = 1; n < 384U; n++) {
            for (i = 0; i < 7U; i++) assert(!Motor_Timing_TraceDue(2U));
            assert(Motor_Timing_TraceDue(2U));
            t.time_us++;
            Motor_Timing_TracePushLight(&t);
        }
        assert(!Motor_Timing_TraceDue(2U));
        assert(!Motor_Timing_TraceGet(0U, &full));
        assert(Motor_Timing_TraceCount() == MOTOR_TRACE_CAPACITY);
        assert(Motor_Timing_TraceGetLight(127U, &out) && !(out.flags & 256U));
        assert(Motor_Timing_TraceGetLight(128U, &out) && (out.flags & 256U));
        assert(Motor_Timing_TraceGetLight(511U, &out) && out.time_us == t.time_us && out.iq_cmd == 0.5f);
        /* 回退事件立即记录，保留ENTRY/LOSS两种标志，不修改控制状态。 */
        Motor_Timing_TraceArmLight();
        assert(Motor_Timing_TraceDue(2U)); Motor_Timing_TracePushLight(&t);
        assert(Motor_Timing_TraceDue(0U)); Motor_Timing_TracePushLight(&t);
        Motor_Timing_TraceStop();
        assert(Motor_Timing_TraceGetLight(1U, &out) && (out.flags & 384U) == 384U);
        Motor_Timing_TraceArm();
        assert(!Motor_Timing_TraceIsLight() && !Motor_Timing_TraceGetLight(0U, &out));
        Motor_Timing_TraceStop();
    }
    {
        motor_trace_light_sample_t t = {0}, out;
        Motor_Timing_TraceArmFlight();
        assert(Motor_Timing_TraceIsFlight() && Motor_Timing_TraceIsLight());
        for (n = 0U; n < 8U; n++) {
            for (i = 0U; i < 29U; i++) assert(!Motor_Timing_TraceDue(1U));
            assert(Motor_Timing_TraceDue(1U));
            t.time_us = n; Motor_Timing_TracePushLight(&t);
        }
        /* 接管丢弃此前样本，固定192条不会被后续长时间滚动覆盖。 */
        for (n = 0U; n < 1192U; n++) {
            if (n) for (i = 0U; i < (n >= 192U ? 127U : 7U); i++) assert(!Motor_Timing_TraceDue(2U));
            assert(Motor_Timing_TraceDue(2U));
            t.time_us = 100U + n; Motor_Timing_TracePushLight(&t);
        }
        assert(Motor_Timing_TraceCount() == 512U);
        assert(!Motor_Timing_TraceGetLight(0U, &out));
        assert(Motor_Timing_TraceDue(0U));
        t.time_us = 2000U; Motor_Timing_TracePushLight(&t);
        assert(!Motor_Timing_TraceDue(2U));
        assert(Motor_Timing_TraceGetLight(0U, &out) && out.time_us == 100U);
        assert(Motor_Timing_TraceGetLight(191U, &out) && out.time_us == 291U);
        assert(Motor_Timing_TraceGetLight(192U, &out) && out.time_us == 973U);
        assert(Motor_Timing_TraceGetLight(511U, &out) && out.time_us == 2000U && (out.flags & 128U));
        /* 接管早期丢失不等满192条；命令停机也可冻结尚未接管的滚动窗口。 */
        Motor_Timing_TraceArmFlight();
        assert(Motor_Timing_TraceDue(2U)); Motor_Timing_TracePushLight(&t);
        assert(Motor_Timing_TraceDue(1U)); Motor_Timing_TracePushLight(&t);
        assert(Motor_Timing_TraceCount() == 2U && Motor_Timing_TraceGetLight(1U, &out));
        Motor_Timing_TraceArmFlight();
        for (i = 0U; i < 29U; i++) assert(!Motor_Timing_TraceDue(1U));
        assert(Motor_Timing_TraceDue(1U)); Motor_Timing_TracePushLight(&t);
        Motor_Timing_TraceStop();
        assert(Motor_Timing_TraceCount() == 1U && Motor_Timing_TraceGetLight(0U, &out));
        Motor_Timing_TraceArmLight();
        assert(!Motor_Timing_TraceIsFlight());
        Motor_Timing_TraceStop();
    }
    Motor_Timing_ProfileSetDetailed(false);
    while (!Motor_Timing_ProfileBegin()) { }
    assert(Motor_Timing_ProfileIsActive());
    assert(!Motor_Timing_ProfileDetailedIsActive());
    Motor_Timing_ProfileSetDetailed(true);
    assert(Motor_Timing_ProfileDetailedIsActive());
    Motor_Timing_ProfileEnd();
    assert(!Motor_Timing_ProfileDetailedIsActive());
    Motor_Timing_ProfileSetDetailed(false);
    {
        motor_trace_compact_sample_t compact = {0}, out;
        motor_trace_sample_t full = {0};
        float runtime_before = Motor_DmaInt_GetAverageRuntimeUs();
        unsigned written = 0U;
        assert(sizeof(compact) == 32U);
        Motor_Timing_TraceArmCompact();
        assert(Motor_Timing_TraceIsCompact() && Motor_Timing_TraceCompactIsActive());
        assert(!Motor_Timing_CallbackIsActive() && !Motor_Timing_IrqDue(0U));
        for (i = 0U; i < 256U; i++) assert(!Motor_Timing_ProfileBegin());
        now_us = 999U;
        Motor_Timing_RecordRuntime(0U);
        assert(Motor_DmaInt_GetAverageRuntimeUs() == runtime_before);
        assert(!Motor_Timing_TraceDue(2U));
        Motor_Timing_TracePush(&full);
        assert(Motor_Timing_TraceCount() == 0U);
        assert(!Motor_Timing_TraceCompactDue(1000U, 1U, 0U, false));
        for (i = 0U; i < (MOTOR_TRACE_COMPACT_CAPACITY + 20U) * MOTOR_TRACE_COMPACT_STRIDE; i++) {
            uint32_t time_us = 1000U + i * 67U;
            Motor_Timing_SpeedTaskNote(time_us, false, 0.0f, 0.0f, 0.0f);
            if (Motor_Timing_TraceCompactDue(time_us, 1U, 0U, true)) {
                compact.time_us = time_us;
                compact.id = (float)written++;
                Motor_Timing_TracePushCompact(&compact);
            }
        }
        assert(written == MOTOR_TRACE_COMPACT_CAPACITY + 20U);
        assert(!Motor_Timing_TraceGetCompact(0U, &out));
        Motor_Timing_TraceStop();
        assert(Motor_Timing_TraceCount() == MOTOR_TRACE_COMPACT_CAPACITY);
        assert(Motor_Timing_TraceGetCompact(0U, &out) && out.id == 20.0f);
        assert(Motor_Timing_TraceGetCompact(MOTOR_TRACE_COMPACT_CAPACITY - 1U, &out));
        assert(out.id == (float)(MOTOR_TRACE_COMPACT_CAPACITY + 19U));
        assert(!Motor_Timing_TraceGetCompact(MOTOR_TRACE_COMPACT_CAPACITY, &out));
        assert(!Motor_Timing_TraceGetCompact(0U, NULL));
        assert(!Motor_Timing_TraceGet(0U, &full));

        Motor_Timing_TraceArmCompact();
        Motor_Timing_SpeedTaskNote(1000U, false, 0.0f, 0.0f, 0.0f);
        assert(Motor_Timing_TraceCompactDue(1000U, 2U, 0U, true));
        compact.time_us = 1000U;
        Motor_Timing_TracePushCompact(&compact);
        assert(Motor_Timing_TraceCompactDue(1001U, 1U, 0U, true));
        compact.time_us = 1001U;
        Motor_Timing_TracePushCompact(&compact);
        for (i = 0U; i < (MOTOR_TRACE_COMPACT_POST - 1U) * MOTOR_TRACE_COMPACT_STRIDE; i++) {
            uint32_t time_us = 1002U + i;
            Motor_Timing_SpeedTaskNote(time_us, false, 0.0f, 0.0f, 0.0f);
            if (Motor_Timing_TraceCompactDue(time_us, 1U, 0U, true)) {
                compact.time_us = time_us;
                Motor_Timing_TracePushCompact(&compact);
            }
        }
        assert(!Motor_Timing_TraceCompactIsActive());
        assert(Motor_Timing_TraceCount() == MOTOR_TRACE_COMPACT_POST + 1U);
        assert(Motor_Timing_TraceGetCompact(0U, &out) && (out.flags & MOTOR_TRACE_COMPACT_ENTRY));
        assert(Motor_Timing_TraceGetCompact(1U, &out) && (out.flags & MOTOR_TRACE_COMPACT_LOSS));

        Motor_Timing_TraceArmCompact();
        assert(Motor_Timing_TraceCompactDue(2000U, 1U, 42U, true));
        compact.time_us = 2000U;
        Motor_Timing_TracePushCompact(&compact);
        assert(Motor_Timing_TraceCompactDue(2001U, 1U, 42U, false));
        compact.time_us = 2001U;
        Motor_Timing_TracePushCompact(&compact);
        assert(!Motor_Timing_TraceCompactIsActive());
        assert(Motor_Timing_TraceGetCompact(0U, &out));
        assert((out.flags & MOTOR_TRACE_COMPACT_FAULT) && (out.flags >> 16) == 42U);
        assert(Motor_Timing_TraceGetCompact(1U, &out) && (out.flags & MOTOR_TRACE_COMPACT_STOP));

        Motor_Timing_TraceArmCompact();
        Motor_Timing_SpeedTaskNote(3000U, false, 0.0f, 0.0f, 0.0f);
        assert(Motor_Timing_TraceCompactDue(3000U, 2U, 0U, true));
        Motor_Timing_TracePushCompact(&compact);
        assert(Motor_Timing_TraceCompactDue(3001U, 0U, 0U, false));
        Motor_Timing_TracePushCompact(&compact);
        assert(Motor_Timing_TraceGetCompact(1U, &out));
        assert((out.flags & MOTOR_TRACE_COMPACT_STOP) && !(out.flags & MOTOR_TRACE_COMPACT_LOSS));
        Motor_Timing_TraceArmCompact();
        assert(!Motor_Timing_SpeedTaskIsActive());
        Motor_Timing_SpeedTaskHeartbeat(UINT32_MAX - 100U);
        assert(!Motor_Timing_TraceCompactDue(9899U, 1U, 0U, true));
        assert(Motor_Timing_TraceCompactDue(9900U, 1U, 0U, true));
        compact.time_us = 9900U;
        Motor_Timing_TracePushCompact(&compact);
        Motor_Timing_TraceStop();
        assert(Motor_Timing_TraceGetCompact(0U, &out) && (out.flags & MOTOR_TRACE_COMPACT_TASK_GAP));
        Motor_Timing_CadenceReset();
        assert(Motor_Timing_IrqDue(0U));
        Motor_Timing_TraceArmLight();
        assert(!Motor_Timing_TraceIsCompact());
        assert(!Motor_Timing_TraceGetCompact(0U, &out));
        Motor_Timing_TraceStop();
    }
    return 0;
}
