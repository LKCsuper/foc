/* 时序与性能监控主机测试：运行时间、故障轨迹与调度诊断 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_timing.h"

MOTOR_PARAM_S motor;
MOTOR_PARAM_S *gpstMotor = &motor;

static float s_adc_hz;
static uint32_t s_time_us;

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：估算 ADC/控制采样触发频率
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_adc_hz
 */
float Hal_Motor_AdcSampleRateHz(void) { return s_adc_hz; }
/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 s_time_us
 */
uint32_t Hal_Motor_TimeUs(void) { return s_time_us; }

/**
 * @description: 判断两浮点值是否在测试容差内。
 * @param {float} a a 参数
 * @param {float} b b 参数
 * @param {float} tolerance tolerance 参数
 * @return {int} 本次计算或查询的结果
 */
static int Near(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
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
    {
        uint32_t sequence;
        assert(sizeof(g_motor_schedule_debug) == 280U);
        Motor_Timing_WakeProbeReset();
        assert(g_motor_schedule_debug.state == 0U);
        Motor_Timing_SpeedTaskNote(1000U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_SpeedTaskNote(2400U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_SpeedPiNote(1000U, true);
        Motor_Timing_SpeedPiNote(2500U, true);
        Motor_Timing_SpeedPiNote(2600U, false);
        Motor_Timing_IrqNote(0U, 10U, 20U, 3U);
        Motor_Timing_ScheduleDebugFreeze();
        assert(g_motor_schedule_debug.magic == 0x464f4353U);
        assert(g_motor_schedule_debug.version == 3U);
        assert(g_motor_schedule_debug.state == 2U);
        assert(!(g_motor_schedule_debug.sequence & 1U));
        assert(g_motor_schedule_debug.mode == 11U);
        assert(g_motor_schedule_debug.task_count == 2U);
        assert(g_motor_schedule_debug.task_max_gap_us == 1400U);
        assert(g_motor_schedule_debug.task_gap_bins[3] == 1U);
        assert(g_motor_schedule_debug.worst_gap.sequence == 2U);
        assert(g_motor_schedule_debug.worst_gap.time_us == 2400U);
        assert(g_motor_schedule_debug.worst_gap.previous_us == 1000U);
        assert(g_motor_schedule_debug.worst_gap.wake.valid == 0U);
        assert(g_motor_schedule_debug.pi_count == 2U);
        assert(g_motor_schedule_debug.pi_no_update == 1U);
        assert(g_motor_schedule_debug.pi_max_gap_us == 1500U);
        assert(g_motor_schedule_debug.pi_over_1ms == 1U);
        assert(g_motor_schedule_debug.pi_streak_max == 1U);
        Motor_Timing_SpeedPiNote(10000U, true);
        assert(g_motor_schedule_debug.pi_count == 2U);
        assert(!Motor_Timing_SpeedTaskIsActive());
        assert(g_motor_schedule_debug.irq[0][1] == 0U);
        sequence = g_motor_schedule_debug.sequence;
        Motor_Timing_ScheduleDebugFreeze();
        assert(g_motor_schedule_debug.sequence == sequence);
        assert(g_motor_schedule_debug.mode == 11U);
        Motor_Timing_CadenceReset();
        assert(g_motor_schedule_debug.state == 0U);
        assert(Motor_Timing_IrqDue(0U));
        Motor_Timing_IrqNote(0U, 10U, 20U, 3U);
        Motor_Timing_ScheduleDebugFreeze();
        assert(g_motor_schedule_debug.mode == 12U);
        assert(g_motor_schedule_debug.irq[0][0] == 1U);
        assert(g_motor_schedule_debug.irq[0][1] == 1U);
        assert(g_motor_schedule_debug.irq[0][2] == 10U);
        assert(g_motor_schedule_debug.irq[0][3] == 10U);
        assert(g_motor_schedule_debug.irq[0][4] == 10U);
        assert(g_motor_schedule_debug.irq[0][5] == 20U);
        assert(g_motor_schedule_debug.irq[0][6] == 1U);
        assert(g_motor_schedule_debug.irq[0][7] == 1U);
        g_motor_schedule_debug.sequence = 0xfffffffeU;
        Motor_Timing_SpeedTaskReset();
        Motor_Timing_ScheduleDebugFreeze();
        assert(g_motor_schedule_debug.sequence == 0U);
    }
    {
        uint32_t features;
        Motor_Timing_TraceStop();
        Motor_Timing_HandoffStop();
        Motor_Timing_CadenceStop();
        assert(Motor_Timing_IsrFeatures() == 0U);
        Motor_Timing_SpeedTaskReset();
        assert(Motor_Timing_IsrFeatures() == 0U);
        Motor_Timing_WakeProbeReset();
        assert(Motor_Timing_IsrFeatures() == 0U);
        Motor_Timing_CadenceReset();
        assert(Motor_Timing_IsrFeatures() == (MOTOR_TIMING_ISR_PROFILE | MOTOR_TIMING_ISR_RUNTIME));
        Motor_Timing_TraceArmCompact();
        assert(Motor_Timing_IsrFeatures() == MOTOR_TIMING_ISR_COMPACT);
        Motor_Timing_TraceStop();
        assert(Motor_Timing_IsrFeatures() == 0U);
        Motor_Timing_FaultTraceArm();
        features = Motor_Timing_IsrFeatures();
        assert((features & MOTOR_TIMING_ISR_FAULT) != 0U);
        assert((features & (MOTOR_TIMING_ISR_TRACE | MOTOR_TIMING_ISR_COMPACT | MOTOR_TIMING_ISR_RUNTIME)) == 0U);
        Motor_Timing_FaultTraceFreeze(0U);
        assert((Motor_Timing_IsrFeatures() & MOTOR_TIMING_ISR_FAULT) == 0U);
        Motor_Timing_TraceArm();
        assert((Motor_Timing_IsrFeatures() & MOTOR_TIMING_ISR_TRACE) != 0U);
        assert(Motor_Timing_TraceDue(0U) == false);
        Motor_Timing_PairArm();
        assert((Motor_Timing_IsrFeatures() & MOTOR_TIMING_ISR_PAIR) != 0U);
        Motor_Timing_PairHalfArm();
        assert((Motor_Timing_IsrFeatures() & MOTOR_TIMING_ISR_PAIR) == 0U);
        Motor_Timing_HandoffStop();
        Motor_Timing_TraceStop();
        Motor_Timing_CadenceStop();
        assert(Motor_Timing_IsrFeatures() == 0U);
    }
    {
        const uint32_t gaps[] = {1000U, 1100U, 1250U, 1500U, 2000U, 2001U};
        uint32_t time_us = 1000U;
        unsigned index;
        Motor_Timing_WakeProbeReset();
        Motor_Timing_SpeedTaskNote(time_us, false, 0.0f, 0.0f, 0.0f);
        for (index = 0U; index < 6U; index++) {
            time_us += gaps[index];
            Motor_Timing_WakeProbeArm(index);
            s_time_us = time_us - 100U;
            Motor_Timing_WakeProbeTick(index);
            Motor_Timing_WakeProbeDmaNote(s_time_us, s_time_us + 10U + index);
            s_time_us += 20U + index;
            Motor_Timing_SpeedTaskWake(s_time_us, index, index);
            Motor_Timing_SpeedTaskNote(time_us, false, 0.0f, 0.0f, 0.0f);
        }
        Motor_Timing_SpeedTaskNote(time_us + 1000U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_ScheduleDebugFreeze();
        assert(g_motor_schedule_debug.task_count == 8U);
        assert(g_motor_schedule_debug.task_gap_bins[0] == 2U);
        for (index = 1U; index < 6U; index++)
            assert(g_motor_schedule_debug.task_gap_bins[index] == 1U);
        assert(g_motor_schedule_debug.worst_gap.sequence == 7U);
        assert(g_motor_schedule_debug.worst_gap.time_us == time_us);
        assert(g_motor_schedule_debug.worst_gap.previous_us == time_us - 2001U);
        assert(g_motor_schedule_debug.worst_gap.wake.valid == 1U);
        assert(g_motor_schedule_debug.worst_gap.wake.delay_us == 25U);
        assert(g_motor_schedule_debug.worst_gap.wake.dma_us == 15U);
        assert(g_motor_schedule_debug.worst_gap.previous_wake.delay_us == 24U);
        assert(g_motor_schedule_debug.worst_gap.previous_wake.dma_us == 14U);
        {
            motor_split_diag_t split;
            motor_callback_t callback;
            Motor_Timing_SplitReset();
            assert(Motor_Timing_IsrFeatures() == 0U);
            assert(g_motor_timing_split_active == 1U);
            s_time_us = 5000U;
            Motor_Timing_WakeProbeArm(3U);
            Motor_Timing_WakeProbeTick(3U);
            Motor_Timing_CallbackBegin(5010U);
            Motor_Timing_CallbackClassify(true);
            Motor_Timing_CallbackPublish(11U);
            Motor_Timing_CallbackEnd(5055U, false);
            Motor_Timing_WakeProbeDmaNote(5010U, 5055U);
            Motor_Timing_CallbackBegin(5060U);
            Motor_Timing_CallbackClassify(false);
            Motor_Timing_CallbackEnd(5064U, false);
            Motor_Timing_WakeProbeDmaNote(5060U, 5064U);
            s_time_us = 5120U;
            Motor_Timing_SpeedTaskWake(5120U, 3U, 3U);
            Motor_Timing_CallbackGet(&callback);
            Motor_Timing_SplitGet(&split);
            assert(callback.full == 1U && callback.full_sum == 45U && callback.skip_sum == 4U);
            assert(callback.publish == 1U && callback.publish_sum == 11U && callback.publish_on_full == 1U);
            assert(split.wake_count == 1U && split.wake_delay_sum_us == 120U && split.wake_dma_sum_us == 49U);
            assert(split.wake_full_sum_us == 45U && split.wake_skip_sum_us == 4U);
            assert(split.wake_publish_count == 1U && split.wake_publish_sum_us == 11U);
            assert(split.worst_delay_us == 120U && split.worst_dma_us == 49U);
            assert(split.worst_full_us + split.worst_skip_us == split.worst_dma_us);
            assert(split.worst_publish_us == 11U);
            s_time_us = 6000U;
            Motor_Timing_WakeProbeArm(4U);
            Motor_Timing_WakeProbeTick(4U);
            Motor_Timing_CallbackBegin(6010U);
            Motor_Timing_CallbackClassify(false);
            Motor_Timing_CallbackEnd(6014U, false);
            Motor_Timing_WakeProbeDmaNote(6010U, 6014U);
            s_time_us = 6200U;
            Motor_Timing_SpeedTaskWake(6200U, 4U, 4U);
            Motor_Timing_SplitGet(&split);
            assert(split.wake_count == 2U && split.wake_publish_count == 1U);
            assert(split.worst_delay_us == 200U && split.worst_dma_us == 4U);
            assert(split.worst_skip_us == 4U && split.worst_full_us == 0U && split.worst_publish_us == 0U);
            assert(split.wake_full_sum_us + split.wake_skip_sum_us == split.wake_dma_sum_us);
        }
        Motor_Timing_SpeedTaskReset();
        Motor_Timing_SpeedTaskNote(0xfffffff0U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_SpeedTaskNote(10U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_ScheduleDebugFreeze();
        assert(g_motor_schedule_debug.task_max_gap_us == 26U);
        assert(g_motor_schedule_debug.task_gap_bins[0] == 1U);
        assert(g_motor_schedule_debug.task_gap_bins[5] == 0U);
        assert(g_motor_schedule_debug.worst_gap.previous_wake.valid == 0U);
    }
    {
        Motor_Timing_SpeedTaskReset();
        Motor_Timing_SpeedTaskNote(1000U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_SpeedTaskStage(1010U, 10U, 1U);
        Motor_Timing_SpeedTaskStage(2000U, 20U, 2U);
        Motor_Timing_SpeedTaskStage(2100U, 21U, 3U);
        Motor_Timing_SpeedTaskStage(2200U, 22U, 4U);
        Motor_Timing_SpeedTaskNote(2250U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_ScheduleDebugFreeze();
        assert(g_motor_schedule_debug.worst_gap.phase_valid == 1U);
        assert(g_motor_schedule_debug.worst_gap.before_wait_us == 10U);
        assert(g_motor_schedule_debug.worst_gap.phase_us[0] == 990U);
        assert(g_motor_schedule_debug.worst_gap.phase_us[1] == 100U);
        assert(g_motor_schedule_debug.worst_gap.phase_us[2] == 100U);
        assert(g_motor_schedule_debug.worst_gap.phase_us[3] == 50U);
        Motor_Timing_SpeedTaskReset();
        Motor_Timing_SpeedTaskNote(0xfffffff0U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_SpeedTaskStage(0xfffffff5U, 0U, 1U);
        Motor_Timing_SpeedTaskStage(0U, 0U, 2U);
        Motor_Timing_SpeedTaskStage(5U, 0U, 3U);
        Motor_Timing_SpeedTaskStage(8U, 0U, 4U);
        Motor_Timing_SpeedTaskNote(10U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_ScheduleDebugFreeze();
        assert(g_motor_schedule_debug.worst_gap.phase_valid == 1U);
        assert(g_motor_schedule_debug.worst_gap.before_wait_us == 5U);
        assert(g_motor_schedule_debug.worst_gap.phase_us[0] == 11U);
        Motor_Timing_SpeedTaskReset();
        Motor_Timing_SpeedTaskNote(1000U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_SpeedTaskStage(1010U, 0U, 1U);
        Motor_Timing_SpeedTaskStage(1100U, 0U, 3U);
        Motor_Timing_SpeedTaskStage(1200U, 0U, 4U);
        Motor_Timing_SpeedTaskNote(2000U, false, 0.0f, 0.0f, 0.0f);
        Motor_Timing_ScheduleDebugFreeze();
        assert(g_motor_schedule_debug.worst_gap.phase_valid == 0U);
    }
    Motor_Timing_SpeedTaskReset();
    s_time_us = 100U;
    Motor_Timing_TaskRuntimeBegin();
    assert(Motor_Timing_WakeProbeDmaDue());
    Motor_Timing_WakeProbeDmaNote(110U, 140U);
    s_time_us = 166U;
    Motor_Timing_TaskRuntimeEnd();
    Motor_Timing_TaskRuntimeEnd();
    s_time_us = 0xfffffff0U;
    Motor_Timing_TaskRuntimeBegin();
    s_time_us = 4U;
    Motor_Timing_TaskRuntimeEnd();
    assert(g_motor_task_runtime_debug.count == 2U);
    assert(g_motor_task_runtime_debug.sum_us == 86U);
    assert(g_motor_task_runtime_debug.min_us == 20U);
    assert(g_motor_task_runtime_debug.max_us == 66U);
    assert(g_motor_task_runtime_debug.dma_sum_us == 30U);
    assert(g_motor_task_runtime_debug.max_wall_dma_us == 30U);
    assert(g_motor_task_runtime_debug.max_wall_dma_count == 1U);
    Motor_Timing_ScheduleDebugFreeze();
    Motor_Timing_TaskRuntimeBegin();
    Motor_Timing_TaskRuntimeEnd();
    assert(g_motor_task_runtime_debug.count == 2U);
    Motor_Timing_SpeedTaskReset();
    assert(g_motor_task_runtime_debug.count == 0U);
    Motor_Timing_TaskRuntimeBegin();
    Motor_Timing_ScheduleDebugFreeze();
    Motor_Timing_TaskRuntimeEnd();
    assert(g_motor_task_runtime_debug.count == 0U);
    assert(!Motor_Timing_WakeProbeDmaDue());
    {
        motor_trace_light_sample_t sample;
        motor_speed_task_diag_t full;
        memset(&sample, 0, sizeof(sample));
        sample.time_us = 10U;
        sample.theta = 1.25f;
        Motor_Timing_SpeedTaskReset();
        Motor_Timing_SpeedTaskCaptureLight(&sample);
        assert(sample.task_age_us == UINT32_MAX && sample.task_phase_age_us == UINT32_MAX);
        Motor_Timing_SpeedTaskStage(0xfffffff0U, 9U, 3U);
        Motor_Timing_SpeedTaskNote(0xfffffff8U, true, 300.0f, 299.0f, 1.0f);
        Motor_Timing_SpeedTaskGet(&full);
        Motor_Timing_SpeedTaskCaptureLight(&sample);
        assert(sample.task_count == full.count && sample.task_age_us == 18U);
        assert(sample.task_gate == 1U && sample.speed_target == full.target);
        assert(sample.speed_feedback == full.feedback && sample.speed_iq == full.iq);
        assert(sample.task_tick == full.tick && sample.task_phase == full.phase);
        assert(sample.task_phase_age_us == 26U && sample.theta == 1.25f);
    }
    puts("test_motor_timing: pass");
    return 0;
}
