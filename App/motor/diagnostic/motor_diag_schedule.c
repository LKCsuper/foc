/*
 * @Description: 控制中断、唤醒探针、任务运行时、速度任务与回调节拍诊断
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_timing.h"
#include "motor_timing_internal.h"
#include "motor_diag_phase.h"
#include "common_assert.h"

#include <stddef.h>
#include <math.h>
#include <string.h>

#include "hal_motor.h"
#include "motor_crit.h"
#if defined(__CC_ARM)
/* 只优化诊断计数和样本复制的调用开销，不改变浮点规则或控制节拍。 */
#pragma O2
#endif
#if MOTOR_CFG_DIAG
motor_phase_debug_t g_motor_phase_debug;
volatile uint32_t g_motor_phase_active;
static bool s_phase_seeded;
static bool s_phase_down;
static uint32_t s_phase_post;
static uint32_t s_phase_start_us;
static bool s_phase_irq_open;
static uint32_t s_phase_irq_start_us;
static uint32_t s_phase_irq_slot;
static uint32_t s_phase_irq_sequence;
static bool s_phase_platform_running;
static bool s_phase_platform_waiting;
static uint32_t s_phase_platform_us;

void Motor_Timing_PhaseEdgeArm(void)
{
    uint32_t key = Motor_Crit_Enter();
    g_motor_phase_active = 0U;
    memset(&g_motor_phase_debug, 0, sizeof(g_motor_phase_debug));
    g_motor_phase_debug.magic = 0x50484153U;
    g_motor_phase_debug.version = 2U;
    s_phase_seeded = false;
    s_phase_post = 0U;
    s_phase_irq_open = false;
    s_phase_irq_sequence = 0U;
    s_phase_platform_running = false;
    s_phase_platform_waiting = false;
    g_motor_phase_active = 1U;
    Motor_Crit_Exit(key);
}

void Motor_Timing_PhaseEdgeReserve3000(void)
{
    Motor_Timing_PhaseEdgeArm();
    g_motor_phase_active = 3U;
}

void Motor_Timing_PhaseEdgePoll(bool running, bool ready, uint32_t time_us)
{
    if (g_motor_phase_active != 3U) return;
    if (!running) {
        if (s_phase_platform_running) {
            g_motor_phase_active = 0U;
            g_motor_phase_debug.reason = 2U;
        }
        s_phase_platform_waiting = false;
        return;
    }
    s_phase_platform_running = true;
    if (!ready) {
        s_phase_platform_waiting = false;
        return;
    }
    if (!s_phase_platform_waiting) {
        s_phase_platform_waiting = true;
        s_phase_platform_us = time_us;
    } else if ((uint32_t)(time_us - s_phase_platform_us) >= 1000000U) {
        g_motor_phase_active = 1U;
    }
}

void Motor_Timing_PhaseEdgeStop(void)
{
    uint32_t key = Motor_Crit_Enter();
    g_motor_phase_active = 0U;
    if (g_motor_phase_debug.reason == 0U) g_motor_phase_debug.reason = 2U;
    Motor_Crit_Exit(key);
}

void Motor_Timing_PhaseIrqBegin(uint32_t time_us)
{
    s_phase_irq_start_us = time_us;
    s_phase_irq_sequence = 0U;
    s_phase_irq_open = true;
}

void Motor_Timing_PhaseIrqEnd(uint32_t time_us, uint32_t dma_status, uint32_t adc_status,
                              uint32_t remaining, uint32_t pending)
{
    motor_phase_event_t *event;
    if (!s_phase_irq_open) return;
    s_phase_irq_open = false;
    if (s_phase_irq_sequence == 0U) return;
    event = &g_motor_phase_debug.events[s_phase_irq_slot];
    if (event->sequence != s_phase_irq_sequence) return;
    event->irq_exit_us = time_us;
    event->dma_status = dma_status;
    event->adc_status = adc_status;
    event->dma_remaining = remaining;
    event->irq_pending = pending;
    event->flags |= 2U;
}

void Motor_Timing_PhaseMarkStatus(void)
{
    motor_phase_event_t *event;
    if (!s_phase_irq_open || s_phase_irq_sequence == 0U) return;
    event = &g_motor_phase_debug.events[s_phase_irq_slot];
    if (event->sequence == s_phase_irq_sequence) event->flags |= 4U;
}

void Motor_Timing_PhaseEdgeNote(uint32_t time_us, uint32_t count, uint32_t cr1,
                            uint32_t divider, uint32_t flags, bool running)
{
    motor_phase_event_t *event;
    bool down = (cr1 & 16U) != 0U;
    bool post = g_motor_phase_active == 2U;
    if (g_motor_phase_active == 0U || g_motor_phase_active == 3U) return;
    if (!running) {
        if (g_motor_phase_debug.seen != 0U) {
            g_motor_phase_active = 0U;
            if (g_motor_phase_debug.reason == 0U) g_motor_phase_debug.reason = 2U;
        }
        return;
    }
    if (g_motor_phase_debug.seen == 0U) s_phase_start_us = time_us;
    g_motor_phase_debug.seen++;
    event = &g_motor_phase_debug.events[g_motor_phase_debug.head];
    event->sequence = g_motor_phase_debug.seen;
    event->time_us = time_us;
    event->count = count;
    event->cr1 = cr1;
    event->divider = divider;
    event->flags = flags;
    event->irq_entry_us = s_phase_irq_open ? s_phase_irq_start_us : 0U;
    event->irq_exit_us = 0U;
    event->dma_status = 0U;
    event->adc_status = 0U;
    event->dma_remaining = 0U;
    event->irq_pending = 0U;
    if (s_phase_irq_open) {
        s_phase_irq_slot = g_motor_phase_debug.head;
        s_phase_irq_sequence = event->sequence;
    }
    g_motor_phase_debug.head = (g_motor_phase_debug.head + 1U) % MOTOR_PHASE_CAPACITY;
    if (g_motor_phase_debug.count < MOTOR_PHASE_CAPACITY) g_motor_phase_debug.count++;
    if ((flags & 1U) != 0U) {
        if (s_phase_seeded && down != s_phase_down && !post) {
            g_motor_phase_debug.trigger_sequence = g_motor_phase_debug.seen;
            g_motor_phase_debug.reason = 1U;
            s_phase_post = 16U;
            g_motor_phase_active = 2U;
        }
        s_phase_down = down;
        s_phase_seeded = true;
    }
    if (post && --s_phase_post == 0U) g_motor_phase_active = 0U;
    if (g_motor_phase_active == 1U && (uint32_t)(time_us - s_phase_start_us) >= 10000000U) {
        g_motor_phase_debug.reason = 3U;
        g_motor_phase_active = 0U;
    }
}

volatile uint32_t s_speed_complete_us;
bool s_cadence_active;
bool s_task_only;
static volatile bool s_probe_enabled, s_probe_armed, s_probe_tick_seen;
static volatile uint32_t s_probe_deadline, s_probe_start, s_probe_dma_count, s_probe_dma_us;
static motor_wake_probe_t s_probe_cycle;
static bool s_callback_full;
static bool s_beat_published;
static uint32_t s_beat_publish_us;
static uint32_t s_wake_full_count, s_wake_full_us, s_wake_skip_count, s_wake_skip_us;
static uint32_t s_wake_publish_count, s_wake_publish_us;
static motor_split_diag_t s_split;
volatile uint8_t g_motor_timing_split_active;
static motor_irq_diag_t s_irq_diag[2];
static uint8_t s_irq_divider[2];
/**
 * @description: 判断本次 IRQ 是否允许记一次耗时。
 * @detail: 节拍统计开启且不是只统计速度任务时，每次调用都加总次数。每 31 次放行一次计时。
 *          kind 0 为 DMA 函数体，1 为 SysTick 函数体；其他值返回 false。CadenceReset 清零分频。不改控制。
 * @param {unsigned} kind 0=DMA，1=SysTick
 * @return {bool} true=本拍应调用 IrqNote
 * @author: lkc
 */
bool Motor_Timing_IrqDue(unsigned kind)
{
    if (!s_cadence_active || s_task_only || kind >= 2U) return false;
    s_irq_diag[kind].seen++;
    if (s_irq_divider[kind] == 0U) {
        s_irq_divider[kind] = MOTOR_TIMING_IRQ_STRIDE - 1U;
        return true;
    }
    s_irq_divider[kind]--;
    return false;
}
/**
 * @description: 记录一次已放行的 IRQ 函数体耗时。
 * @detail: kind 0 为 DMA，1 为 SysTick。pending 的 bit0 为 SysTick，bit1 为 PendSV。
 *          SysTick 耗时包含被 DMA 抢占的时间，不能与 DMA 相加。不含异常入出栈和本函数自身。
 *          未开启或 kind 非法则返回。不改控制。
 * @param {unsigned} kind       0=DMA，1=SysTick
 * @param {uint32_t} start_us   函数体入口时间 (us)
 * @param {uint32_t} end_us     函数体返回时间 (us)
 * @param {unsigned} pending    bit0=SysTick 挂起，bit1=PendSV 挂起
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_IrqNote(unsigned kind, uint32_t start_us, uint32_t end_us, unsigned pending)
{
    motor_irq_diag_t *d;
    uint32_t elapsed;
    if (!s_cadence_active || s_task_only || kind >= 2U) return;
    d = &s_irq_diag[kind];
    elapsed = end_us - start_us;
    if (!d->count) d->first_us = start_us;
    d->count++;
    d->last_us = end_us;
    d->sum_us += elapsed;
    if (elapsed > d->max_us) d->max_us = elapsed;
    if (pending & 1U) d->tick_pending++;
    if (pending & 2U) d->switch_pending++;
}
/**
 * @description: 拷贝一类 IRQ 耗时统计。
 * @detail: 进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。kind 非法则不写输出。
 * @param {unsigned} kind         0=DMA，1=SysTick
 * @param {motor_irq_diag_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_IrqGet(unsigned kind, motor_irq_diag_t *out)
{
    uint32_t key;
    COMMON_ASSERT(out != NULL);
    if (kind >= 2U) return;
    key = Motor_Crit_Enter();
    *out = s_irq_diag[kind];
    Motor_Crit_Exit(key);
}
static motor_speed_task_diag_t s_speed_task;
static motor_wake_probe_t s_task_previous_wake;
static uint32_t s_task_gap_bins[6];
static motor_task_gap_event_t s_task_worst_gap;
static uint32_t s_task_phase_stamp[4], s_task_phase_mask;
volatile motor_schedule_debug_t g_motor_schedule_debug = {.magic = 0x464f4353U, .version = 3U};
typedef char motor_schedule_debug_size_check_t[sizeof(motor_schedule_debug_t) == 280U ? 1 : -1];
/**
 * @description: 查询目标 tick 到任务唤醒的 DMA 探测是否已武装。
 * @return {bool} true=已武装
 * @author: lkc
 */
bool Motor_Timing_WakeProbeArmed(void) { return s_probe_armed; }
/**
 * @description: 武装一轮从截止 tick 到任务唤醒的 DMA 计时。
 * @detail: 仅在 WakeProbeReset 打开探测后生效。不改任务调度。
 * @param {uint32_t} deadline_tick 本轮 osDelayUntil 截止 tick
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_WakeProbeArm(uint32_t deadline_tick)
{
    uint32_t key;
    if (!s_probe_enabled) return;
    key = Motor_Crit_Enter();
    s_probe_deadline = deadline_tick;
    s_probe_tick_seen = false;
    s_probe_dma_count = 0U;
    s_probe_dma_us = 0U;
    s_wake_full_count = s_wake_full_us = s_wake_skip_count = s_wake_skip_us = 0U;
    s_wake_publish_count = s_wake_publish_us = 0U;
    s_probe_armed = true;
    Motor_Crit_Exit(key);
}
/**
 * @description: 在截止 tick 的 SysTick 入口记下探测起点。
 * @detail: 只记第一次命中。不改任务调度。
 * @param {uint32_t} next_tick 即将发生的 tick
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_WakeProbeTick(uint32_t next_tick)
{
    if (s_probe_armed && !s_probe_tick_seen && next_tick == s_probe_deadline) {
        uint32_t key = Motor_Crit_Enter();
        s_probe_start = Hal_Motor_TimeUs();
        s_probe_tick_seen = true;
        Motor_Crit_Exit(key);
    }
}
/**
 * @description: 查询唤醒或任务执行窗口是否需要累计 DMA。
 * @return {bool} true=应调用 WakeProbeDmaNote
 * @author: lkc
 */
static volatile bool s_task_runtime_active;
static volatile uint32_t s_task_runtime_dma_us, s_task_runtime_dma_count;
bool Motor_Timing_WakeProbeDmaDue(void) { return (s_probe_armed && s_probe_tick_seen) || s_task_runtime_active; }
/**
 * @description: 分别累计唤醒等待窗口与任务执行窗口的一次 DMA 耗时。
 * @detail: 未到采集窗口则返回。不改 PWM 或任务调度。
 * @param {uint32_t} start_us DMA 入口时间 (us)
 * @param {uint32_t} end_us   DMA 返回时间 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_WakeProbeDmaNote(uint32_t start_us, uint32_t end_us)
{
    if (!Motor_Timing_WakeProbeDmaDue()) return;
    if (s_probe_armed && s_probe_tick_seen) {
        uint32_t elapsed = end_us - start_us;
        s_probe_dma_count++;
        s_probe_dma_us += elapsed;
        if (g_motor_timing_split_active) {
            if (s_callback_full) {
                s_wake_full_count++;
                s_wake_full_us += elapsed;
            } else {
                s_wake_skip_count++;
                s_wake_skip_us += elapsed;
            }
            if (s_beat_published) {
                s_wake_publish_count++;
                s_wake_publish_us += s_beat_publish_us;
            }
        }
    }
    if (s_task_runtime_active) {
        s_task_runtime_dma_count++;
        s_task_runtime_dma_us += end_us - start_us;
    }
}
/**
 * @description: 查询速度任务统计是否开启。
 * @detail: 供调用方跳过不必要的诊断参数采集。不改控制。
 * @return {bool} true=节拍统计开启
 * @author: lkc
 */
bool Motor_Timing_SpeedTaskIsActive(void) { return s_cadence_active; }

volatile motor_task_runtime_debug_t g_motor_task_runtime_debug;
static uint32_t s_task_runtime_start;

void Motor_Timing_TaskRuntimeBegin(void)
{
    uint32_t key;
    if (!s_cadence_active) {
        s_task_runtime_active = false;
        return;
    }
    key = Motor_Crit_Enter();
    s_task_runtime_dma_us = 0U;
    s_task_runtime_dma_count = 0U;
    s_task_runtime_start = Hal_Motor_TimeUs();
    s_task_runtime_active = s_cadence_active;
    Motor_Crit_Exit(key);
}

void Motor_Timing_TaskRuntimeEnd(void)
{
    uint32_t elapsed;
    uint32_t key;
    if (!s_task_runtime_active) return;
    key = Motor_Crit_Enter();
    elapsed = Hal_Motor_TimeUs() - s_task_runtime_start;
    s_task_runtime_active = false;
    if (s_cadence_active && g_motor_task_runtime_debug.count < 100000U) {
        if (!g_motor_task_runtime_debug.count || elapsed < g_motor_task_runtime_debug.min_us)
            g_motor_task_runtime_debug.min_us = elapsed;
        if (!g_motor_task_runtime_debug.count || elapsed > g_motor_task_runtime_debug.max_us) {
            g_motor_task_runtime_debug.max_us = elapsed;
            g_motor_task_runtime_debug.max_wall_dma_us = s_task_runtime_dma_us;
            g_motor_task_runtime_debug.max_wall_dma_count = s_task_runtime_dma_count;
        }
        g_motor_task_runtime_debug.last_us = elapsed;
        g_motor_task_runtime_debug.sum_us += elapsed;
        g_motor_task_runtime_debug.dma_sum_us += s_task_runtime_dma_us;
        g_motor_task_runtime_debug.count++;
    }
    Motor_Crit_Exit(key);
}
/**
 * @description: 发布一次速度任务完成心跳。
 * @detail: 每次任务完成都写，供紧凑记录判断超时。不启用其他统计，不改调度。
 * @param {uint32_t} time_us 任务完成时间 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskHeartbeat(uint32_t time_us) { s_speed_complete_us = time_us; }
/**
 * @description: 记录速度 PI 是否在本拍实际更新。
 * @detail: 仅节拍统计开启时计入。间隔包含跳拍和禁用前的时间。超过 1 ms 计一次超时。不改 PI。
 * @param {uint32_t} time_us 本次观察时间 (us)
 * @param {bool} updated     true=本拍 PI 序号已前进
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedPiNote(uint32_t time_us, bool updated)
{
    uint32_t key, gap;
    if (!s_cadence_active) return;
    key = Motor_Crit_Enter();
    if (updated) {
        if (s_speed_task.pi_count) {
            unsigned bin;
            gap = time_us - s_speed_task.pi_last_us;
            s_speed_task.pi_sum_gap_us += gap;
            if (gap > s_speed_task.pi_max_gap_us) {
                s_speed_task.pi_max_gap_us = gap;
                s_speed_task.worst_pi_wake = s_probe_cycle;
            }
            bin = gap <= 1000U ? 0U : gap <= 1100U ? 1U : gap <= 1250U ? 2U :
                  gap <= 1500U ? 3U : gap <= 2000U ? 4U : 5U;
            s_speed_task.pi_gap_bins[bin]++;
            if (gap > 1000U) {
                s_speed_task.pi_over_1ms++;
                s_speed_task.pi_over_streak++;
                if (s_speed_task.pi_over_streak > s_speed_task.pi_over_streak_max)
                    s_speed_task.pi_over_streak_max = s_speed_task.pi_over_streak;
            } else {
                s_speed_task.pi_over_streak = 0U;
            }
        }
        s_speed_task.pi_count++;
        s_speed_task.pi_last_us = time_us;
    } else {
        s_speed_task.pi_no_update++;
    }
    Motor_Crit_Exit(key);
}
/**
 * @description: 记录速度任务阶段切换耗时。
 * @detail: phase 1 为等待周期，2 为已唤醒，3 为位置环完成，4 为速度调用完成。
 *          只在相邻阶段前进时更新上一段峰值；复用时间戳关联同一轮1/2/3/4阶段。
 *          未开启或 phase 超出 1..4 则返回。不改调度。
 * @param {uint32_t} time_us 阶段时间 (us)
 * @param {uint32_t} tick    任务看到的 tick
 * @param {uint32_t} phase   阶段号，1..4
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskStage(uint32_t time_us, uint32_t tick, uint32_t phase)
{
    uint32_t key;
    if (!s_cadence_active || phase < 1U || phase > 4U) return;
    key = Motor_Crit_Enter();
    if (phase == 1U) s_task_phase_mask = 1U;
    else if (s_task_phase_mask == (1U << (phase - 1U)) - 1U) s_task_phase_mask |= 1U << (phase - 1U);
    else s_task_phase_mask = 0U;
    s_task_phase_stamp[phase - 1U] = time_us;
    if (s_speed_task.phase == (phase == 1U ? 4U : phase - 1U)) {
        uint32_t index = s_speed_task.phase - 1U;
        uint32_t elapsed = time_us - s_speed_task.phase_us;
        if (elapsed > s_speed_task.phase_max_us[index]) {
            s_speed_task.phase_max_us[index] = elapsed;
            s_speed_task.phase_max_ticks[index] = tick - s_speed_task.tick;
        }
    }
    s_speed_task.tick = tick;
    s_speed_task.phase = phase;
    s_speed_task.phase_us = time_us;
    Motor_Crit_Exit(key);
}
/**
 * @description: 记录任务唤醒，并发布阶段 2。
 * @detail: 比较实际 tick 和本轮 osDelayUntil 截止 tick，只保留最晚的正偏差。
 *          若探测已看到同一截止 tick，记下唤醒延迟和期间 DMA 累计。不改任务调度。
 * @param {uint32_t} time_us        唤醒时间 (us)
 * @param {uint32_t} tick           实际 tick
 * @param {uint32_t} deadline_tick  本轮截止 tick
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskWake(uint32_t time_us, uint32_t tick, uint32_t deadline_tick)
{
    uint32_t key;
    int32_t late;
    if (!s_cadence_active) return;
    key = Motor_Crit_Enter();
    s_probe_cycle.valid = s_probe_armed && s_probe_tick_seen && s_probe_deadline == deadline_tick;
    s_probe_cycle.delay_us = s_probe_cycle.valid ? Hal_Motor_TimeUs() - s_probe_start : 0U;
    s_probe_cycle.dma_count = s_probe_cycle.valid ? s_probe_dma_count : 0U;
    s_probe_cycle.dma_us = s_probe_cycle.valid ? s_probe_dma_us : 0U;
    s_probe_armed = false;
    if (s_probe_cycle.valid && g_motor_timing_split_active) {
        s_split.wake_count++;
        s_split.wake_delay_sum_us += s_probe_cycle.delay_us;
        s_split.wake_dma_sum_us += s_probe_cycle.dma_us;
        s_split.wake_full_count += s_wake_full_count;
        s_split.wake_full_sum_us += s_wake_full_us;
        s_split.wake_skip_count += s_wake_skip_count;
        s_split.wake_skip_sum_us += s_wake_skip_us;
        s_split.wake_publish_count += s_wake_publish_count;
        s_split.wake_publish_sum_us += s_wake_publish_us;
    }
    if (s_probe_cycle.valid && (!s_speed_task.worst_wake.valid ||
                               s_probe_cycle.delay_us > s_speed_task.worst_wake.delay_us)) {
        s_speed_task.worst_wake = s_probe_cycle;
        if (g_motor_timing_split_active) {
            s_split.worst_delay_us = s_probe_cycle.delay_us;
            s_split.worst_dma_us = s_probe_cycle.dma_us;
            s_split.worst_full_count = s_wake_full_count;
            s_split.worst_full_us = s_wake_full_us;
            s_split.worst_skip_count = s_wake_skip_count;
            s_split.worst_skip_us = s_wake_skip_us;
            s_split.worst_publish_count = s_wake_publish_count;
            s_split.worst_publish_us = s_wake_publish_us;
        }
    }
    Motor_Crit_Exit(key);
    Motor_Timing_SpeedTaskStage(time_us, tick, 2U);
    key = Motor_Crit_Enter();
    s_speed_task.deadline_tick = deadline_tick;
    late = (int32_t)(tick - deadline_tick);
    if (late > 0 && (uint32_t)late > s_speed_task.wake_late_max_ticks)
        s_speed_task.wake_late_max_ticks = (uint32_t)late;
    Motor_Crit_Exit(key);
}
/**
 * @description: 记录一次外环调用完成。
 * @detail: 先写心跳。节拍统计开启时更新完成间隔，并保存门控、目标、反馈和 Iq。
 *          新的最长间隔同时保存完整阶段链；阶段链不完整或总和不符时标为无效。
 *          allowed 是运行门控，不是 PI 实际更新次数。不改控制。
 * @param {uint32_t} time_us 完成时间 (us)
 * @param {bool} allowed     本拍外环是否允许输出
 * @param {float} target     速度目标 (RPM)
 * @param {float} feedback   速度反馈 (RPM)
 * @param {float} iq         速度环 Iq (A)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskNote(uint32_t time_us, bool allowed, float target, float feedback, float iq)
{
    uint32_t key, gap;
    Motor_Timing_SpeedTaskHeartbeat(time_us);
    if (!s_cadence_active) return;
    key = Motor_Crit_Enter();
    gap = time_us - s_speed_task.last_us;
    if (s_speed_task.count) {
        unsigned bin = gap <= 1000U ? 0U : gap <= 1100U ? 1U : gap <= 1250U ? 2U :
                       gap <= 1500U ? 3U : gap <= 2000U ? 4U : 5U;
        s_task_gap_bins[bin]++;
    }
    if (s_speed_task.count && gap > s_speed_task.max_gap_us) {
        s_speed_task.max_gap_us = gap;
        s_speed_task.max_gap_ticks = s_speed_task.tick - s_speed_task.complete_tick;
        s_task_worst_gap.sequence = s_speed_task.count + 1U;
        s_task_worst_gap.time_us = time_us;
        s_task_worst_gap.previous_us = s_speed_task.last_us;
        s_task_worst_gap.wake = s_probe_cycle;
        s_task_worst_gap.previous_wake = s_task_previous_wake;
        s_task_worst_gap.phase_valid = 0U;
        s_task_worst_gap.before_wait_us = 0U;
        memset(s_task_worst_gap.phase_us, 0, sizeof(s_task_worst_gap.phase_us));
        if (s_task_phase_mask == 15U) {
            uint32_t before_wait = s_task_phase_stamp[0] - s_speed_task.last_us;
            uint32_t durations[4];
            uint64_t sum = before_wait;
            unsigned index;
            for (index = 0U; index < 4U; index++) {
                durations[index] = (index == 3U ? time_us : s_task_phase_stamp[index + 1U]) - s_task_phase_stamp[index];
                sum += durations[index];
            }
            if (sum == gap && gap < 0x80000000U) {
                s_task_worst_gap.phase_valid = 1U;
                s_task_worst_gap.before_wait_us = before_wait;
                memcpy(s_task_worst_gap.phase_us, durations, sizeof(durations));
            }
        }
    }
    s_task_phase_mask = 0U;
    s_task_previous_wake = s_probe_cycle;
    s_speed_task.complete_tick = s_speed_task.tick;
    s_speed_task.count++;
    s_speed_task.last_us = time_us;
    s_speed_task.allowed = allowed;
    s_speed_task.target = target;
    s_speed_task.feedback = feedback;
    s_speed_task.iq = iq;
    Motor_Crit_Exit(key);
}
/**
 * @description: 拷贝速度任务诊断。
 * @detail: 进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_speed_task_diag_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskGet(motor_speed_task_diag_t *out)
{
    uint32_t key;
    COMMON_ASSERT(out != NULL);
    key = Motor_Crit_Enter();
    *out = s_speed_task;
    Motor_Crit_Exit(key);
}

void Motor_Timing_SpeedTaskCaptureLight(motor_trace_light_sample_t *sample)
{
    uint32_t key;
    COMMON_ASSERT(sample != NULL);
    key = Motor_Crit_Enter();
    sample->task_count = s_speed_task.count;
    sample->task_age_us = s_speed_task.count ? sample->time_us - s_speed_task.last_us : UINT32_MAX;
    sample->task_gate = s_speed_task.allowed ? 1U : 0U;
    sample->speed_target = s_speed_task.target;
    sample->speed_feedback = s_speed_task.feedback;
    sample->speed_iq = s_speed_task.iq;
    sample->task_tick = s_speed_task.tick;
    sample->task_phase = s_speed_task.phase;
    sample->task_phase_age_us = s_speed_task.phase ? sample->time_us - s_speed_task.phase_us : UINT32_MAX;
    Motor_Crit_Exit(key);
}
static motor_cadence_t s_cadence;
static uint32_t s_cadence_last;
static bool s_cadence_seed;
static motor_callback_t s_callback;
static uint32_t s_callback_start, s_callback_last;
static bool s_callback_active, s_callback_seed;
static uint32_t s_trace_entry_last;
bool s_trace_entry_seed;
/**
 * @description: 标记一次 DMA 回调开始，并统计回调入口间隔。
 * @detail: 随 CadenceReset 启用；统计和轨迹均关闭则立即返回。轨迹启用时仍保存起点并清除 full，
 *          之后若本回调执行到 CadenceNote 会再标成 true。第一次调用只建立时间基准。
 *          间隔大于 1000 us 只加 gaps。本函数不读 DMA 标志。
 * @param {uint32_t} time_us 回调入口时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackBegin(uint32_t time_us)
{
    uint32_t delta;
    unsigned bin;
    if (!s_callback_active && (!s_trace_active || s_trace_compact)) return;
    s_callback_start = time_us;
    s_callback_full = false;
    s_beat_published = false;
    s_beat_publish_us = 0U;
    if (!s_callback_active) return;
    delta = time_us - s_callback_last;
    s_callback_last = time_us;
    if (!s_callback_seed) { s_callback_seed = true; return; }
    if (delta > 1000U) { s_callback.gaps++; return; }
    s_callback.interval_count++;
    s_callback.interval_sum += delta;
    if (delta > s_callback.interval_max) s_callback.interval_max = delta;
    bin = delta <= 25U ? 0U : delta <= 40U ? 1U : delta <= 60U ? 2U :
          delta <= 75U ? 3U : delta <= 110U ? 4U : 5U;
    s_callback.bins[bin]++;
}
/**
 * @description: 结束一次 DMA 回调，按 full/skip 分开累计耗时。
 * @detail: 统计和轨迹均关闭则返回；仅轨迹启用时只更新轨迹窗口。耗时大于 1000 us 时按 1000 us 计入。
 *          否则计入 skip。over33/over66 两种回调都计。count 达到 100 万后停止。
 *          诊断仍武装时，本回调的耗时和 pending 写在 TracePush 之后，进入下一条样本的窗口。
 * @param {uint32_t} time_us 回调返回时间戳 (us)
 * @param {bool} pending     true=返回时同一 DMA 标志再次置位
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackEnd(uint32_t time_us, bool pending)
{
    uint32_t elapsed;
    if (!s_callback_active && (!s_trace_active || s_trace_compact)) return;
    elapsed = time_us - s_callback_start;
    if (elapsed > 1000U) elapsed = 1000U;
    if (s_trace_active && !s_trace_compact) {
        if (s_callback_full && elapsed > s_trace_full_max) s_trace_full_max = (uint16_t)elapsed;
        if (pending && s_trace_pending < UINT16_MAX) s_trace_pending++;
    }
    if (!s_callback_active) return;
    s_callback.count++;
    if (s_callback_full) {
        s_callback.full++;
        s_callback.full_sum += elapsed;
        if (elapsed > s_callback.full_max) s_callback.full_max = elapsed;
    } else {
        s_callback.skip_sum += elapsed;
        if (elapsed > s_callback.skip_max) s_callback.skip_max = elapsed;
    }
    if (elapsed > 33U) s_callback.over33++;
    if (elapsed > 66U) s_callback.over66++;
    if (pending) s_callback.pending++;
    if (s_beat_published) {
        s_callback.publish++;
        s_callback.publish_sum += s_beat_publish_us;
        if (s_beat_publish_us > s_callback.publish_max) s_callback.publish_max = s_beat_publish_us;
        if (s_callback_full) s_callback.publish_on_full++;
        s_split.publish_count = s_callback.publish;
        s_split.publish_sum_us = s_callback.publish_sum;
        s_split.publish_max_us = s_callback.publish_max;
        s_split.publish_on_full = s_callback.publish_on_full;
    }
    if (s_callback.count >= 1000000U) s_callback_active = false;
}
/**
 * @description: 标记本回调是完整控制拍还是跳过拍。
 * @detail: 仅细分诊断开启时由 FOC 入口调用。须在 CallbackEnd 之前。不改控制。
 * @param {bool} full true=本回调执行了完整控制
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackClassify(bool full) { s_callback_full = full; }
/**
 * @description: 记下本回调内一次状态发布的自身耗时。
 * @detail: elapsed_us 只含快照发布，不含分频计数。同一回调只记最后一次。不改控制。
 * @param {uint32_t} elapsed_us 状态发布耗时 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackPublish(uint32_t elapsed_us)
{
    s_beat_published = true;
    s_beat_publish_us = elapsed_us;
}
/**
 * @description: 拷贝 DMA 回调耗时统计。
 * @detail: 本函数进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_callback_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackGet(motor_callback_t *out)
{
    uint32_t key;
    COMMON_ASSERT(out != NULL);
    key = Motor_Crit_Enter();
    *out = s_callback;
    Motor_Crit_Exit(key);
}

/**
 * @description: 关闭 DMA 回调耗时统计。
 * @detail: 进入临界区，只清回调统计开关。已有统计保留。BSP 可据此省去两次时间戳读取。不影响 IRQ 总占用统计。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackStop(void)
{
    uint32_t key = Motor_Crit_Enter();
    s_callback_active = false;
    Motor_Crit_Exit(key);
}

/**
 * @description: 查询回调起点是否仍需记录。
 * @detail: 回调统计开启，或非紧凑轨迹仍在采集时返回 true。不改统计。
 * @return {bool} true=应调用 CallbackBegin
 * @author: lkc
 */
bool Motor_Timing_CallbackIsActive(void) { return s_callback_active || (s_trace_active && !s_trace_compact); }

/**
 * @description: 清零节拍、回调和分段耗时窗口，并重新允许采集。
 * @detail: 临界区内清零。不清环形诊断，也不清上一次时间基准；之后第一次 Note/CallbackBegin 丢掉首个间隔。
 *          不修改算法 dt。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CadenceReset(void)
{
    uint32_t key = Motor_Crit_Enter();
    g_motor_schedule_debug.state = 0U;
    s_probe_enabled = false;
    s_probe_armed = false;
    memset(&s_probe_cycle, 0, sizeof(s_probe_cycle));
    s_task_only = false;
    s_performance_enabled = true;
    memset(&s_cadence, 0, sizeof(s_cadence));
    memset(&s_speed_task, 0, sizeof(s_speed_task));
    memset((void *)&g_motor_task_runtime_debug, 0, sizeof(g_motor_task_runtime_debug));
    s_task_runtime_active = false;
    memset(&s_task_previous_wake, 0, sizeof(s_task_previous_wake));
    memset(s_task_gap_bins, 0, sizeof(s_task_gap_bins));
    memset(&s_task_worst_gap, 0, sizeof(s_task_worst_gap));
    s_task_phase_mask = 0U;
    memset(s_task_phase_stamp, 0, sizeof(s_task_phase_stamp));
    memset(s_irq_diag, 0, sizeof(s_irq_diag));
    memset(s_irq_divider, 0, sizeof(s_irq_divider));
    memset((void *)s_profile_sum, 0, sizeof(s_profile_sum));
    memset((void *)s_profile_count, 0, sizeof(s_profile_count));
    memset((void *)s_profile_max, 0, sizeof(s_profile_max));
    s_cadence_seed = false;
    s_cadence_active = true;
    memset(&s_callback, 0, sizeof(s_callback));
    s_callback_seed = false;
    s_callback_active = true;
    g_motor_timing_split_active = 0U;
    memset(&s_split, 0, sizeof(s_split));
    Motor_Crit_Exit(key);
}
/**
 * @description: 打开完整拍、跳过拍和状态发布的同窗口细分，并武装唤醒重叠。
 * @detail: 关闭性能统计和分段采样，保留回调耗时与速度任务节拍。不改控制周期、增益或任务优先级。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SplitReset(void)
{
    uint32_t key;
    Motor_Timing_CadenceReset();
    key = Motor_Crit_Enter();
    s_task_only = true;
    s_performance_enabled = false;
    s_profile_active = false;
    s_callback_active = true;
    s_probe_enabled = true;
    g_motor_timing_split_active = 1U;
    Motor_Crit_Exit(key);
}
/**
 * @description: 拷贝拍型细分与唤醒重叠。
 * @detail: 进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_split_diag_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SplitGet(motor_split_diag_t *out)
{
    uint32_t key;
    COMMON_ASSERT(out != NULL);
    key = Motor_Crit_Enter();
    *out = s_split;
    Motor_Crit_Exit(key);
}
/**
 * @description: 清零并只开启速度任务统计。
 * @detail: 先做 CadenceReset，再关闭回调统计、性能统计和分段采样。不改变轨迹模式或控制参数。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskReset(void)
{
    uint32_t key = Motor_Crit_Enter();
    Motor_Timing_CadenceReset();
    s_task_only = true;
    s_callback_active = false;
    s_performance_enabled = false;
    s_profile_active = false;
    Motor_Crit_Exit(key);
}
/**
 * @description: 打开目标 tick 到任务唤醒的 DMA 探测。
 * @detail: 先做 SpeedTaskReset，再允许 WakeProbeArm。探测是插桩，不改任务调度。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_WakeProbeReset(void)
{
    uint32_t key = Motor_Crit_Enter();
    Motor_Timing_SpeedTaskReset();
    s_probe_enabled = true;
    Motor_Crit_Exit(key);
}
/**
 * @description: 停止节拍和 DMA 回调累加。
 * @detail: 临界区内关闭性能统计、分段采样、唤醒探测、节拍和回调累加。已有统计和环形诊断保留。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CadenceStop(void)
{
    uint32_t key = Motor_Crit_Enter();
    s_performance_enabled = false;
    s_profile_active = false;
    s_probe_enabled = false;
    s_probe_armed = false;
    s_cadence_active = false;
    s_callback_active = false;
    g_motor_timing_split_active = 0U;
    Motor_Crit_Exit(key);
}

/**
 * @description: 在停机命令任务中冻结调度统计，发布只读 RAM 快照。
 * @detail: 复用已有计数，不增加 ISR 插桩；短临界区内关闭采集并复制，发布前后使用序号和内存屏障。
 * @return {void} 无
 */
void Motor_Timing_ScheduleDebugFreeze(void)
{
    unsigned index;
    uint32_t key = Motor_Crit_Enter();
    if (g_motor_schedule_debug.state == 2U) {
        Motor_Crit_Exit(key);
        return;
    }
    g_motor_schedule_debug.sequence++;
#if MOTOR_CRIT_HAS_IRQ_LOCK
    __DMB();
#endif
    g_motor_schedule_debug.state = 0U;
    g_motor_schedule_debug.mode = (s_task_only ? 1U : 0U) | (s_probe_enabled ? 2U : 0U) |
                                 (s_performance_enabled ? 4U : 0U) | (s_cadence_active ? 8U : 0U);
    Motor_Timing_CadenceStop();
    g_motor_schedule_debug.task_count = s_speed_task.count;
    g_motor_schedule_debug.task_max_gap_us = s_speed_task.max_gap_us;
    g_motor_schedule_debug.pi_count = s_speed_task.pi_count;
    g_motor_schedule_debug.pi_no_update = s_speed_task.pi_no_update;
    g_motor_schedule_debug.pi_max_gap_us = s_speed_task.pi_max_gap_us;
    g_motor_schedule_debug.pi_over_1ms = s_speed_task.pi_over_1ms;
    g_motor_schedule_debug.pi_streak_max = s_speed_task.pi_over_streak_max;
    g_motor_schedule_debug.wake_late_max_ticks = s_speed_task.wake_late_max_ticks;
    for (index = 0U; index < 4U; index++)
        g_motor_schedule_debug.phase_max_us[index] = s_speed_task.phase_max_us[index];
    for (index = 0U; index < 6U; index++)
        g_motor_schedule_debug.pi_gap_bins[index] = s_speed_task.pi_gap_bins[index];
    g_motor_schedule_debug.worst_pi_wake = s_speed_task.worst_pi_wake;
    g_motor_schedule_debug.worst_wake = s_speed_task.worst_wake;
    for (index = 0U; index < 6U; index++)
        g_motor_schedule_debug.task_gap_bins[index] = s_task_gap_bins[index];
    g_motor_schedule_debug.worst_gap = s_task_worst_gap;
    for (index = 0U; index < 2U; index++) {
        const motor_irq_diag_t *irq = &s_irq_diag[index];
        g_motor_schedule_debug.irq[index][0] = irq->seen;
        g_motor_schedule_debug.irq[index][1] = irq->count;
        g_motor_schedule_debug.irq[index][2] = irq->sum_us;
        g_motor_schedule_debug.irq[index][3] = irq->max_us;
        g_motor_schedule_debug.irq[index][4] = irq->first_us;
        g_motor_schedule_debug.irq[index][5] = irq->last_us;
        g_motor_schedule_debug.irq[index][6] = irq->tick_pending;
        g_motor_schedule_debug.irq[index][7] = irq->switch_pending;
    }
    g_motor_schedule_debug.state = 2U;
#if MOTOR_CRIT_HAS_IRQ_LOCK
    __DMB();
#endif
    g_motor_schedule_debug.sequence++;
    Motor_Crit_Exit(key);
}
/**
 * @description: 标记本回调为完整控制拍，并记录控制入口间隔。
 * @detail: 无论节拍统计是否启用，都先把回调标成 full，并在诊断武装时更新入口间隔峰值和向上计数。
 *          节拍未启用则随后返回。启用后第一次调用只建立时间基准。间隔大于 1000 us 只加 gaps。
 *          有效间隔达到 100 万后停止，该次间隔不计入。upward 为真表示入口处 TIM1 DIR 为向上计数。
 * @param {uint32_t} time_us 控制入口时间戳 (us)
 * @param {bool} upward      true=入口处 DIR 读数为向上计数
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CadenceNote(uint32_t time_us, bool upward)
{
    uint32_t delta;
    unsigned bin;
    s_callback_full = true;
    if (s_trace_active && !s_trace_compact) {
        uint32_t trace_delta = time_us - s_trace_entry_last;
        s_trace_entry_last = time_us;
        if (s_trace_entry_seed && trace_delta > s_trace_gap_max)
            s_trace_gap_max = (uint16_t)(trace_delta > UINT16_MAX ? UINT16_MAX : trace_delta);
        s_trace_entry_seed = true;
        if (upward && s_trace_up < UINT16_MAX) s_trace_up++;
    }
    if (!s_cadence_active || s_task_only) return;
    delta = time_us - s_cadence_last;
    s_cadence_last = time_us;
    if (!s_cadence_seed) { s_cadence_seed = true; return; }
    if (delta > 1000U) { s_cadence.gaps++; return; }
    if (s_cadence.count >= 1000000U) { s_cadence_active = false; return; }
    if (s_cadence.count == 0U || delta < s_cadence.min_us) s_cadence.min_us = delta;
    if (delta > s_cadence.max_us) s_cadence.max_us = delta;
    s_cadence.count++; s_cadence.sum_us += delta;
    if (upward) s_cadence.up++; else s_cadence.down++;
    bin = delta <= 50U ? 0U : delta <= 60U ? 1U : delta <= 73U ? 2U : delta <= 90U ? 3U :
          delta <= 110U ? 4U : delta <= 150U ? 5U : delta <= 250U ? 6U : 7U;
    s_cadence.bins[bin]++;
}
/**
 * @description: 拷贝控制入口节拍统计。
 * @detail: 本函数进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_cadence_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CadenceGet(motor_cadence_t *out)
{
    uint32_t key;
    COMMON_ASSERT(out != NULL);
    key = Motor_Crit_Enter();
    *out = s_cadence;
    Motor_Crit_Exit(key);
}
#endif /* MOTOR_CFG_DIAG */
