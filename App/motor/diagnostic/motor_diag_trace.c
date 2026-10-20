/*
 * @Description: 无感运行轨迹与故障冻结轨迹记录
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_timing.h"
#include "motor_timing_internal.h"
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
/* 记录格式互斥，共用已有存储，不新增环形缓冲区。 */
static union {
    motor_trace_sample_t full[MOTOR_TRACE_CAPACITY];
    motor_trace_light_sample_t light[MOTOR_TRACE_CAPACITY];
    motor_trace_compact_sample_t compact[MOTOR_TRACE_COMPACT_CAPACITY];
    motor_fault_trace_sample_t fault[MOTOR_TRACE_CAPACITY];
} s_trace_storage;
typedef char motor_trace_compact_size_check_t[sizeof(motor_trace_compact_sample_t) == 32U ? 1 : -1];
typedef char motor_trace_storage_size_check_t[
    sizeof(s_trace_storage) == sizeof(motor_trace_sample_t) * MOTOR_TRACE_CAPACITY ? 1 : -1];
#define s_trace (s_trace_storage.full)
volatile motor_fault_debug_descriptor_t g_motor_fault_debug = {
    MOTOR_FAULT_DEBUG_MAGIC, MOTOR_FAULT_DEBUG_VERSION, 0U, MOTOR_FAULT_DEBUG_INVALID,
    0U, sizeof(motor_fault_trace_sample_t), MOTOR_TRACE_CAPACITY, MOTOR_FAULT_TRACE_STRIDE,
    0U, 0U, 0U, 0U, 0U, sizeof(motor_fault_debug_descriptor_t),
};
typedef char motor_fault_debug_size_check_t[sizeof(motor_fault_debug_descriptor_t) == 56U ? 1 : -1];
typedef char motor_fault_sample_size_check_t[sizeof(motor_fault_trace_sample_t) == 84U ? 1 : -1];
static unsigned s_trace_head, s_trace_count, s_trace_divider, s_trace_post;
static bool s_trace_closed, s_trace_trigger;
bool s_trace_active;
static bool s_trace_entry_mode, s_trace_entry_trigger;
static bool s_trace_entry_fast;
static bool s_trace_light;
static bool s_compact_running, s_compact_stop;
bool s_trace_compact;
bool s_trace_fault;
static uint32_t s_fault_trace_last_us;
static bool s_fault_trace_seeded;
static uint32_t s_compact_flags, s_compact_trigger_flags;
static bool s_trace_flight;
#define MOTOR_TRACE_FLIGHT_ENTRY 192U
#define MOTOR_TRACE_FLIGHT_RING (MOTOR_TRACE_CAPACITY - MOTOR_TRACE_FLIGHT_ENTRY)
static unsigned s_flight_entry_count, s_flight_head, s_flight_count;
uint16_t s_trace_gap_max, s_trace_full_max, s_trace_pending, s_trace_up;

/**
 * @description: 在武装、冻结或复用缓冲时发布调试器描述符。
 * @detail: 短临界区串行化写者，奇偶序号与内存屏障保护后台读者；不在逐样本Push中调用。
 * @param {uint32_t} state 无效、武装或冻结
 * @param {unsigned} fault 冻结故障码，非故障状态传0
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Timing_FaultDebugPublish(uint32_t state, unsigned fault)
{
    uint32_t key = Motor_Crit_Enter();
    g_motor_fault_debug.sequence++;
#if MOTOR_CRIT_HAS_IRQ_LOCK
    __DMB();
#endif
    g_motor_fault_debug.state = MOTOR_FAULT_DEBUG_INVALID;
    g_motor_fault_debug.address = (uint32_t)(uintptr_t)s_trace_storage.fault;
    g_motor_fault_debug.head = state == MOTOR_FAULT_DEBUG_FROZEN ? s_trace_head : 0U;
    g_motor_fault_debug.count = state == MOTOR_FAULT_DEBUG_FROZEN ? s_trace_count : 0U;
    g_motor_fault_debug.fault = fault & 0xffffU;
    g_motor_fault_debug.first_us = 0U;
    g_motor_fault_debug.last_us = 0U;
    if (state == MOTOR_FAULT_DEBUG_FROZEN && s_trace_count != 0U) {
        unsigned first = (s_trace_head + MOTOR_TRACE_CAPACITY - s_trace_count) % MOTOR_TRACE_CAPACITY;
        unsigned last = (s_trace_head + MOTOR_TRACE_CAPACITY - 1U) % MOTOR_TRACE_CAPACITY;
        g_motor_fault_debug.first_us = s_trace_storage.fault[first].time_us;
        g_motor_fault_debug.last_us = s_trace_storage.fault[last].time_us;
    }
    g_motor_fault_debug.state = state;
#if MOTOR_CRIT_HAS_IRQ_LOCK
    __DMB();
#endif
    g_motor_fault_debug.sequence++;
    Motor_Crit_Exit(key);
}

/**
 * @description: 复位环形诊断索引并开始记录。
 * @detail: 先失效旧调试描述符，再清下标与触发状态，不擦除数组，不检查电机是否运行。
 * @return {void} 无
 */
void Motor_Timing_TraceArm(void)
{
    Motor_Timing_FaultDebugPublish(MOTOR_FAULT_DEBUG_INVALID, 0U);
    s_trace_fault = false;
    s_trace_compact = false;
    s_performance_enabled = true;
    s_trace_head = s_trace_count = s_trace_divider = s_trace_post = 0U;
    s_trace_closed = s_trace_trigger = false;
    s_trace_entry_mode = s_trace_entry_trigger = false;
    s_trace_entry_fast = false;
    s_trace_light = false;
    s_trace_flight = false;
    s_flight_entry_count = s_flight_head = s_flight_count = 0U;
    s_trace_entry_seed = false;
    s_trace_gap_max = s_trace_full_max = s_trace_pending = s_trace_up = 0U;
    s_trace_active = true;
}

/**
 * @description: 停机时武装紧凑飞行记录。
 * @detail: 复用完整缓冲，每条 32 字节。默认关闭性能统计，CadenceReset 可再打开。
 *          正常每 30 个控制拍写一条。首次 CLOSED 当拍做标记；失锁、故障或任务超过 10 ms 后再留 16 条。
 *          停机立即冻结。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArmCompact(void)
{
    uint32_t key = Motor_Crit_Enter();
    Motor_Timing_TraceArm();
    Motor_Timing_CadenceStop();
    s_performance_enabled = s_profile_active = false;
    s_trace_compact = true;
    s_compact_running = s_compact_stop = false;
    s_compact_flags = s_compact_trigger_flags = 0U;
    Motor_Crit_Exit(key);
}
/**
 * @description: 查询当前记录格式是否为紧凑轨迹。
 * @detail: 故障轨迹复用同一缓冲，此时返回 false。
 * @return {bool} true=紧凑格式且不是故障轨迹
 * @author: lkc
 */
bool Motor_Timing_TraceIsCompact(void) { return s_trace_compact && !s_trace_fault; }
/**
 * @description: 查询紧凑轨迹是否仍在采集。
 * @return {bool} true=紧凑格式且未冻结
 * @author: lkc
 */
bool Motor_Timing_TraceCompactIsActive(void) { return s_trace_compact && !s_trace_fault && s_trace_active; }

/**
 * @description: 停机任务里武装故障前分频环形记录。
 * @detail: 复用完整轨迹 RAM，并暂停性能统计。已有故障须由应用拒绝武装，本函数不清除故障。
 *          采集期间不要从任务读取或切换模式。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_FaultTraceArm(void)
{
    uint32_t key = Motor_Crit_Enter();
    Motor_Timing_TraceArmCompact();
    s_trace_fault = true;
    s_fault_trace_seeded = false;
    Motor_Timing_FaultDebugPublish(MOTOR_FAULT_DEBUG_ARMED, 0U);
    Motor_Crit_Exit(key);
}
/**
 * @description: 查询当前记录格式是否为故障轨迹。
 * @return {bool} true=故障轨迹格式
 * @author: lkc
 */
bool Motor_Timing_TraceIsFault(void) { return s_trace_fault; }
/**
 * @description: 查询故障轨迹是否仍在采集。
 * @detail: ISR 写者用它检查采集开关。停机任务只宜用它判断格式是否活动，不要读取活动数组。
 * @return {bool} true=故障格式且未冻结
 * @author: lkc
 */
bool Motor_Timing_FaultTraceIsActive(void) { return s_trace_fault && s_trace_active; }
/**
 * @description: 判断本控制入口是否写入一条故障轨迹。
 * @detail: ISR 采样前调用。普通情况每 4 拍放行一次。force 为真时强制本拍，同一入口时间不重复。
 *          running 为 false 且已有样本时冻结并返回 false，不采集停机数据。time_us 是控制入口 (us)。
 * @param {uint32_t} time_us 控制入口时间 (us)
 * @param {bool} running     本拍控制是否仍在运行
 * @param {bool} force       true=故障拍强制采集
 * @return {bool} true=本拍应 Push
 * @author: lkc
 */
bool Motor_Timing_FaultTraceDue(uint32_t time_us, bool running, bool force)
{
    if (!Motor_Timing_FaultTraceIsActive()) return false;
    if (!running) {
        if (s_trace_count != 0U) Motor_Timing_FaultTraceFreeze(0U);
        return false;
    }
    if (s_fault_trace_seeded && s_fault_trace_last_us == time_us) return false;
    if (!force && ++s_trace_divider < MOTOR_FAULT_TRACE_STRIDE) return false;
    s_trace_divider = 0U;
    s_fault_trace_last_us = time_us;
    s_fault_trace_seeded = true;
    return true;
}
/**
 * @description: 复制一条故障轨迹样本。
 * @detail: 只在已武装且未冻结时写入。sample 不可为空，违反约定触发 COMMON_ASSERT。故障冻结后不覆盖。
 *          采样入口的 ADC、abc、母线电压和 dt 是本拍值；角度、dq、积分和观测量由调用者填入。
 * @param {const motor_fault_trace_sample_t *} sample 本拍样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_FaultTracePush(const motor_fault_trace_sample_t *sample)
{
    COMMON_ASSERT(sample != NULL);
    if (!Motor_Timing_FaultTraceIsActive()) return;
    s_trace_storage.fault[s_trace_head] = *sample;
    if (++s_trace_head >= MOTOR_TRACE_CAPACITY) s_trace_head = 0U;
    if (s_trace_count < MOTOR_TRACE_CAPACITY) s_trace_count++;
}
/**
 * @description: 冻结故障轨迹，并给最后一条标故障码。
 * @detail: ISR 在故障处理复位前调用。fault 非 0 时写入最后一条 flags 的高 16 位。
 *          停机任务只在已经停机时传 0，只冻结、不改最后一条。未在采集则返回。
 * @param {unsigned} fault 故障码；0 表示只冻结
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_FaultTraceFreeze(unsigned fault)
{
    if (!Motor_Timing_FaultTraceIsActive()) return;
    if (fault != 0U && s_trace_count != 0U) {
        unsigned last = (s_trace_head + MOTOR_TRACE_CAPACITY - 1U) % MOTOR_TRACE_CAPACITY;
        s_trace_storage.fault[last].flags |= (fault & 0xffffU) << 16;
    }
    s_trace_active = false;
    Motor_Timing_FaultDebugPublish(MOTOR_FAULT_DEBUG_FROZEN, fault);
}
/**
 * @description: 按时间顺序读取一条已冻结的故障轨迹。
 * @detail: 未冻结、不是故障格式、sample 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index                     时间顺序下标
 * @param {motor_fault_trace_sample_t *} sample 输出；空则失败
 * @return {bool} true=已写入 sample
 * @author: lkc
 */
bool Motor_Timing_FaultTraceGet(unsigned index, motor_fault_trace_sample_t *sample)
{
    unsigned first;
    if (!s_trace_fault || s_trace_active || sample == NULL || index >= s_trace_count) return false;
    first = (s_trace_head + MOTOR_TRACE_CAPACITY - s_trace_count) % MOTOR_TRACE_CAPACITY;
    *sample = s_trace_storage.fault[(first + index) % MOTOR_TRACE_CAPACITY];
    return true;
}

/**
 * @description: 判断本控制拍是否写入一条紧凑轨迹。
 * @detail: 未运行且无故障时不采集。首次 CLOSED 当拍做标记。失锁、故障或任务心跳超过 10 ms 后留 16 条。
 *          停机立即放行并在 Push 后冻结。time_us 为控制入口 (us)。不改控制。
 * @param {uint32_t} time_us 控制入口时间 (us)
 * @param {unsigned} state   观测器状态，2 为 CLOSED
 * @param {unsigned} fault   故障码，0 为无故障
 * @param {bool} running     本拍控制是否仍在运行
 * @return {bool} true=本拍应 Push
 * @author: lkc
 */
bool Motor_Timing_TraceCompactDue(uint32_t time_us, unsigned state, unsigned fault, bool running)
{
    uint32_t events = 0U;
    if (!Motor_Timing_TraceCompactIsActive()) return false;
    if (running) s_compact_running = true;
    if (!running && !s_compact_running && fault == 0U) return false;
    if (state == 2U && !s_trace_closed) events |= MOTOR_TRACE_COMPACT_ENTRY;
    if (running && s_trace_closed && state != 2U) events |= MOTOR_TRACE_COMPACT_LOSS;
    if (state == 2U) s_trace_closed = true;
    if (fault != 0U) events |= MOTOR_TRACE_COMPACT_FAULT;
    if (running && s_speed_complete_us != 0U &&
        time_us - s_speed_complete_us > MOTOR_TRACE_COMPACT_TASK_TIMEOUT_US)
        events |= MOTOR_TRACE_COMPACT_TASK_GAP;
    if (!running) {
        events |= MOTOR_TRACE_COMPACT_STOP;
        s_compact_stop = true;
    }
    if (!s_trace_trigger && (events & (MOTOR_TRACE_COMPACT_LOSS | MOTOR_TRACE_COMPACT_FAULT |
                                     MOTOR_TRACE_COMPACT_TASK_GAP)) != 0U) {
        s_trace_trigger = true;
        s_trace_post = MOTOR_TRACE_COMPACT_POST;
        s_compact_trigger_flags = events;
    } else {
        events &= MOTOR_TRACE_COMPACT_ENTRY | MOTOR_TRACE_COMPACT_STOP;
    }
    s_trace_divider++;
    if (events == 0U && s_trace_divider < MOTOR_TRACE_COMPACT_STRIDE) return false;
    s_trace_divider = 0U;
    s_compact_flags = (state & 3U) | ((fault & 0xffffU) << 16) | events | s_compact_trigger_flags;
    return true;
}
/**
 * @description: 写入一条紧凑轨迹，并附上本拍事件标志。
 * @detail: sample 不可为空，违反约定触发 COMMON_ASSERT。未在采集则返回。
 *          停机或尾段配额用完后清除采集开关。不改控制。
 * @param {const motor_trace_compact_sample_t *} sample 本拍样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TracePushCompact(const motor_trace_compact_sample_t *sample)
{
    motor_trace_compact_sample_t *destination;
    COMMON_ASSERT(sample != NULL);
    if (!Motor_Timing_TraceCompactIsActive()) return;
    destination = &s_trace_storage.compact[s_trace_head];
    *destination = *sample;
    destination->flags = s_compact_flags;
    if (++s_trace_head >= MOTOR_TRACE_COMPACT_CAPACITY) s_trace_head = 0U;
    if (s_trace_count < MOTOR_TRACE_COMPACT_CAPACITY) s_trace_count++;
    if (s_compact_stop || (s_trace_trigger && s_trace_post != 0U && --s_trace_post == 0U))
        s_trace_active = false;
}
/**
 * @description: 按时间顺序读取一条已冻结的紧凑轨迹。
 * @detail: 仍在采集、不是紧凑格式、sample 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index                       时间顺序下标
 * @param {motor_trace_compact_sample_t *} sample 输出；空则失败
 * @return {bool} true=已写入 sample
 * @author: lkc
 */
bool Motor_Timing_TraceGetCompact(unsigned index, motor_trace_compact_sample_t *sample)
{
    unsigned first;
    if (s_trace_active || !s_trace_compact || s_trace_fault || sample == NULL || index >= s_trace_count) return false;
    first = (s_trace_head + MOTOR_TRACE_COMPACT_CAPACITY - s_trace_count) % MOTOR_TRACE_COMPACT_CAPACITY;
    *sample = s_trace_storage.compact[(first + index) % MOTOR_TRACE_COMPACT_CAPACITY];
    return true;
}
/**
 * @description: 按接管触发方式武装环形诊断。
 * @detail: 先做与 TraceArm 相同的复位，再改为首次 state==2 时触发。触发后连同当拍再收 384 条并停止。
 *          容量 512，缓冲区已经记满时，触发后大约还留 128 条触发前样本。本函数不停止电机。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArmEntry(void)
{
    Motor_Timing_TraceArm();
    s_trace_entry_mode = true;
}
/**
 * @description: 首次 CLOSED 后逐控制拍记录 384 条。
 * @detail: 在接管触发方式上改为触发后每拍放行。收满后冻结。不停止电机，不改控制节拍。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArmEntryFast(void)
{
    Motor_Timing_TraceArmEntry();
    s_trace_entry_fast = true;
}
/**
 * @description: 按接管触发方式武装轻量轨迹。
 * @detail: 样本比完整记录短，仍复用同一缓冲。触发后每 8 拍记录，配额 384 条。不停止电机。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArmLight(void)
{
    Motor_Timing_TraceArmEntry();
    s_trace_light = true;
}
/**
 * @description: 查询当前记录格式是否为轻量轨迹。
 * @return {bool} true=轻量格式
 * @author: lkc
 */
bool Motor_Timing_TraceIsLight(void) { return s_trace_light; }
/**
 * @description: 武装飞行记录，复用轻量缓冲。
 * @detail: 首次接管保留 192 条，其余 320 条循环覆盖。离开 CLOSED 的当拍冻结。不停止电机。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArmFlight(void)
{
    Motor_Timing_TraceArmLight();
    s_trace_flight = true;
}
/**
 * @description: 查询当前记录是否为飞行格式。
 * @return {bool} true=飞行记录
 * @author: lkc
 */
bool Motor_Timing_TraceIsFlight(void) { return s_trace_flight; }
/**
 * @description: 返回飞行记录里固定保留的接管条数。
 * @detail: 最多 192。循环区条数不在这里。
 * @return {unsigned} 固定区条数
 * @author: lkc
 */
unsigned Motor_Timing_TraceFlightEntryCount(void) { return s_flight_entry_count; }
/**
 * @description: 停止继续写入环形诊断。
 * @detail: 只清除 active。不检查停机，不清样本。停止后 Get 可以读取；收满触发后样本也会自行清除 active。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceStop(void)
{
    uint32_t key = Motor_Crit_Enter();
    if (s_trace_fault) Motor_Timing_FaultTraceFreeze(0U);
    else s_trace_active = false;
    Motor_Crit_Exit(key);
}
/**
 * @description: 返回当前已写入、尚未被新数据覆盖的条数。
 * @detail: 仍在记录时也会增长，最大为 MOTOR_TRACE_CAPACITY。条数不等于 Get 已经允许读取。
 * @return {unsigned} 条数
 * @author: lkc
 */
unsigned Motor_Timing_TraceCount(void)
{
    return s_trace_flight ? s_flight_entry_count + s_flight_count : s_trace_count;
}
/**
 * @description: 判断这次调用是否应写入一条环形诊断。
 * @detail: active 为 false 时返回 false。state==2 表示已经到过 CLOSED，这个标记在本次事件判断之后才更新。
 *          普通武装：此前到过 CLOSED 且本次 state 不是 2，当拍触发，并把后续配额设为 16。
 *          entry 武装：首次 state==2 当拍触发，并把后续配额设为 384。
 *          没有触发时每 30 次调用返回一次 true；有触发时当拍返回 true，并把计数清零。
 *          快速接管触发后改为逐拍。轻量触发后每 8 拍。飞行记录满 192 条固定区后每 128 拍。
 *          紧凑格式不走本函数。
 * @param {unsigned} state 观测器状态，2 为 CLOSED
 * @return {bool} true=本拍应 Push
 * @author: lkc
 */
bool Motor_Timing_TraceDue(unsigned state)
{
    bool event;
    if (!s_trace_active || s_trace_compact) return false;
    event = !s_trace_trigger && s_trace_closed && state != 2U;
    if (s_trace_entry_mode && !s_trace_entry_trigger && state == 2U) {
        s_trace_entry_trigger = true;
        s_trace_post = 384U;
        /* 首次接管以前的滚动样本不与接管固定窗口重复导出。 */
        if (s_trace_flight) s_flight_head = s_flight_count = 0U;
        event = true;
    } else if (event) {
        s_trace_trigger = true;
        if (!s_trace_entry_trigger) s_trace_post = 16U;
    }
    if (state == 2U) s_trace_closed = true;
    s_trace_divider++;
    if (!event) {
        unsigned divider = s_trace_entry_fast && s_trace_entry_trigger ? 1U :
                           (s_trace_light && s_trace_entry_trigger ? 8U : 30U);
        if (s_trace_flight && s_trace_entry_trigger && s_flight_entry_count >= MOTOR_TRACE_FLIGHT_ENTRY)
            divider = 128U; /* 320条覆盖约2.7s，避免只看到失锁计时的尾端。 */
        if (s_trace_divider < divider) return false;
    }
    s_trace_divider = 0U;
    return true;
}
/**
 * @description: 写入一条环形诊断，并附上当前窗口的间隔统计。
 * @detail: sample 不可为空，违反约定触发 COMMON_ASSERT。未在采集或当前是轻量/紧凑格式时返回。
 *          写入后用窗口里的 gap/full/pending/up 覆盖样本对应字段并清窗口。
 *          失锁触发给 flags 加 bit7(128)，接管触发加 bit8(256)。配额减到 0 时清除 active。
 * @param {const motor_trace_sample_t *} sample 本拍样本；空指针直接返回
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TracePush(const motor_trace_sample_t *sample)
{
    motor_trace_sample_t *destination;
    COMMON_ASSERT(sample != NULL);
    if (!s_trace_active || s_trace_light || s_trace_compact) return;
    destination = &s_trace[s_trace_head];
    *destination = *sample;
    destination->gap_max_us = s_trace_gap_max;
    destination->full_max_us = s_trace_full_max;
    destination->pending_count = s_trace_pending;
    destination->up_count = s_trace_up;
    s_trace_gap_max = s_trace_full_max = s_trace_pending = s_trace_up = 0U;
    if (s_trace_trigger) destination->flags |= 128U;
    if (s_trace_entry_trigger) destination->flags |= 256U;
    s_trace_head = (s_trace_head + 1U) % MOTOR_TRACE_CAPACITY;
    if (s_trace_count < MOTOR_TRACE_CAPACITY) s_trace_count++;
    if ((s_trace_trigger || s_trace_entry_trigger) && s_trace_post && --s_trace_post == 0U) s_trace_active = false;
}
/**
 * @description: 按时间顺序读取一条已经停止写入的环形诊断。
 * @detail: active 仍为 true、sample 为空或 index 越界时返回 false。停止可以来自 TraceStop，也可以来自配额用完。
 *          下标 0 是当前缓冲区里最旧的一条。本函数不输出串口。
 * @param {unsigned} index                时间顺序下标，0 为最旧
 * @param {motor_trace_sample_t *} sample 输出，不可为空
 * @return {bool} true=已写入 sample
 * @author: lkc
 */
bool Motor_Timing_TraceGet(unsigned index, motor_trace_sample_t *sample)
{
    unsigned first;
    if (s_trace_active || s_trace_light || s_trace_compact || sample == NULL || index >= s_trace_count) return false;
    first = (s_trace_head + MOTOR_TRACE_CAPACITY - s_trace_count) % MOTOR_TRACE_CAPACITY;
    *sample = s_trace[(first + index) % MOTOR_TRACE_CAPACITY];
    return true;
}

/**
 * @description: 写入一条轻量或飞行轨迹。
 * @detail: sample 不可为空，违反约定触发 COMMON_ASSERT。未武装则返回。
 *          飞行格式先填满 192 条固定区，再在其余 320 条里循环；离开 CLOSED 的当拍冻结。
 *          普通轻量格式按配额停止。不改控制。
 * @param {const motor_trace_light_sample_t *} sample 本拍样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TracePushLight(const motor_trace_light_sample_t *sample)
{
    motor_trace_light_sample_t *destination;
    COMMON_ASSERT(sample != NULL);
    if (!s_trace_active || !s_trace_light) return;
    if (s_trace_flight) {
        if (s_trace_entry_trigger && s_flight_entry_count < MOTOR_TRACE_FLIGHT_ENTRY) {
            destination = &s_trace_storage.light[s_flight_entry_count++];
        } else {
            destination = &s_trace_storage.light[MOTOR_TRACE_FLIGHT_ENTRY + s_flight_head];
            s_flight_head = (s_flight_head + 1U) % MOTOR_TRACE_FLIGHT_RING;
            if (s_flight_count < MOTOR_TRACE_FLIGHT_RING) s_flight_count++;
        }
    } else destination = &s_trace_storage.light[s_trace_head];
    *destination = *sample;
    destination->gap_max_us = s_trace_gap_max;
    destination->full_max_us = s_trace_full_max;
    s_trace_gap_max = s_trace_full_max = s_trace_pending = s_trace_up = 0U;
    if (s_trace_trigger) destination->flags |= 128U;
    if (s_trace_entry_trigger) destination->flags |= 256U;
    if (s_trace_flight) {
        /* ISR状态回退保留当拍后立即冻结；命令守卫由TraceStop在关桥前冻结。 */
        if (s_trace_trigger) s_trace_active = false;
        return;
    }
    s_trace_head = (s_trace_head + 1U) % MOTOR_TRACE_CAPACITY;
    if (s_trace_count < MOTOR_TRACE_CAPACITY) s_trace_count++;
    if ((s_trace_trigger || s_trace_entry_trigger) && s_trace_post && --s_trace_post == 0U) s_trace_active = false;
}

/**
 * @description: 按时间顺序读取一条已停止的轻量或飞行轨迹。
 * @detail: 仍在采集、不是轻量格式、sample 为空或下标越界时返回 false。下标 0 为最旧。
 *          飞行格式先读固定区，再读循环区。
 * @param {unsigned} index                     时间顺序下标
 * @param {motor_trace_light_sample_t *} sample 输出；空则失败
 * @return {bool} true=已写入 sample
 * @author: lkc
 */
bool Motor_Timing_TraceGetLight(unsigned index, motor_trace_light_sample_t *sample)
{
    unsigned first;
    if (s_trace_active || !s_trace_light || sample == NULL || index >= Motor_Timing_TraceCount()) return false;
    if (s_trace_flight) {
        if (index < s_flight_entry_count) *sample = s_trace_storage.light[index];
        else {
            first = (s_flight_head + MOTOR_TRACE_FLIGHT_RING - s_flight_count) % MOTOR_TRACE_FLIGHT_RING;
            *sample = s_trace_storage.light[MOTOR_TRACE_FLIGHT_ENTRY +
                       (first + index - s_flight_entry_count) % MOTOR_TRACE_FLIGHT_RING];
        }
        return true;
    }
    first = (s_trace_head + MOTOR_TRACE_CAPACITY - s_trace_count) % MOTOR_TRACE_CAPACITY;
    *sample = s_trace_storage.light[(first + index) % MOTOR_TRACE_CAPACITY];
    return true;
}
