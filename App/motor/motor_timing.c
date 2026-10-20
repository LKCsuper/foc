/*
 * @Description: ADC/控制周期计算与 ADC-DMA 中断耗时统计
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_timing.h"
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
/* Private define ------------------------------------------------------------*/
/**
 * @description: 把完整控制入口间隔换算成秒。
 * @detail: 首次启用或未使能时使用名义周期。接受名义周期的 0.5 到 3 倍；超出时 *dt 为 0 并返回 false，
 *          本次入口已写入 last_us。时间戳允许 uint32_t 回绕。入口间隔不是 ADC 硬件采样时间戳。
 *          clock 或 dt 为空、或名义周期非法时返回 false。不改控制量。
 * @param {motor_control_clock_t *} clock 调用者持有的时钟状态；空则失败
 * @param {bool} enabled                  true=按入口间隔计时
 * @param {uint32_t} time_us              本控制入口时间 (us)
 * @param {float} nominal_dt              名义控制周期 (s)，须为有限正值
 * @param {float *} dt                    输出周期 (s)；空则失败
 * @return {bool} true=dt 可用
 * @author: lkc
 */
bool Motor_Timing_ControlClockStep(motor_control_clock_t *clock, bool enabled, uint32_t time_us,
                                   float nominal_dt, float *dt)
{
    float measured_dt;
    uint32_t elapsed_us;
    if (clock == NULL || dt == NULL) return false;
    *dt = 0.0f;
    if (!isfinite(nominal_dt) || nominal_dt <= 0.0f) {
        clock->seeded = false;
        return false;
    }
    if (!enabled) {
        clock->seeded = false;
        *dt = nominal_dt;
        return true;
    }
    if (!clock->seeded) {
        clock->last_us = time_us;
        clock->seeded = true;
        *dt = nominal_dt;
        return true;
    }
    elapsed_us = time_us - clock->last_us;
    clock->last_us = time_us;
    measured_dt = (float)elapsed_us * 1.0e-6f;
    if (measured_dt < nominal_dt * 0.5f || measured_dt > nominal_dt * 3.0f) return false;
    *dt = measured_dt;
    return true;
}

/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint32_t s_runtime_average_q16;
static union {
    motor_handoff_sample_t handoff[MOTOR_HANDOFF_CAPACITY];
    motor_phase_sample_t phase[MOTOR_HANDOFF_CAPACITY];
    motor_observer_step_sample_t step[MOTOR_HANDOFF_CAPACITY];
    motor_pair_sample_t pair[MOTOR_HANDOFF_CAPACITY];
} s_capture;
typedef char motor_phase_fits_handoff[(sizeof(motor_phase_sample_t) <= sizeof(motor_handoff_sample_t)) ? 1 : -1];
typedef char motor_step_fits_handoff[(sizeof(motor_observer_step_sample_t) <= sizeof(motor_handoff_sample_t)) ? 1 : -1];
typedef char motor_pair_fits_handoff[(sizeof(motor_pair_sample_t) <= sizeof(motor_handoff_sample_t)) ? 1 : -1];
static bool s_pair_mode, s_pair_previous_valid;
static bool s_pair_half;
static motor_pair_frame_t s_pair_previous, s_pair_current;
static bool s_phase_mode;
static bool s_phase_long;
static bool s_step_mode;
static unsigned s_handoff_head, s_handoff_count, s_handoff_divider, s_handoff_post;
static bool s_handoff_active, s_handoff_marked, s_handoff_triggered;
/**
 * @description: 停机时武装交接短窗口。
 * @detail: 进入临界区后清索引并开始采集。与相位、观测步进和配对记录互斥，复用 32 条缓冲。
 *          每 15 个电流控制拍记录一次；完成交接拍强制记录，再留 15 条后停止。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffArm(void)
{
    uint32_t key = Motor_Crit_Enter();
    s_handoff_head = s_handoff_count = s_handoff_divider = s_handoff_post = 0U;
    s_handoff_marked = s_handoff_triggered = false;
    s_handoff_active = true;
    s_phase_mode = false;
    s_phase_long = false;
    s_step_mode = false;
    s_pair_mode = s_pair_previous_valid = false;
    s_pair_half = false;
    Motor_Crit_Exit(key);
}
/**
 * @description: 停止交接窗口继续写入。
 * @detail: 只清采集开关。不清已有样本，不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffStop(void)
{
    uint32_t key = Motor_Crit_Enter();
    s_handoff_active = false;
    Motor_Crit_Exit(key);
}
/**
 * @description: 标记下一拍为交接完成拍。
 * @detail: 仅在窗口仍活动且尚未触发时生效。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffMark(void)
{
    if (s_handoff_active && !s_handoff_triggered) s_handoff_marked = true;
}
/**
 * @description: 按当前窗口的分频决定本拍是否采样。
 * @detail: 未武装返回 false。已标记的完成拍立即放行，长窗口后再留 27 条，短窗口再留 15 条。
 *          否则半周期配对每 901 拍、长窗口每 450 拍、交接窗口每 15 拍放行一次。不改控制。
 * @return {bool} true=本拍应写入一条样本
 * @author: lkc
 */
static bool Motor_Timing_CaptureDue(void)
{
    if (!s_handoff_active) return false;
    if (s_handoff_marked) {
        s_handoff_triggered = true;
        s_handoff_post = s_phase_long ? 28U : 16U;
    } else if (++s_handoff_divider < (s_pair_half ? 901U : (s_phase_long ? 450U : 15U))) return false;
    s_handoff_divider = 0U;
    return true;
}
/**
 * @description: 判断本电流控制拍是否应记录交接样本。
 * @detail: 相位模式开启时返回 false。否则按 15 拍分频；已标记的完成拍立即放行。不改控制。
 * @return {bool} true=本拍应依次调用 Before、Transport、After
 * @author: lkc
 */
bool Motor_Timing_HandoffDue(void) { return !s_phase_mode && Motor_Timing_CaptureDue(); }
/**
 * @description: 武装相位记录，与交接记录互斥复用缓冲。
 * @detail: 进入临界区。PLL 更新后由调用者写入同拍输入输出。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PhaseArm(void)
{
    uint32_t key = Motor_Crit_Enter();
    Motor_Timing_HandoffArm();
    s_phase_mode = true;
    Motor_Crit_Exit(key);
}
/**
 * @description: 查询相位记录是否为当前格式。
 * @return {bool} true=相位模式
 * @author: lkc
 */
bool Motor_Timing_PhaseMode(void) { return s_phase_mode; }
/**
 * @description: 停机时武装长相位窗口。
 * @detail: 复用 32 条缓冲。每 450 拍抽样，完成拍及其后 27 条后自动停止。不保证捕获拍间尖峰。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PhaseArmLong(void)
{
    uint32_t key = Motor_Crit_Enter();
    Motor_Timing_PhaseArm();
    s_phase_long = true;
    Motor_Crit_Exit(key);
}
/**
 * @description: 判断本拍是否写入相位样本。
 * @detail: 步进或配对模式、未武装时返回 false。completed 为真且尚未触发时标记完成拍。不改控制。
 * @param {bool} completed 本拍相位环已完成
 * @return {bool} true=本拍应 Push
 * @author: lkc
 */
bool Motor_Timing_PhaseDue(bool completed)
{
    if (!s_phase_mode || s_step_mode || s_pair_mode || !s_handoff_active) return false;
    if (completed && !s_handoff_triggered) s_handoff_marked = true;
    return Motor_Timing_CaptureDue();
}
/**
 * @description: 写入一条相位样本。
 * @detail: 时间戳在本函数内读取。调用者保证 sample 非空。配额用完后停止。不改控制。
 * @param {const motor_phase_sample_t *} sample 本拍样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PhasePush(const motor_phase_sample_t *sample)
{
    motor_phase_sample_t *destination = &s_capture.phase[s_handoff_head];
    *destination = *sample;
    destination->time_us = Hal_Motor_TimeUs();
    destination->flags = s_handoff_marked ? 1U : (s_handoff_triggered ? 2U : 0U);
    s_handoff_marked = false;
    s_handoff_head = (s_handoff_head + 1U) % MOTOR_HANDOFF_CAPACITY;
    if (s_handoff_count < MOTOR_HANDOFF_CAPACITY) s_handoff_count++;
    if (s_handoff_post && --s_handoff_post == 0U) s_handoff_active = false;
}
/**
 * @description: 按时间顺序读取一条已停止的相位样本。
 * @detail: 仍在采集、不是相位格式、out 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index               时间顺序下标
 * @param {motor_phase_sample_t *} out   输出；空则失败
 * @return {bool} true=已写入 out
 * @author: lkc
 */
bool Motor_Timing_PhaseGet(unsigned index, motor_phase_sample_t *out)
{
    unsigned first;
    if (!s_phase_mode || s_step_mode || s_pair_mode || s_handoff_active || out == NULL || index >= s_handoff_count) return false;
    first = (s_handoff_head + MOTOR_HANDOFF_CAPACITY - s_handoff_count) % MOTOR_HANDOFF_CAPACITY;
    *out = s_capture.phase[(first + index) % MOTOR_HANDOFF_CAPACITY];
    return true;
}
/**
 * @description: 停机时武装观测步进记录。
 * @detail: 复用 32 条缓冲。每 450 拍抽样，完成拍后的首拍触发，再留 27 条。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ObserverStepArm(void)
{
    uint32_t key = Motor_Crit_Enter();
    Motor_Timing_PhaseArmLong();
    s_step_mode = true;
    Motor_Crit_Exit(key);
}
/**
 * @description: 查询观测步进记录是否为当前格式。
 * @return {bool} true=步进模式
 * @author: lkc
 */
bool Motor_Timing_ObserverStepMode(void) { return s_step_mode; }
/**
 * @description: 停机时武装连续两帧配对记录。
 * @detail: 复用 32 条缓冲，每 450 拍抽取一对。回调寄存器不是 ADC 触发锁存值。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PairArm(void)
{
    uint32_t key = Motor_Crit_Enter();
    Motor_Timing_PhaseArmLong();
    s_pair_mode = true;
    memset(&s_pair_current, 0, sizeof(s_pair_current));
    Motor_Crit_Exit(key);
}
/**
 * @description: 查询配对记录是否为当前格式。
 * @return {bool} true=配对模式
 * @author: lkc
 */
bool Motor_Timing_PairMode(void) { return s_pair_mode; }
/**
 * @description: 停机时武装原始半周期回调配对。
 * @detail: 每 901 次回调抽样。起转时应停止诊断，本函数不阻止控制。状态位复用 reconstructed。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PairHalfArm(void)
{
    uint32_t key = Motor_Crit_Enter();
    Motor_Timing_PairArm();
    s_pair_half = true;
    Motor_Crit_Exit(key);
}
/**
 * @description: 查询是否为半周期配对格式。
 * @return {bool} true=半周期配对
 * @author: lkc
 */
bool Motor_Timing_PairHalfMode(void) { return s_pair_half; }
/**
 * @description: 查询配对记录是否仍在采集。
 * @return {bool} true=配对模式且窗口未冻结
 * @author: lkc
 */
bool Motor_Timing_PairActive(void) { return s_pair_mode && s_handoff_active; }
/**
 * @description: 写入重构后的电流帧，并判断本拍是否凑成一对。
 * @detail: 每个选中 FOC 拍调用一次。第一帧只建立前拍，返回 false。之后按分频返回 true 时，同拍调用 PairPush。
 *          frame 为空或未在采集时返回 false。不改控制。
 * @param {const motor_pair_frame_t *} frame 本拍电流帧；空则失败
 * @return {bool} true=本拍应调用 PairPush
 * @author: lkc
 */
bool Motor_Timing_PairFrame(const motor_pair_frame_t *frame)
{
    uint32_t sequence;
    if (!Motor_Timing_PairActive() || frame == NULL) return false;
    sequence = s_pair_current.sample_sequence + 1U;
    s_pair_previous = s_pair_current;
    s_pair_current = *frame;
    s_pair_current.sample_sequence = sequence;
    if (!s_pair_previous_valid) {
        s_pair_previous_valid = true;
        return false;
    }
    return Motor_Timing_CaptureDue();
}
/**
 * @description: 写入一对电流帧及本拍 α/β 电压。
 * @detail: 仅在 PairFrame 返回 true 的同一拍调用。配额用完后停止。不改控制。
 * @param {float} v_alpha α 轴电压 (V)
 * @param {float} v_beta  β 轴电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PairPush(float v_alpha, float v_beta)
{
    motor_pair_sample_t *destination = &s_capture.pair[s_handoff_head];
    destination->time_us = Hal_Motor_TimeUs();
    destination->flags = s_handoff_marked ? 1U : (s_handoff_triggered ? 2U : 0U);
    destination->previous = s_pair_previous; destination->current = s_pair_current;
    destination->v_alpha = v_alpha; destination->v_beta = v_beta;
    s_handoff_marked = false;
    s_handoff_head = (s_handoff_head + 1U) % MOTOR_HANDOFF_CAPACITY;
    if (s_handoff_count < MOTOR_HANDOFF_CAPACITY) s_handoff_count++;
    if (s_handoff_post && --s_handoff_post == 0U) s_handoff_active = false;
}
/**
 * @description: 按时间顺序读取一条已停止的配对样本。
 * @detail: 仍在采集、不是配对格式、out 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index              时间顺序下标
 * @param {motor_pair_sample_t *} out   输出；空则失败
 * @return {bool} true=已写入 out
 * @author: lkc
 */
bool Motor_Timing_PairGet(unsigned index, motor_pair_sample_t *out)
{
    unsigned first;
    if (!s_pair_mode || s_handoff_active || out == NULL || index >= s_handoff_count) return false;
    first = (s_handoff_head + MOTOR_HANDOFF_CAPACITY - s_handoff_count) % MOTOR_HANDOFF_CAPACITY;
    *out = s_capture.pair[(first + index) % MOTOR_HANDOFF_CAPACITY];
    return true;
}
/**
 * @description: 查询观测步进记录是否仍在采集。
 * @detail: 默认实例更新前调用，用于维护连续前一拍电流。不推进抽样计数。
 * @return {bool} true=步进模式且窗口未冻结
 * @author: lkc
 */
bool Motor_Timing_ObserverStepActive(void) { return s_step_mode && s_handoff_active; }
/**
 * @description: 判断本拍是否写入观测步进样本。
 * @detail: 只在步进模式推进一次抽样计数。不改控制。
 * @return {bool} true=本拍应 Push
 * @author: lkc
 */
bool Motor_Timing_ObserverStepDue(void)
{
    return s_step_mode && Motor_Timing_CaptureDue();
}
/**
 * @description: 写入一条观测步进样本。
 * @detail: 时间戳在本函数内读取。调用者保证 sample 非空。配额用完后停止。不改控制。
 * @param {const motor_observer_step_sample_t *} sample 本拍样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ObserverStepPush(const motor_observer_step_sample_t *sample)
{
    motor_observer_step_sample_t *destination = &s_capture.step[s_handoff_head];
    *destination = *sample;
    destination->time_us = Hal_Motor_TimeUs();
    destination->flags = s_handoff_marked ? 1U : (s_handoff_triggered ? 2U : 0U);
    s_handoff_marked = false;
    s_handoff_head = (s_handoff_head + 1U) % MOTOR_HANDOFF_CAPACITY;
    if (s_handoff_count < MOTOR_HANDOFF_CAPACITY) s_handoff_count++;
    if (s_handoff_post && --s_handoff_post == 0U) s_handoff_active = false;
}
/**
 * @description: 按时间顺序读取一条已停止的观测步进样本。
 * @detail: 仍在采集、不是步进格式、out 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index                      时间顺序下标
 * @param {motor_observer_step_sample_t *} out  输出；空则失败
 * @return {bool} true=已写入 out
 * @author: lkc
 */
bool Motor_Timing_ObserverStepGet(unsigned index, motor_observer_step_sample_t *out)
{
    unsigned first;
    if (!s_step_mode || s_handoff_active || out == NULL || index >= s_handoff_count) return false;
    first = (s_handoff_head + MOTOR_HANDOFF_CAPACITY - s_handoff_count) % MOTOR_HANDOFF_CAPACITY;
    *out = s_capture.step[(first + index) % MOTOR_HANDOFF_CAPACITY];
    return true;
}
/**
 * @description: 写入交接拍限幅前的电流与积分。
 * @detail: 仅在 Due 返回 true 后、同一电流控制拍、Transport 之前调用。
 *          id/iq 是本拍目标，样本里的反馈取 motor->Id/Iq。调用者保证 motor 非空。不改控制。
 * @param {const MOTOR_PARAM_S *} motor 电流、积分与母线
 * @param {float} theta                 电角度 (rad)
 * @param {float} delta                 坐标交接附加角 (rad)
 * @param {float} id                    本拍 Id 目标 (A)
 * @param {float} iq                    本拍 Iq 目标 (A)
 * @param {float} dt                    控制周期 (s)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffBefore(const MOTOR_PARAM_S *motor, float theta, float delta, float id, float iq, float dt)
{
    motor_handoff_sample_t *sample = &s_capture.handoff[s_handoff_head];
    sample->time_us = Hal_Motor_TimeUs();
    sample->flags = s_handoff_marked ? 1U : (s_handoff_triggered ? 2U : 0U);
    s_handoff_marked = false;
    sample->theta = theta; sample->delta = delta; sample->id = motor->Id; sample->iq = motor->Iq;
    sample->old_id = motor->id_target; sample->old_iq = motor->iq_target;
    sample->id_cmd = id; sample->iq_cmd = iq; sample->dt = dt; sample->kp = motor->ol_current_kp;
    sample->vd_before = motor->ol_vd_int; sample->vq_before = motor->ol_vq_int;
    sample->ia = motor->ia; sample->ib = motor->ib; sample->ic = motor->ic; sample->vbus = motor->v_bus;
    sample->adc[0] = motor->currentAdc[0]; sample->adc[1] = motor->currentAdc[1]; sample->adc[2] = motor->currentAdc[2];
}
/**
 * @description: 写入坐标旋转后的电流积分。
 * @detail: 同一拍在 Before 之后、After 之前调用。调用者保证 motor 非空。不改控制。
 * @param {const MOTOR_PARAM_S *} motor 读取 ol_vd_int/ol_vq_int (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffTransport(const MOTOR_PARAM_S *motor)
{
    s_capture.handoff[s_handoff_head].vd_transport = motor->ol_vd_int;
    s_capture.handoff[s_handoff_head].vq_transport = motor->ol_vq_int;
}
/**
 * @description: 写入限幅后的积分与电压，并推进环形索引。
 * @detail: 同一拍最后调用。配额用完后停止采集。调用者保证 motor 非空。不改控制。
 * @param {const MOTOR_PARAM_S *} motor 读取限幅后积分 (V)
 * @param {float} vd                    限幅后 vd (V)
 * @param {float} vq                    限幅后 vq (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffAfter(const MOTOR_PARAM_S *motor, float vd, float vq)
{
    motor_handoff_sample_t *sample = &s_capture.handoff[s_handoff_head];
    sample->vd_after = motor->ol_vd_int; sample->vq_after = motor->ol_vq_int;
    sample->vd = vd; sample->vq = vq;
    s_handoff_head = (s_handoff_head + 1U) % MOTOR_HANDOFF_CAPACITY;
    if (s_handoff_count < MOTOR_HANDOFF_CAPACITY) s_handoff_count++;
    if (s_handoff_post && --s_handoff_post == 0U) s_handoff_active = false;
}
/**
 * @description: 返回交接缓冲里尚未被覆盖的条数。
 * @detail: 仍在采集时也会增长，最大 32。不表示 Get 已经允许读取。
 * @return {unsigned} 条数
 * @author: lkc
 */
unsigned Motor_Timing_HandoffCount(void) { return s_handoff_count; }
/**
 * @description: 按时间顺序读取一条已停止的交接样本。
 * @detail: 仍在采集、处于相位模式、out 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index               时间顺序下标
 * @param {motor_handoff_sample_t *} out 输出；空则失败
 * @return {bool} true=已写入 out
 * @author: lkc
 */
bool Motor_Timing_HandoffGet(unsigned index, motor_handoff_sample_t *out)
{
    unsigned first;
    if (s_phase_mode || s_handoff_active || out == NULL || index >= s_handoff_count) return false;
    first = (s_handoff_head + MOTOR_HANDOFF_CAPACITY - s_handoff_count) % MOTOR_HANDOFF_CAPACITY;
    *out = s_capture.handoff[(first + index) % MOTOR_HANDOFF_CAPACITY];
    return true;
}
static volatile uint32_t s_runtime_maximum_us;
static volatile uint32_t s_sample_up_count;
static volatile uint32_t s_sample_down_count;
static uint8_t s_profile_divider;
static bool s_profile_active;
static bool s_profile_detailed;
static bool s_performance_enabled = true;
static volatile uint32_t s_profile_sum[MOTOR_TIMING_STAGE_COUNT];
static volatile uint32_t s_profile_count[MOTOR_TIMING_STAGE_COUNT];
static volatile uint32_t s_profile_max[MOTOR_TIMING_STAGE_COUNT];
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
static bool s_trace_active, s_trace_closed, s_trace_trigger;
static bool s_trace_entry_mode, s_trace_entry_trigger;
static bool s_trace_entry_fast;
static bool s_trace_light;
static bool s_trace_compact, s_compact_running, s_compact_stop;
static bool s_trace_fault;
static uint32_t s_fault_trace_last_us;
static bool s_fault_trace_seeded;
static uint32_t s_compact_flags, s_compact_trigger_flags;
static volatile uint32_t s_speed_complete_us;
static bool s_cadence_active;
static bool s_task_only;
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
static bool s_trace_flight;
#define MOTOR_TRACE_FLIGHT_ENTRY 192U
#define MOTOR_TRACE_FLIGHT_RING (MOTOR_TRACE_CAPACITY - MOTOR_TRACE_FLIGHT_ENTRY)
static unsigned s_flight_entry_count, s_flight_head, s_flight_count;
static motor_cadence_t s_cadence;
static uint32_t s_cadence_last;
static bool s_cadence_seed;
static motor_callback_t s_callback;
static uint32_t s_callback_start, s_callback_last;
static bool s_callback_active, s_callback_seed;
static uint32_t s_trace_entry_last;
static bool s_trace_entry_seed;
static uint16_t s_trace_gap_max, s_trace_full_max, s_trace_pending, s_trace_up;
/* Private functions ---------------------------------------------------------*/
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
 * @description: 一次汇总高频控制路径中已开启的诊断。
 * @detail: 不推进分频、不读取时钟。回调 full 标记在细分诊断中由 CallbackClassify 写入；
 *          非紧凑轨迹的入口统计仍由 ProfileOpen 更新。
 * @return {uint32_t} 本拍诊断位掩码
 */
uint32_t Motor_Timing_IsrFeatures(void)
{
    uint32_t features = 0U;
    if (s_performance_enabled || (s_cadence_active && !s_task_only) || (s_trace_active && !s_trace_compact))
        features |= MOTOR_TIMING_ISR_PROFILE;
    if (s_performance_enabled) features |= MOTOR_TIMING_ISR_RUNTIME;
    if (s_pair_mode && s_handoff_active && !s_pair_half) features |= MOTOR_TIMING_ISR_PAIR;
    if (s_trace_active) {
        if (s_trace_fault) features |= MOTOR_TIMING_ISR_FAULT;
        else if (s_trace_compact) features |= MOTOR_TIMING_ISR_COMPACT;
        else features |= MOTOR_TIMING_ISR_TRACE;
    }
    return features;
}
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

/**
 * @description: 推进分段耗时分频，并决定本拍是否采样。
 * @detail: 性能统计关闭时返回 false。分频计数是 uint8_t，每次调用加 1，回绕为 0 时返回 true，即每 256 次调用采 1 次。
 *          调用方只在通过控制门控之后调用。本函数不读取耗时，也不改变控制量。
 * @return {bool} true=本拍后续 Record 会累加
 * @author: lkc
 */
bool Motor_Timing_ProfileBegin(void)
{
    if (!s_performance_enabled) { s_profile_active = false; return false; }
    s_profile_divider++;
    s_profile_active = s_profile_divider == 0U;
    return s_profile_active;
}

/**
 * @description: 返回本拍 Begin 写下的采样标志。
 * @detail: End 会把它清掉。保护失败提前返回时，Begin 可能已为 true，但各段尚未 Record。
 * @return {bool} true=本拍采样仍然有效
 * @author: lkc
 */
bool Motor_Timing_ProfileIsActive(void) { return s_profile_active; }
/**
 * @description: 开关电流环内部的细分计时。
 * @detail: 默认关闭。只影响诊断采样，不改变控制周期。总分段采样仍由 ProfileBegin 决定。
 * @param {bool} enabled true=允许细分阶段记录
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileSetDetailed(bool enabled) { s_profile_detailed = enabled; }
/**
 * @description: 查询本拍细分计时是否有效。
 * @detail: 总采样标志和细分开关都为真才返回 true。
 * @return {bool} true=本拍可记录细分阶段
 * @author: lkc
 */
bool Motor_Timing_ProfileDetailedIsActive(void) { return s_profile_active && s_profile_detailed; }

/**
 * @description: 读取分段计时用的当前时间戳。
 * @detail: 直接返回 Hal_Motor_TimeUs()，不改统计。
 * @return {uint32_t} 时间戳 (us)
 * @author: lkc
 */
uint32_t Motor_Timing_ProfileTimeUs(void) { return Hal_Motor_TimeUs(); }

/**
 * @description: 清除本拍分段采样标志。
 * @detail: 无条件写 false。Begin 返回 false 时再调用，结果仍是 false。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileEnd(void) { s_profile_active = false; }

/**
 * @description: 把一个阶段从 start_us 到现在的耗时累进当前窗口。
 * @detail: 采样标志为 false、阶段号非法，或耗时大于 1000 us 时直接返回。
 *          该段样本数已经达到 256 时，先清掉该段的 sum/count/max，再计入本拍。
 *          跳过的 ADC 回调不会调用本函数。CONTROL 与 OBSERVER、TRANSPORT 的时间互相包含；
 *          BLEND/CLOSED 与 OBSERVER 是同一段耗时，平均值不能相加。
 *          SENSE/SENSOR/PROTECTION 只在使能的无感 I/F 细分拍计入，停机拍留在 SAMPLE。
 *          VOLTAGE_REBUILD 含在 OUTPUT 内，不能与 OUTPUT 相加。
 * @param {motor_timing_stage_t} stage 阶段编号
 * @param {uint32_t} start_us          阶段开始时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileRecord(motor_timing_stage_t stage, uint32_t start_us)
{
    uint32_t elapsed;
    if (!s_profile_active || (unsigned)stage >= MOTOR_TIMING_STAGE_COUNT) return;
    elapsed = Hal_Motor_TimeUs() - start_us;
    if (elapsed > 1000U) return;
    /* 该段满 256 次先清掉再计入本拍，避免 sum 一直累加。
     * 跳过的 ADC 回调不会进到这里；full/skip 拆分在 CallbackEnd。 */
    if (s_profile_count[stage] >= 256U) {
        s_profile_max[stage] = 0U;
        s_profile_sum[stage] = 0U;
        s_profile_count[stage] = 0U;
    }
    if (elapsed > s_profile_max[stage]) s_profile_max[stage] = elapsed;
    s_profile_sum[stage] += elapsed;
    s_profile_count[stage]++;
}

/**
 * @description: 读出一个阶段当前窗口的样本数和峰值。
 * @detail: 本函数进入临界区。输出指针不可为空，违反约定触发 COMMON_ASSERT。stage 非法则直接返回。
 *          CadenceReset 会清这些数组。
 * @param {motor_timing_stage_t} stage 阶段编号
 * @param {uint32_t *} count           输出样本数，不可为空
 * @param {uint32_t *} maximum         输出峰值 (us)，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileDetail(motor_timing_stage_t stage, uint32_t *count, uint32_t *maximum)
{
    uint32_t key;
    COMMON_ASSERT(count != NULL && maximum != NULL);
    if ((unsigned)stage >= MOTOR_TIMING_STAGE_COUNT) return;
    key = Motor_Crit_Enter();
    *count = s_profile_count[stage];
    *maximum = s_profile_max[stage];
    Motor_Crit_Exit(key);
}
/**
 * @description: 在短临界区内复制全部分段原始统计。
 * @detail: 平均值和打印由调用方在临界区外计算。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_profile_snapshot_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileSnapshot(motor_profile_snapshot_t *out)
{
    unsigned index;
    uint32_t key;
    COMMON_ASSERT(out != NULL);
    key = Motor_Crit_Enter();
    for (index = 0U; index < MOTOR_TIMING_STAGE_COUNT; index++) {
        out->count[index] = s_profile_count[index];
        out->sum_us[index] = s_profile_sum[index];
        out->max_us[index] = s_profile_max[index];
    }
    Motor_Crit_Exit(key);
}

/**
 * @description: 按各段 sum/count 写出平均耗时。
 * @detail: 本函数不加临界区。count 为 0 的段写 0。samples 写出的是 CONTROL 段当前 count，不是 Begin 次数。
 * @param {float[MOTOR_TIMING_STAGE_COUNT]} average_us 输出平均耗时 (us)，不可为空
 * @param {uint32_t *} samples                          输出 CONTROL 段样本数，可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileGet(float average_us[MOTOR_TIMING_STAGE_COUNT], uint32_t *samples)
{
    unsigned i;
    COMMON_ASSERT(average_us != NULL);
    for (i = 0U; i < MOTOR_TIMING_STAGE_COUNT; ++i) {
        uint32_t count = s_profile_count[i];
        average_us[i] = count ? (float)s_profile_sum[i] / (float)count : 0.0f;
    }
    if (samples != NULL) *samples = s_profile_count[MOTOR_TIMING_CONTROL];
}
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 更新本拍 ADC-DMA 中断耗时统计（平均与最大值）。
 * @detail: 性能统计关闭或超过 1000 us 的样本直接丢弃。均值用 Q16 整数递推，系数 1/50，读取时换算为 us。
 * @param {uint32_t} start_time_us 本拍开始时的时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_RecordRuntime(uint32_t start_time_us)
{
    uint32_t elapsed_us;
    uint32_t elapsed_q16;
    if (!s_performance_enabled) return;
    elapsed_us = Hal_Motor_TimeUs() - start_time_us;
    if (elapsed_us > 1000U) return;
    elapsed_q16 = elapsed_us << 16;
    if (s_runtime_average_q16 == 0U) {
        s_runtime_average_q16 = elapsed_q16;
    } else {
        int32_t delta = (int32_t)elapsed_q16 - (int32_t)s_runtime_average_q16;
        s_runtime_average_q16 = (uint32_t)((int32_t)s_runtime_average_q16 + delta / 50);
    }
    if (elapsed_us > s_runtime_maximum_us) {
        s_runtime_maximum_us = elapsed_us;
    }
}

/**
 * @description: 计算 ADC/保护用采样周期。
 * @detail: 优先用 Hal_Motor_AdcSampleRateHz()；无效时回退 motor->ol_foc_freq，并写回该字段。
 * @param {MOTOR_PARAM_S *} motor 电机对象，为空时返回 1 s
 * @return {float} 采样周期 (s)，即 1/ADC 触发频率
 * @author: lkc
 */
float Motor_Timing_GetAdcDt(MOTOR_PARAM_S *motor)
{
    float adc_hz;
    if (motor == NULL) return 1.0f;
    adc_hz = Hal_Motor_AdcSampleRateHz();
    if (adc_hz > 1.0f) {
        motor->ol_foc_freq = adc_hz;
    } else {
        adc_hz = motor->ol_foc_freq;
        if (adc_hz <= 1.0f) {
            adc_hz = 1.0f;
            motor->ol_foc_freq = adc_hz;
        }
    }
    return 1.0f / adc_hz;
}

/**
 * @description: 计算 FOC 控制环周期（已乘分频）。
 * @detail: dt = ADC 周期 × FOC_CONTROL_LOOP_FREQ_DIVIDER，供斜坡、PI 和观测器使用。
 * @param {MOTOR_PARAM_S *} motor 电机对象
 * @return {float} 控制周期 (s)
 * @author: lkc
 */
float Motor_Timing_GetControlDt(MOTOR_PARAM_S *motor)
{
    return Motor_Timing_GetAdcDt(motor) * (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
}

/**
 * @description: 记录一次实际执行的 FOC 控制拍位于 PWM 向上或向下计数侧。
 * @detail: 在控制拍调用，只累加计数，不阻塞。性能统计关闭时仍然计数。
 * @param {bool} upward true=PWM 向上计数侧
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_RecordSampleDirection(bool upward)
{
    if (upward) {
        s_sample_up_count++;
    } else {
        s_sample_down_count++;
    }
}

/**
 * @description: 读取 FOC 控制拍落在 PWM 向上/向下计数侧的累计次数。
 * @detail: 输出指针可为空，空指针对应的一侧不写出。可在任务中调用。
 * @param {uint32_t *} up   向上计数侧累计次数，可为 NULL
 * @param {uint32_t *} down 向下计数侧累计次数，可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_DmaInt_GetSampleDirectionCounts(uint32_t *up, uint32_t *down)
{
    if (up != NULL) {
        *up = s_sample_up_count;
    }
    if (down != NULL) {
        *down = s_sample_down_count;
    }
}

/**
 * @description: 返回 ADC-DMA 中断的平均执行时间。
 * @detail: 由 Q16 均值换算。未采样时为 0。超过 1000 us 的样本不参与统计。
 * @return {float} 平均耗时 (us)
 * @author: lkc
 */
float Motor_DmaInt_GetAverageRuntimeUs(void) { return (float)s_runtime_average_q16 / 65536.0f; }

/**
 * @description: 返回 ADC-DMA 中断的历史最长执行时间。
 * @detail: 超过 1000 us 的样本不参与统计。未采样时为 0。
 * @return {float} 最大耗时 (us)
 * @author: lkc
 */
float Motor_DmaInt_GetMaximumRuntimeUs(void) { return (float)s_runtime_maximum_us; }

/**
 * @description: 返回实际 ADC-DMA 触发频率。
 * @detail: HAL 频率无效时回退 motor.ol_foc_freq。
 * @return {float} ADC 采样频率 (Hz)
 * @author: lkc
 */
float Motor_DmaInt_GetAdcFrequencyHz(void)
{
    float adc_hz = Hal_Motor_AdcSampleRateHz();
    return (adc_hz > 0.0f) ? adc_hz : motor.ol_foc_freq;
}

/**
 * @description: 返回 FOC 控制环实际运行频率。
 * @detail: 控制频率 = ADC 频率 / FOC_CONTROL_LOOP_FREQ_DIVIDER。
 * @return {float} 控制环频率 (Hz)
 * @author: lkc
 */
float Motor_DmaInt_GetControlFrequencyHz(void)
{
    return Motor_DmaInt_GetAdcFrequencyHz() / (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
}
