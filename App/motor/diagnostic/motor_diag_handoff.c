/*
 * @Description: 无感接管交接、相位、观测步进与配对短窗口采样
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
static union {
    motor_handoff_sample_t handoff[MOTOR_HANDOFF_CAPACITY];
    motor_phase_sample_t phase[MOTOR_HANDOFF_CAPACITY];
    motor_observer_step_sample_t step[MOTOR_HANDOFF_CAPACITY];
    motor_pair_sample_t pair[MOTOR_HANDOFF_CAPACITY];
} s_capture;
typedef char motor_phase_fits_handoff[(sizeof(motor_phase_sample_t) <= sizeof(motor_handoff_sample_t)) ? 1 : -1];
typedef char motor_step_fits_handoff[(sizeof(motor_observer_step_sample_t) <= sizeof(motor_handoff_sample_t)) ? 1 : -1];
typedef char motor_pair_fits_handoff[(sizeof(motor_pair_sample_t) <= sizeof(motor_handoff_sample_t)) ? 1 : -1];
static bool s_pair_previous_valid;
bool s_pair_mode;
bool s_pair_half;
static motor_pair_frame_t s_pair_previous, s_pair_current;
static bool s_phase_mode;
static bool s_phase_long;
static bool s_step_mode;
static unsigned s_handoff_head, s_handoff_count, s_handoff_divider, s_handoff_post;
static bool s_handoff_marked, s_handoff_triggered;
bool s_handoff_active;
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
