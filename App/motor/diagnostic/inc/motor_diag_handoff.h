/*
 * @Description: 无感接管交接、相位、观测步进与配对短窗口采样
 * @Author: lkc
 */
#ifndef MOTOR_DIAG_HANDOFF_H
#define MOTOR_DIAG_HANDOFF_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "motor_param.h"

typedef struct {
    uint32_t time_us, flags;
    float theta, delta, id, iq, old_id, old_iq, id_cmd, iq_cmd;
    float vd_before, vq_before, vd_transport, vq_transport, vd_after, vq_after;
    float vd, vq, ia, ib, ic, vbus, kp, dt;
    uint16_t adc[3];
} motor_handoff_sample_t;
#define MOTOR_HANDOFF_CAPACITY 32U
typedef struct {
    uint32_t time_us, flags;
    float observer, pll_before, pll_after, phase_error, speed_before, speed_after, rpm;
    float flux, x_alpha, x_beta, v_alpha, v_beta, i_alpha, i_beta;
    float dt, kp, ki, resistance, inductance, flux_nominal, phase_offset, blend, control_angle;
} motor_phase_sample_t;
typedef struct {
    uint32_t time_us, flags;
    float x_before_alpha, x_before_beta, x_after_alpha, x_after_beta;
    float i_before_alpha, i_before_beta, i_alpha, i_beta, v_alpha, v_beta;
    float correction_alpha, correction_beta, dt, resistance, inductance, flux_nominal, gain;
    float pll_before, observer, phase_error, blend, control_angle, previous_dt, phase_offset;
} motor_observer_step_sample_t;
typedef struct {
    uint32_t sample_sequence, pwm_sequence, entry_us;
    float ia, ib, ic;
    uint16_t adc[3], ccr[3];
    uint16_t callback_count, callback_cr1, reconstructed, period;
} motor_pair_frame_t;
typedef struct {
    uint32_t time_us, flags;
    motor_pair_frame_t previous, current;
    float v_alpha, v_beta;
} motor_pair_sample_t;
/**
 * @description: 停机时武装连续两帧配对记录。
 * @detail: 复用 32 条缓冲，每 450 拍抽取一对。回调寄存器不是 ADC 触发锁存值。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PairArm(void);

/**
 * @description: 停机时武装原始半周期回调配对。
 * @detail: 每 901 次回调抽样。起转时应停止诊断，本函数不阻止控制。状态位复用 reconstructed。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PairHalfArm(void);

/**
 * @description: 查询是否为半周期配对格式。
 * @return {bool} true=半周期配对
 * @author: lkc
 */
bool Motor_Timing_PairHalfMode(void);

/**
 * @description: 查询配对记录是否为当前格式。
 * @return {bool} true=配对模式
 * @author: lkc
 */
bool Motor_Timing_PairMode(void);

/**
 * @description: 查询配对记录是否仍在采集。
 * @return {bool} true=配对模式且窗口未冻结
 * @author: lkc
 */
bool Motor_Timing_PairActive(void);

/**
 * @description: 写入重构后的电流帧，并判断本拍是否凑成一对。
 * @detail: 每个选中 FOC 拍调用一次。第一帧只建立前拍，返回 false。之后按分频返回 true 时，同拍调用 PairPush。
 *          frame 为空或未在采集时返回 false。不改控制。
 * @param {const motor_pair_frame_t *} frame 本拍电流帧；空则失败
 * @return {bool} true=本拍应调用 PairPush
 * @author: lkc
 */
bool Motor_Timing_PairFrame(const motor_pair_frame_t *frame);

/**
 * @description: 写入一对电流帧及本拍 α/β 电压。
 * @detail: 仅在 PairFrame 返回 true 的同一拍调用。配额用完后停止。不改控制。
 * @param {float} v_alpha α 轴电压 (V)
 * @param {float} v_beta  β 轴电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PairPush(float v_alpha, float v_beta);

/**
 * @description: 按时间顺序读取一条已停止的配对样本。
 * @detail: 仍在采集、不是配对格式、out 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index              时间顺序下标
 * @param {motor_pair_sample_t *} out   输出；空则失败
 * @return {bool} true=已写入 out
 * @author: lkc
 */
bool Motor_Timing_PairGet(unsigned index, motor_pair_sample_t *out);

/**
 * @description: 停机时武装观测步进记录。
 * @detail: 复用 32 条缓冲。每 450 拍抽样，完成拍后的首拍触发，再留 27 条。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ObserverStepArm(void);

/**
 * @description: 查询观测步进记录是否为当前格式。
 * @return {bool} true=步进模式
 * @author: lkc
 */
bool Motor_Timing_ObserverStepMode(void);

/**
 * @description: 查询观测步进记录是否仍在采集。
 * @detail: 默认实例更新前调用，用于维护连续前一拍电流。不推进抽样计数。
 * @return {bool} true=步进模式且窗口未冻结
 * @author: lkc
 */
bool Motor_Timing_ObserverStepActive(void);

/**
 * @description: 判断本拍是否写入观测步进样本。
 * @detail: 只在步进模式推进一次抽样计数。不改控制。
 * @return {bool} true=本拍应 Push
 * @author: lkc
 */
bool Motor_Timing_ObserverStepDue(void);

/**
 * @description: 写入一条观测步进样本。
 * @detail: 时间戳在本函数内读取。调用者保证 sample 非空。配额用完后停止。不改控制。
 * @param {const motor_observer_step_sample_t *} sample 本拍样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ObserverStepPush(const motor_observer_step_sample_t *sample);

/**
 * @description: 按时间顺序读取一条已停止的观测步进样本。
 * @detail: 仍在采集、不是步进格式、out 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index                      时间顺序下标
 * @param {motor_observer_step_sample_t *} out  输出；空则失败
 * @return {bool} true=已写入 out
 * @author: lkc
 */
bool Motor_Timing_ObserverStepGet(unsigned index, motor_observer_step_sample_t *out);

/**
 * @description: 武装相位记录，与交接记录互斥复用缓冲。
 * @detail: 进入临界区。PLL 更新后由调用者写入同拍输入输出。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PhaseArm(void);

/**
 * @description: 停机时武装长相位窗口。
 * @detail: 复用 32 条缓冲。每 450 拍抽样，完成拍及其后 27 条后自动停止。不保证捕获拍间尖峰。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PhaseArmLong(void);

/**
 * @description: 查询相位记录是否为当前格式。
 * @return {bool} true=相位模式
 * @author: lkc
 */
bool Motor_Timing_PhaseMode(void);

/**
 * @description: 判断本拍是否写入相位样本。
 * @detail: 步进或配对模式、未武装时返回 false。completed 为真且尚未触发时标记完成拍。不改控制。
 * @param {bool} completed 本拍相位环已完成
 * @return {bool} true=本拍应 Push
 * @author: lkc
 */
bool Motor_Timing_PhaseDue(bool completed);

/**
 * @description: 写入一条相位样本。
 * @detail: 时间戳在本函数内读取。调用者保证 sample 非空。配额用完后停止。不改控制。
 * @param {const motor_phase_sample_t *} sample 本拍样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_PhasePush(const motor_phase_sample_t *sample);

/**
 * @description: 按时间顺序读取一条已停止的相位样本。
 * @detail: 仍在采集、不是相位格式、out 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index               时间顺序下标
 * @param {motor_phase_sample_t *} out   输出；空则失败
 * @return {bool} true=已写入 out
 * @author: lkc
 */
bool Motor_Timing_PhaseGet(unsigned index, motor_phase_sample_t *out);

/**
 * @description: 停机时武装交接短窗口。
 * @detail: 进入临界区后清索引并开始采集。与相位、观测步进和配对记录互斥，复用 32 条缓冲。
 *          每 15 个电流控制拍记录一次；完成交接拍强制记录，再留 15 条后停止。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffArm(void);

/**
 * @description: 停止交接窗口继续写入。
 * @detail: 只清采集开关。不清已有样本，不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffStop(void);

/**
 * @description: 标记下一拍为交接完成拍。
 * @detail: 仅在窗口仍活动且尚未触发时生效。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffMark(void);

/**
 * @description: 判断本电流控制拍是否应记录交接样本。
 * @detail: 相位模式开启时返回 false。否则按 15 拍分频；已标记的完成拍立即放行。不改控制。
 * @return {bool} true=本拍应依次调用 Before、Transport、After
 * @author: lkc
 */
bool Motor_Timing_HandoffDue(void);

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
void Motor_Timing_HandoffBefore(const MOTOR_PARAM_S *motor, float theta, float delta, float id, float iq, float dt);

/**
 * @description: 写入坐标旋转后的电流积分。
 * @detail: 同一拍在 Before 之后、After 之前调用。调用者保证 motor 非空。不改控制。
 * @param {const MOTOR_PARAM_S *} motor 读取 ol_vd_int/ol_vq_int (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffTransport(const MOTOR_PARAM_S *motor);

/**
 * @description: 写入限幅后的积分与电压，并推进环形索引。
 * @detail: 同一拍最后调用。配额用完后停止采集。调用者保证 motor 非空。不改控制。
 * @param {const MOTOR_PARAM_S *} motor 读取限幅后积分 (V)
 * @param {float} vd                    限幅后 vd (V)
 * @param {float} vq                    限幅后 vq (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_HandoffAfter(const MOTOR_PARAM_S *motor, float vd, float vq);

/**
 * @description: 返回交接缓冲里尚未被覆盖的条数。
 * @detail: 仍在采集时也会增长，最大 32。不表示 Get 已经允许读取。
 * @return {unsigned} 条数
 * @author: lkc
 */
unsigned Motor_Timing_HandoffCount(void);

/**
 * @description: 按时间顺序读取一条已停止的交接样本。
 * @detail: 仍在采集、处于相位模式、out 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index               时间顺序下标
 * @param {motor_handoff_sample_t *} out 输出；空则失败
 * @return {bool} true=已写入 out
 * @author: lkc
 */
bool Motor_Timing_HandoffGet(unsigned index, motor_handoff_sample_t *out);

#ifdef __cplusplus
}
#endif
#endif
