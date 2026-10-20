/*
 * @Description: 无感运行轨迹与故障冻结轨迹记录
 * @Author: lkc
 */
#ifndef MOTOR_DIAG_TRACE_H
#define MOTOR_DIAG_TRACE_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "motor_config.h"
#include "motor_param.h"

#define MOTOR_TRACE_CAPACITY 512U


/**
 * @description: 无感运行 RAM 环形诊断的一条已写入样本。
 * @detail: 角度为电角度 (rad)，磁链 Wb，电流 A，电压 V，转速为机械 RPM。
 *          gap_max_us 和 up_count 含本条控制入口，因为 CadenceNote 在 Push 之前更新。
 *          full_max_us 和 pending_count 不含正在执行的这次 DMA 回调，CallbackEnd 在 Push 之后才写入。
 *          up_count 来自入口处的 TIM1 DIR 读数，不是 ADC 锁存边沿。
 *          flags 的 bit7/bit8 由 Push 按触发状态补上，调用方填入的样本不必带这两位。
 */
typedef struct {
    uint32_t time_us; /* 写入时刻 (us) */
    uint32_t flags;   /* state[1:0], flux_ok[2], direction_ok[3], encoder_valid[4], loss[7], entry[8] */
    float theta;      /* 本拍控制电角 (rad) */
    float pll;        /* PLL 电角 (rad) */
    float flux;       /* 转子磁链 (Wb) */
    float rpm;        /* 观测机械转速 (RPM) */
    float id;         /* Id 反馈 (A) */
    float iq;         /* Iq 反馈 (A) */
    float iq_target;  /* 速度 PI 输出 (A) */
    float v_alpha;    /* 诊断电压 α (V) */
    float v_beta;     /* 诊断电压 β (V) */
    float enc_angle;  /* 编码器电角 (rad) */
    float enc_rpm;    /* 编码器机械转速 (RPM) */
    float enc_age;    /* 编码器样本年龄 (s) */
    float blend;      /* 角度接管比例，0=I/F，1=观测角 */
    float loss_time;  /* 失锁累计 (s) */
    float id_cmd;     /* 本拍最终 Id 目标 (A) */
    float iq_cmd;     /* 本拍最终 Iq 目标 (A) */
    float i_alpha;    /* Iα (A) */
    float i_beta;     /* Iβ (A) */
    float vd;         /* 限幅后 vd (V) */
    float vq;         /* 限幅后 vq (V) */
    float v_bus;      /* 母线电压 (V) */
    float v_limit;    /* 电压矢量上限 (V) */
    uint16_t gap_max_us;    /* 窗口内最长控制间隔 (us) */
    uint16_t full_max_us;   /* 窗口内最长完整回调耗时 (us) */
    uint16_t pending_count; /* 窗口内 DMA 标志再次置位次数 */
    uint16_t up_count;      /* 窗口内向上计数侧次数 */
} motor_trace_sample_t;

typedef struct {
    uint32_t time_us, flags;
    float theta, rpm, id, iq, id_cmd, iq_cmd;
} motor_trace_compact_sample_t;

typedef struct {
    uint32_t time_us, flags;
    uint16_t adc[3], reserved;
    float ia, ib, ic, theta, pll, flux, rpm, v_alpha, v_beta;
    float vd_int, vq_int, id, iq, id_cmd, iq_cmd, v_bus, dt;
} motor_fault_trace_sample_t;

#define MOTOR_FAULT_TRACE_STRIDE 4U
#define MOTOR_FAULT_DEBUG_MAGIC 0x464f4344U
#define MOTOR_FAULT_DEBUG_VERSION 1U
#define MOTOR_FAULT_DEBUG_INVALID 0U
#define MOTOR_FAULT_DEBUG_ARMED 1U
#define MOTOR_FAULT_DEBUG_FROZEN 2U

/**
 * @description: 调试器读取的固定56字节描述符，所有字段为32位小端。
 * @detail: sequence为偶数且前后相等才稳定。只在武装/冻结/复用时发布，armed的head/count不是实时值。
 *          frozen时数组只读；其他模式复用缓冲前先失效。不提供运行中的跨字段快照。
 */
typedef struct {
    uint32_t magic, version, sequence, state;
    uint32_t address, sample_bytes, capacity, stride;
    uint32_t head, count, fault, first_us, last_us, descriptor_bytes;
} motor_fault_debug_descriptor_t;

extern volatile motor_fault_debug_descriptor_t g_motor_fault_debug;

#if MOTOR_CFG_DIAG
/**
 * @description: 停机任务里武装故障前分频环形记录。
 * @detail: 复用完整轨迹 RAM，并暂停性能统计。已有故障须由应用拒绝武装，本函数不清除故障。
 *          采集期间不要从任务读取或切换模式。采样入口的 ADC、abc、母线和 dt 由调用者填本拍值。
 *          flags 高 16 位为故障码，低 2 位为观测状态，bit8 为跟踪事件锁存。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_FaultTraceArm(void);

/**
 * @description: 查询故障轨迹是否仍在采集。
 * @detail: ISR 写者用它检查采集开关。停机任务只宜用它判断格式是否活动，不要读取活动数组。
 * @return {bool} true=故障格式且未冻结
 * @author: lkc
 */
bool Motor_Timing_FaultTraceIsActive(void);

/**
 * @description: 查询当前记录格式是否为故障轨迹。
 * @return {bool} true=故障轨迹格式
 * @author: lkc
 */
bool Motor_Timing_TraceIsFault(void);

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
bool Motor_Timing_FaultTraceDue(uint32_t time_us, bool running, bool force);

/**
 * @description: 复制一条故障轨迹样本。
 * @detail: 只在已武装且未冻结时写入。sample 不可为空，违反约定触发 COMMON_ASSERT。故障冻结后不覆盖。
 * @param {const motor_fault_trace_sample_t *} sample 本拍样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_FaultTracePush(const motor_fault_trace_sample_t *sample);

/**
 * @description: 冻结故障轨迹，并给最后一条标故障码。
 * @detail: ISR 在故障处理复位前调用。fault 非 0 时写入最后一条 flags 的高 16 位。
 *          停机任务只在已经停机时传 0，只冻结、不改最后一条。未在采集则返回。
 * @param {unsigned} fault 故障码；0 表示只冻结
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_FaultTraceFreeze(unsigned fault);

/**
 * @description: 按时间顺序读取一条已冻结的故障轨迹。
 * @detail: 未冻结、不是故障格式、sample 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index                      时间顺序下标
 * @param {motor_fault_trace_sample_t *} sample 输出；空则失败
 * @return {bool} true=已写入 sample
 * @author: lkc
 */
bool Motor_Timing_FaultTraceGet(unsigned index, motor_fault_trace_sample_t *sample);
#endif
#define MOTOR_TRACE_COMPACT_CAPACITY \
    (MOTOR_TRACE_CAPACITY * sizeof(motor_trace_sample_t) / sizeof(motor_trace_compact_sample_t))
#define MOTOR_TRACE_COMPACT_STRIDE 30U
#define MOTOR_TRACE_COMPACT_POST 16U
#define MOTOR_TRACE_COMPACT_TASK_TIMEOUT_US 10000U
#define MOTOR_TRACE_COMPACT_LOSS 128U
#define MOTOR_TRACE_COMPACT_ENTRY 256U
#define MOTOR_TRACE_COMPACT_FAULT 512U
#define MOTOR_TRACE_COMPACT_TASK_GAP 1024U
#define MOTOR_TRACE_COMPACT_STOP 2048U
#if MOTOR_CFG_DIAG
/**
 * @description: 停机时武装紧凑飞行记录。
 * @detail: 复用完整缓冲，每条 32 字节。默认关闭性能统计，CadenceReset 可再打开。
 *          正常每 30 个控制拍写一条。首次 CLOSED 当拍做标记；失锁、故障或任务超过 10 ms 后再留 16 条。
 *          停机立即冻结。flags 低 2 位为状态，高 16 位为故障码。不改控制。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArmCompact(void);

/**
 * @description: 查询当前记录格式是否为紧凑轨迹。
 * @detail: 故障轨迹复用同一缓冲，此时返回 false。
 * @return {bool} true=紧凑格式且不是故障轨迹
 * @author: lkc
 */
bool Motor_Timing_TraceIsCompact(void);

/**
 * @description: 查询紧凑轨迹是否仍在采集。
 * @return {bool} true=紧凑格式且未冻结
 * @author: lkc
 */
bool Motor_Timing_TraceCompactIsActive(void);

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
bool Motor_Timing_TraceCompactDue(uint32_t time_us, unsigned state, unsigned fault, bool running);

/**
 * @description: 写入一条紧凑轨迹，并附上本拍事件标志。
 * @detail: sample 不可为空，违反约定触发 COMMON_ASSERT。未在采集则返回。
 *          停机或尾段配额用完后清除采集开关。不改控制。
 * @param {const motor_trace_compact_sample_t *} sample 本拍样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TracePushCompact(const motor_trace_compact_sample_t *sample);

/**
 * @description: 按时间顺序读取一条已冻结的紧凑轨迹。
 * @detail: 仍在采集、不是紧凑格式、sample 为空或下标越界时返回 false。下标 0 为最旧。
 * @param {unsigned} index                        时间顺序下标
 * @param {motor_trace_compact_sample_t *} sample 输出；空则失败
 * @return {bool} true=已写入 sample
 * @author: lkc
 */
bool Motor_Timing_TraceGetCompact(unsigned index, motor_trace_compact_sample_t *sample);
#endif

/* 轻量记录：96字节，附速度任务证据；仍小于完整记录，不增大union缓冲。 */
typedef struct {
    uint32_t time_us, flags;
    float theta, pll, flux, rpm;
    float id, iq, id_cmd, iq_cmd;
    float blend, loss, v_alpha, v_beta;
    uint16_t gap_max_us, full_max_us;
    uint32_t task_count, task_age_us, task_gate;
    float speed_target, speed_feedback, speed_iq;
    uint32_t task_tick, task_phase, task_phase_age_us;
} motor_trace_light_sample_t;

#if MOTOR_CFG_DIAG
/**
 * @description: 按接管触发方式武装轻量轨迹。
 * @detail: 样本比完整记录短，仍复用同一缓冲。触发后每 8 拍记录，配额 384 条。不停止电机。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArmLight(void);

/**
 * @description: 武装飞行记录，复用轻量缓冲。
 * @detail: 首次接管保留 192 条，其余 320 条循环覆盖。离开 CLOSED 的当拍冻结。不停止电机。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArmFlight(void);

/**
 * @description: 查询当前记录是否为飞行格式。
 * @return {bool} true=飞行记录
 * @author: lkc
 */
bool Motor_Timing_TraceIsFlight(void);

/**
 * @description: 返回飞行记录里固定保留的接管条数。
 * @detail: 最多 192。循环区条数不在这里。
 * @return {unsigned} 固定区条数
 * @author: lkc
 */
unsigned Motor_Timing_TraceFlightEntryCount(void);

/**
 * @description: 查询当前记录格式是否为轻量轨迹。
 * @return {bool} true=轻量格式
 * @author: lkc
 */
bool Motor_Timing_TraceIsLight(void);

/**
 * @description: 写入一条轻量或飞行轨迹。
 * @detail: sample 不可为空，违反约定触发 COMMON_ASSERT。未武装则返回。
 *          飞行格式先填满 192 条固定区，再在其余 320 条里循环；离开 CLOSED 的当拍冻结。
 *          普通轻量格式按配额停止。不改控制。
 * @param {const motor_trace_light_sample_t *} sample 本拍样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TracePushLight(const motor_trace_light_sample_t *sample);

/**
 * @description: 按时间顺序读取一条已停止的轻量或飞行轨迹。
 * @detail: 仍在采集、不是轻量格式、sample 为空或下标越界时返回 false。下标 0 为最旧。
 *          飞行格式先读固定区，再读循环区。
 * @param {unsigned} index                      时间顺序下标
 * @param {motor_trace_light_sample_t *} sample 输出；空则失败
 * @return {bool} true=已写入 sample
 * @author: lkc
 */
bool Motor_Timing_TraceGetLight(unsigned index, motor_trace_light_sample_t *sample);

/**
 * @description: 复位环形诊断索引并开始记录。
 * @detail: 只清下标、条数和触发标志，不擦除样本数组，也不检查电机是否在转。
 *          未触发时 Due 每 30 次调用放行一次。曾进入 CLOSED(2) 后再离开时触发，触发后连同当拍再收 16 条并停止。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArm(void);

/**
 * @description: 按接管触发方式武装环形诊断。
 * @detail: 先做与 TraceArm 相同的复位，再改为首次 state==2 时触发。触发后连同当拍再收 384 条并停止。
 *          容量 512，缓冲区已经记满时，触发后大约还留 128 条触发前样本。本函数不停止电机。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArmEntry(void);
/**
 * @description: 首次 CLOSED 后逐控制拍记录 384 条。
 * @detail: 在接管触发方式上改为触发后每拍放行。收满后冻结。不停止电机，不改控制节拍。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArmEntryFast(void);

/**
 * @description: 停止继续写入环形诊断。
 * @detail: 只清除 active。不检查停机，不清样本。停止后 Get 可以读取；收满触发后样本也会自行清除 active。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceStop(void);

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
bool Motor_Timing_TraceDue(unsigned state);

/**
 * @description: 写入一条环形诊断，并附上当前窗口的间隔统计。
 * @detail: sample 不可为空，违反约定触发 COMMON_ASSERT。未在采集或当前是轻量/紧凑格式时返回。
 *          写入后用窗口里的 gap/full/pending/up 覆盖样本对应字段并清窗口。
 *          失锁触发给 flags 加 bit7(128)，接管触发加 bit8(256)。配额减到 0 时清除 active。
 * @param {const motor_trace_sample_t *} sample 本拍样本；不可为空，违反约定触发 COMMON_ASSERT
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TracePush(const motor_trace_sample_t *sample);

/**
 * @description: 返回当前已写入、尚未被新数据覆盖的条数。
 * @detail: 仍在记录时也会增长，最大为 MOTOR_TRACE_CAPACITY。条数不等于 Get 已经允许读取。
 * @return {unsigned} 条数
 * @author: lkc
 */
unsigned Motor_Timing_TraceCount(void);

/**
 * @description: 按时间顺序读取一条已经停止写入的环形诊断。
 * @detail: active 仍为 true、sample 为空或 index 越界时返回 false。停止可以来自 TraceStop，也可以来自配额用完。
 *          下标 0 是当前缓冲区里最旧的一条。本函数不输出串口。
 * @param {unsigned} index                时间顺序下标，0 为最旧
 * @param {motor_trace_sample_t *} sample 输出，不可为空
 * @return {bool} true=已写入 sample
 * @author: lkc
 */
bool Motor_Timing_TraceGet(unsigned index, motor_trace_sample_t *sample);
#else
/* 诊断关闭：武装命令不生效，Due 恒为 false，控制路径不再填样本。 */
static inline bool Motor_Timing_FaultTraceDue(uint32_t time_us, bool running, bool force)
{
    (void)time_us;
    (void)running;
    (void)force;
    return false;
}
static inline void Motor_Timing_FaultTracePush(const motor_fault_trace_sample_t *sample) { (void)sample; }
static inline void Motor_Timing_FaultTraceFreeze(unsigned fault) { (void)fault; }
static inline bool Motor_Timing_TraceIsCompact(void) { return false; }
static inline bool Motor_Timing_TraceCompactIsActive(void) { return false; }
static inline bool Motor_Timing_TraceCompactDue(uint32_t time_us, unsigned state, unsigned fault, bool running)
{
    (void)time_us;
    (void)state;
    (void)fault;
    (void)running;
    return false;
}
static inline void Motor_Timing_TracePushCompact(const motor_trace_compact_sample_t *sample) { (void)sample; }
static inline void Motor_Timing_TraceArmLight(void) {}
static inline void Motor_Timing_TraceArmFlight(void) {}
static inline bool Motor_Timing_TraceIsLight(void) { return false; }
static inline void Motor_Timing_TracePushLight(const motor_trace_light_sample_t *sample) { (void)sample; }
static inline void Motor_Timing_TraceArm(void) {}
static inline void Motor_Timing_TraceArmEntry(void) {}
static inline void Motor_Timing_TraceArmEntryFast(void) {}
static inline void Motor_Timing_TraceStop(void) {}
static inline bool Motor_Timing_TraceDue(unsigned state) { (void)state; return false; }
static inline void Motor_Timing_TracePush(const motor_trace_sample_t *sample) { (void)sample; }
#endif

#ifdef __cplusplus
}
#endif
#endif
