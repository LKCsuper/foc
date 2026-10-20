/*
 * @Description: 控制中断、唤醒探针、任务运行时、速度任务与回调节拍诊断
 * @Author: lkc
 */
#ifndef MOTOR_DIAG_SCHEDULE_H
#define MOTOR_DIAG_SCHEDULE_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "motor_config.h"
#include "motor_config.h"
#include "motor_param.h"
#include "motor_diag_trace.h"

/**
 * @description: 通过控制门控后的入口间隔统计。
 * @detail: 不修改算法 dt。有效间隔最多 100 万个；单间隔大于 1000 us 只计入 gaps。
 */
typedef struct {
    uint32_t count;   /* 有效间隔数 */
    uint32_t sum_us;  /* 间隔累计 (us) */
    uint32_t min_us;  /* 最短间隔 (us) */
    uint32_t max_us;  /* 最长间隔 (us) */
    uint32_t up;      /* 向上计数侧次数 */
    uint32_t down;    /* 向下计数侧次数 */
    uint32_t gaps;    /* 超过 1000 us 而丢弃的间隔数 */
    uint32_t bins[8]; /* <=50,60,73,90,110,150,250,>250 us */
} motor_cadence_t;

#if MOTOR_CFG_DIAG
/**
 * @description: 清零节拍、回调和分段耗时窗口，并重新允许采集。
 * @detail: 临界区内清零。不清环形诊断，也不清上一次时间基准；之后第一次 Note/CallbackBegin 丢掉首个间隔。
 *          不修改算法 dt。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CadenceReset(void);
/**
 * @description: 清零并只开启速度任务统计。
 * @detail: 先做 CadenceReset，再关闭回调统计、性能统计和分段采样。不改变轨迹模式或控制参数。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskReset(void);

/**
 * @description: 停止节拍和 DMA 回调累加。
 * @detail: 临界区内关闭性能统计、分段采样、唤醒探测、节拍和回调累加。已有统计和环形诊断保留。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CadenceStop(void);

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
void Motor_Timing_CadenceNote(uint32_t time_us, bool upward);

/**
 * @description: 拷贝控制入口节拍统计。
 * @detail: 本函数进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_cadence_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CadenceGet(motor_cadence_t *out);
#endif

/**
 * @description: Begin 到 End 之间的 DMA 回调耗时统计。
 * @detail: 当前调用点包住整个 Motor_DmaInt，所以这段时间含状态发布。pending 是 End 参数，
 *          表示返回时同一 DMA 标志再次置位。统计本身不改清标志或触发顺序。
 */
typedef struct {
    uint32_t count;          /* 已结束回调次数 */
    uint32_t interval_count; /* 有效入口间隔数 */
    uint32_t interval_sum;   /* 入口间隔累计 (us) */
    uint32_t interval_max;   /* 最长入口间隔 (us) */
    uint32_t gaps;           /* 超过 1000 us 的入口间隔数 */
    uint32_t bins[6];        /* <=25,40,60,75,110,>110 us */
    uint32_t full;           /* 完整控制拍的回调次数 */
    uint32_t full_sum;       /* 完整控制拍回调耗时累计 (us) */
    uint32_t full_max;       /* 完整控制拍回调最长耗时 (us) */
    uint32_t skip_sum;       /* 跳过控制拍的回调耗时累计 (us) */
    uint32_t skip_max;       /* 跳过控制拍的回调最长耗时 (us) */
    uint32_t over33;         /* 耗时超过 33 us 的次数 */
    uint32_t over66;         /* 耗时超过 66 us 的次数 */
    uint32_t pending;        /* 返回时同一 DMA 标志再次置位的次数 */
    uint32_t publish;        /* 本回调内实际发布状态帧的次数 */
    uint32_t publish_sum;    /* 状态发布自身耗时累计 (us)，已含在 full 或 skip 内 */
    uint32_t publish_max;    /* 单次状态发布最长耗时 (us) */
    uint32_t publish_on_full; /* 状态发布落在完整控制拍的次数 */
} motor_callback_t;

#if MOTOR_CFG_DIAG
/**
 * @description: 标记一次 DMA 回调开始，并统计回调入口间隔。
 * @detail: 随 CadenceReset 启用；统计和轨迹均关闭则立即返回。轨迹启用时仍保存起点并清除 full，
 *          之后若本回调执行到 CadenceNote 会再标成 true。第一次调用只建立时间基准。
 *          间隔大于 1000 us 只加 gaps。本函数不读 DMA 标志。
 * @param {uint32_t} time_us 回调入口时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackBegin(uint32_t time_us);

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
void Motor_Timing_CallbackEnd(uint32_t time_us, bool pending);

/**
 * @description: 标记本回调是完整控制拍还是跳过拍。
 * @detail: 仅细分诊断开启时由 FOC 入口调用。须在 CallbackEnd 之前。不改控制。
 * @param {bool} full true=本回调执行了完整控制
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackClassify(bool full);

/**
 * @description: 记下本回调内一次状态发布的自身耗时。
 * @detail: elapsed_us 只含快照发布，不含分频计数。同一回调只记最后一次。不改控制。
 * @param {uint32_t} elapsed_us 状态发布耗时 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackPublish(uint32_t elapsed_us);
#endif

/**
 * @description: 同一运行窗口内的拍型耗时，以及它们落在任务唤醒等待中的部分。
 * @detail: full/skip 的唤醒累计是中断函数体时间，状态发布是其中的子集，不能与 full/skip 相加。
 *          worst_* 来自同一次最长唤醒，worst_full_us 与 worst_skip_us 之和等于该次 DMA 累计。
 */
typedef struct {
    uint32_t publish_count, publish_sum_us, publish_max_us, publish_on_full;
    uint32_t wake_count, wake_delay_sum_us, wake_dma_sum_us;
    uint32_t wake_full_count, wake_full_sum_us;
    uint32_t wake_skip_count, wake_skip_sum_us;
    uint32_t wake_publish_count, wake_publish_sum_us;
    uint32_t worst_delay_us, worst_dma_us;
    uint32_t worst_full_count, worst_full_us;
    uint32_t worst_skip_count, worst_skip_us;
    uint32_t worst_publish_count, worst_publish_us;
} motor_split_diag_t;

#if MOTOR_CFG_DIAG
/**
 * @description: 打开完整拍、跳过拍和状态发布的同窗口细分，并武装唤醒重叠。
 * @detail: 关闭性能统计和分段采样，保留回调耗时与速度任务节拍。不改控制周期、增益或任务优先级。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SplitReset(void);

/**
 * @description: 拷贝拍型细分与唤醒重叠。
 * @detail: 进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_split_diag_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SplitGet(motor_split_diag_t *out);

/** @brief 细分诊断开关。关闭时 FOC 入口不调用分类。 */
extern volatile uint8_t g_motor_timing_split_active;

/**
 * @description: 查询拍型细分诊断是否开启。
 * @return {bool} true=FOC 入口应调用 CallbackClassify/CallbackPublish
 */
static inline bool Motor_Timing_SplitIsActive(void) { return g_motor_timing_split_active != 0U; }

/**
 * @description: 拷贝 DMA 回调耗时统计。
 * @detail: 本函数进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_callback_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackGet(motor_callback_t *out);

/**
 * @description: 关闭 DMA 回调耗时统计。
 * @detail: 进入临界区，只清回调统计开关。已有统计保留。BSP 可据此省去两次时间戳读取。不影响 IRQ 总占用统计。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackStop(void);

/**
 * @description: 查询回调起点是否仍需记录。
 * @detail: 回调统计开启，或非紧凑轨迹仍在采集时返回 true。不改统计。
 * @return {bool} true=应调用 CallbackBegin
 * @author: lkc
 */
bool Motor_Timing_CallbackIsActive(void);
#endif

typedef struct {
    uint32_t valid, delay_us, dma_count, dma_us;
} motor_wake_probe_t;

typedef struct {
    uint32_t sequence, time_us, previous_us;
    motor_wake_probe_t wake, previous_wake;
    uint32_t phase_valid, before_wait_us, phase_us[4];
} motor_task_gap_event_t;

typedef struct {
    uint32_t count, last_us, max_gap_us;
    bool allowed;
    float target, feedback, iq;
    uint32_t tick, phase, phase_us, complete_tick, max_gap_ticks;
    uint32_t phase_max_us[4], phase_max_ticks[4];
    uint32_t deadline_tick, wake_late_max_ticks;
    uint32_t pi_count, pi_no_update, pi_last_us, pi_max_gap_us;
    uint64_t pi_sum_gap_us;
    uint32_t pi_gap_bins[6];
    uint32_t pi_over_1ms, pi_over_streak, pi_over_streak_max;
    motor_wake_probe_t worst_pi_wake, worst_wake;
} motor_speed_task_diag_t;

typedef struct {
    uint32_t magic, version, sequence, state;
    uint32_t mode;
    uint32_t task_count, task_max_gap_us, pi_count, pi_no_update, pi_max_gap_us;
    uint32_t pi_over_1ms, pi_streak_max, wake_late_max_ticks;
    uint32_t phase_max_us[4], pi_gap_bins[6];
    motor_wake_probe_t worst_pi_wake, worst_wake;
    uint32_t irq[2][8];
    uint32_t task_gap_bins[6];
    motor_task_gap_event_t worst_gap;
} motor_schedule_debug_t;

extern volatile motor_schedule_debug_t g_motor_schedule_debug;

typedef struct {
    uint32_t count, min_us, max_us, last_us;
    uint64_t sum_us;
    uint64_t dma_sum_us;
    uint32_t max_wall_dma_us, max_wall_dma_count;
} motor_task_runtime_debug_t;

extern volatile motor_task_runtime_debug_t g_motor_task_runtime_debug;

#if MOTOR_CFG_DIAG
/** @brief 开始一次外环任务执行计时，不包含周期休眠。 */
void Motor_Timing_TaskRuntimeBegin(void);
/** @brief 结束本轮计时，累计包含中断抢占的墙钟耗时。 */
void Motor_Timing_TaskRuntimeEnd(void);

/**
 * @description: 停止调度统计并发布供调试器读取的固定布局快照。
 * @detail: 仅供停机后的命令任务调用；不停止电机。state=2 且 sequence 为偶数才有效。
 *          mode bit0=任务模式、bit1=唤醒探测、bit2=性能计时、bit3=冻结前采集开启。
 *          IRQ 每行依次为 seen/count/sum/max/first/last/tick_pending/switch_pending。
 * @return {void} 无
 */
void Motor_Timing_ScheduleDebugFreeze(void);
/**
 * @description: 打开目标 tick 到任务唤醒的 DMA 探测。
 * @detail: 先做 SpeedTaskReset，再允许 WakeProbeArm。探测是插桩，不改任务调度。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_WakeProbeReset(void);

/**
 * @description: 武装一轮从截止 tick 到任务唤醒的 DMA 计时。
 * @detail: 仅在 WakeProbeReset 打开探测后生效。不改任务调度。
 * @param {uint32_t} deadline_tick 本轮 osDelayUntil 截止 tick
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_WakeProbeArm(uint32_t deadline_tick);

/**
 * @description: 查询目标 tick 到任务唤醒的 DMA 探测是否已武装。
 * @return {bool} true=已武装
 * @author: lkc
 */
bool Motor_Timing_WakeProbeArmed(void);

/**
 * @description: 在截止 tick 的 SysTick 入口记下探测起点。
 * @detail: 只记第一次命中。不改任务调度。
 * @param {uint32_t} next_tick 即将发生的 tick
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_WakeProbeTick(uint32_t next_tick);

/**
 * @description: 查询本轮探测是否已经看到截止 tick，可以累计 DMA。
 * @return {bool} true=应调用 WakeProbeDmaNote
 * @author: lkc
 */
bool Motor_Timing_WakeProbeDmaDue(void);

/**
 * @description: 累计截止 tick 之后、任务唤醒之前的一次 DMA 耗时。
 * @detail: 未到采集窗口则返回。不改 PWM 或任务调度。
 * @param {uint32_t} start_us DMA 入口时间 (us)
 * @param {uint32_t} end_us   DMA 返回时间 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_WakeProbeDmaNote(uint32_t start_us, uint32_t end_us);
#endif
typedef struct {
    uint32_t count, first_us, last_us, sum_us, max_us, tick_pending, switch_pending;
    uint32_t seen;
} motor_irq_diag_t;
#define MOTOR_TIMING_IRQ_STRIDE 31U
#if MOTOR_CFG_DIAG
/**
 * @description: 判断本次 IRQ 是否允许记一次耗时。
 * @detail: 节拍统计开启且不是只统计速度任务时，每次调用都加总次数。每 31 次放行一次计时。
 *          kind 0 为 DMA 函数体，1 为 SysTick 函数体；其他值返回 false。CadenceReset 清零分频。不改控制。
 * @param {unsigned} kind 0=DMA，1=SysTick
 * @return {bool} true=本拍应调用 IrqNote
 * @author: lkc
 */
bool Motor_Timing_IrqDue(unsigned kind);

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
void Motor_Timing_IrqNote(unsigned kind, uint32_t start_us, uint32_t end_us, unsigned pending);

/**
 * @description: 拷贝一类 IRQ 耗时统计。
 * @detail: 进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。kind 非法则不写输出。
 * @param {unsigned} kind          0=DMA，1=SysTick
 * @param {motor_irq_diag_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_IrqGet(unsigned kind, motor_irq_diag_t *out);

/**
 * @description: 记录速度任务阶段切换耗时。
 * @detail: phase 1 为等待周期，2 为已唤醒，3 为位置环完成，4 为速度调用完成。
 *          只在相邻阶段前进时更新上一段峰值。未开启或 phase 超出 1..4 则返回。不改调度。
 * @param {uint32_t} time_us 阶段时间 (us)
 * @param {uint32_t} tick    任务看到的 tick
 * @param {uint32_t} phase   阶段号，1..4
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskStage(uint32_t time_us, uint32_t tick, uint32_t phase);

/**
 * @description: 记录任务唤醒，并发布阶段 2。
 * @detail: 比较实际 tick 和本轮 osDelayUntil 截止 tick，只保留最晚的正偏差。
 *          若探测已看到同一截止 tick，记下唤醒延迟和期间 DMA 累计。不改任务调度。
 * @param {uint32_t} time_us       唤醒时间 (us)
 * @param {uint32_t} tick          实际 tick
 * @param {uint32_t} deadline_tick 本轮截止 tick
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskWake(uint32_t time_us, uint32_t tick, uint32_t deadline_tick);

/**
 * @description: 记录一次外环调用完成。
 * @detail: 先写心跳。节拍统计开启时更新完成间隔，并保存门控、目标、反馈和 Iq。
 *          allowed 是运行门控，不是 PI 实际更新次数。不改控制。
 * @param {uint32_t} time_us 完成时间 (us)
 * @param {bool} allowed     本拍外环是否允许输出
 * @param {float} target     速度目标 (RPM)
 * @param {float} feedback   速度反馈 (RPM)
 * @param {float} iq         速度环 Iq (A)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskNote(uint32_t time_us, bool allowed, float target, float feedback, float iq);

/**
 * @description: 拷贝速度任务诊断。
 * @detail: 进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_speed_task_diag_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskGet(motor_speed_task_diag_t *out);

/**
 * @description: 查询速度任务统计是否开启。
 * @detail: 供调用方跳过不必要的诊断参数采集。不改控制。
 * @return {bool} true=节拍统计开启
 * @author: lkc
 */
bool Motor_Timing_SpeedTaskIsActive(void);

/**
 * @description: 发布一次速度任务完成心跳。
 * @detail: 每次任务完成都写，供紧凑记录判断超时。不启用其他统计，不改调度。
 * @param {uint32_t} time_us 任务完成时间 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedTaskHeartbeat(uint32_t time_us);

/**
 * @description: 记录速度 PI 是否在本拍实际更新。
 * @detail: 仅节拍统计开启时计入。间隔包含跳拍和禁用前的时间。超过 1 ms 计一次超时。不改 PI。
 * @param {uint32_t} time_us 本次观察时间 (us)
 * @param {bool} updated     true=本拍 PI 序号已前进
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_SpeedPiNote(uint32_t time_us, bool updated);
/**
 * @brief 在短临界区仅填充轻量轨迹需要的任务字段。
 * @param sample 非空，本拍 time_us 已填写；其余非任务字段保持不变。
 */
void Motor_Timing_SpeedTaskCaptureLight(motor_trace_light_sample_t *sample);
#else
/* 诊断关闭：节拍/唤醒/任务统计恒为未开启，ISR 和任务里的调用编译为空。 */
static inline void Motor_Timing_CadenceStop(void) {}
static inline void Motor_Timing_CadenceNote(uint32_t time_us, bool upward)
{
    (void)time_us;
    (void)upward;
}
static inline void Motor_Timing_CallbackBegin(uint32_t time_us) { (void)time_us; }
static inline void Motor_Timing_CallbackEnd(uint32_t time_us, bool pending)
{
    (void)time_us;
    (void)pending;
}
static inline void Motor_Timing_CallbackClassify(bool full) { (void)full; }
static inline void Motor_Timing_CallbackPublish(uint32_t elapsed_us) { (void)elapsed_us; }
static inline bool Motor_Timing_SplitIsActive(void) { return false; }
static inline bool Motor_Timing_CallbackIsActive(void) { return false; }
static inline void Motor_Timing_TaskRuntimeBegin(void) {}
static inline void Motor_Timing_TaskRuntimeEnd(void) {}
static inline void Motor_Timing_WakeProbeArm(uint32_t deadline_tick) { (void)deadline_tick; }
static inline bool Motor_Timing_WakeProbeArmed(void) { return false; }
static inline void Motor_Timing_WakeProbeTick(uint32_t next_tick) { (void)next_tick; }
static inline bool Motor_Timing_WakeProbeDmaDue(void) { return false; }
static inline void Motor_Timing_WakeProbeDmaNote(uint32_t start_us, uint32_t end_us)
{
    (void)start_us;
    (void)end_us;
}
static inline bool Motor_Timing_IrqDue(unsigned kind) { (void)kind; return false; }
static inline void Motor_Timing_IrqNote(unsigned kind, uint32_t start_us, uint32_t end_us, unsigned pending)
{
    (void)kind;
    (void)start_us;
    (void)end_us;
    (void)pending;
}
static inline void Motor_Timing_SpeedTaskStage(uint32_t time_us, uint32_t tick, uint32_t phase)
{
    (void)time_us;
    (void)tick;
    (void)phase;
}
static inline void Motor_Timing_SpeedTaskWake(uint32_t time_us, uint32_t tick, uint32_t deadline_tick)
{
    (void)time_us;
    (void)tick;
    (void)deadline_tick;
}
static inline void Motor_Timing_SpeedTaskNote(uint32_t time_us, bool allowed, float target, float feedback, float iq)
{
    (void)time_us;
    (void)allowed;
    (void)target;
    (void)feedback;
    (void)iq;
}
static inline bool Motor_Timing_SpeedTaskIsActive(void) { return false; }
static inline void Motor_Timing_SpeedTaskHeartbeat(uint32_t time_us) { (void)time_us; }
static inline void Motor_Timing_SpeedPiNote(uint32_t time_us, bool updated)
{
    (void)time_us;
    (void)updated;
}
static inline void Motor_Timing_SpeedTaskCaptureLight(motor_trace_light_sample_t *sample) { (void)sample; }
#endif

#ifdef __cplusplus
}
#endif
#endif
