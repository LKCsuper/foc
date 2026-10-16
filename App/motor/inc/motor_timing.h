/*
 * @Description: 电机控制时间基准与 ISR 耗时统计
 * @Author: lkc
 */
#ifndef MOTOR_TIMING_H
#define MOTOR_TIMING_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* ADC 每个中心对齐 PWM 周期采样两次，FOC 默认每个完整 PWM 周期运行一次。 */
#ifndef FOC_CONTROL_LOOP_FREQ_DIVIDER
#define FOC_CONTROL_LOOP_FREQ_DIVIDER 2
#endif
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint32_t last_us;
    bool seeded;
} motor_control_clock_t;
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
/** 停机武装；复用32条缓冲，每450拍抽取连续两帧。回调寄存器不是ADC触发锁存值。 */
void Motor_Timing_PairArm(void);
/** 仅停机原始半周期回调配对；901回调抽样。起转时停止诊断，不阻止控制；状态位复用reconstructed。 */
void Motor_Timing_PairHalfArm(void);
bool Motor_Timing_PairHalfMode(void);
bool Motor_Timing_PairMode(void);
bool Motor_Timing_PairActive(void);
/** 每个选中FOC拍一次，写入重构后电流；返回true时同拍调用PairPush。 */
bool Motor_Timing_PairFrame(const motor_pair_frame_t *frame);
void Motor_Timing_PairPush(float v_alpha, float v_beta);
bool Motor_Timing_PairGet(unsigned index, motor_pair_sample_t *out);
/** 停机武装，复用32条缓冲；450拍抽样，完成拍后的首拍触发，再留27条。 */
void Motor_Timing_ObserverStepArm(void);
bool Motor_Timing_ObserverStepMode(void);
/** 默认实例更新前调用；Active用于维护连续前一拍电流，Due只推进一次抽样计数。 */
bool Motor_Timing_ObserverStepActive(void);
bool Motor_Timing_ObserverStepDue(void);
void Motor_Timing_ObserverStepPush(const motor_observer_step_sample_t *sample);
bool Motor_Timing_ObserverStepGet(unsigned index, motor_observer_step_sample_t *out);
/** 与交接积分记录互斥复用缓冲，PLL更新后记录同拍输入/输出。 */
void Motor_Timing_PhaseArm(void);
/** 停机调用；复用32条缓冲，450拍抽样，完成拍及其后27条自动冻结；不保证捕获拍间尖峰。 */
void Motor_Timing_PhaseArmLong(void);
bool Motor_Timing_PhaseMode(void);
bool Motor_Timing_PhaseDue(bool completed);
void Motor_Timing_PhasePush(const motor_phase_sample_t *sample);
bool Motor_Timing_PhaseGet(unsigned index, motor_phase_sample_t *out);
/** 停机时武装独立短窗口；每15个电流控制拍记录一次，完成交接拍强制记录并再留15条。 */
void Motor_Timing_HandoffArm(void);
void Motor_Timing_HandoffStop(void);
void Motor_Timing_HandoffMark(void);
bool Motor_Timing_HandoffDue(void);
/** 以下三接口仅在Due返回true后于同一电流控制拍依次调用；motor必须有效。 */
void Motor_Timing_HandoffBefore(const MOTOR_PARAM_S *motor, float theta, float delta, float id, float iq, float dt);
void Motor_Timing_HandoffTransport(const MOTOR_PARAM_S *motor);
void Motor_Timing_HandoffAfter(const MOTOR_PARAM_S *motor, float vd, float vq);
unsigned Motor_Timing_HandoffCount(void);
bool Motor_Timing_HandoffGet(unsigned index, motor_handoff_sample_t *out);

/**
 * 完整控制入口间隔换算为秒。首次启用使用名义周期；关闭时清除历史。
 * 接受 0.5..3 倍名义周期，异常返回 false 且输出 0，调用者须停机。
 * 时间戳允许 uint32_t 回绕；入口间隔不等于 ADC 硬件采样时间戳。
 */
bool Motor_Timing_ControlClockStep(motor_control_clock_t *clock, bool enabled, uint32_t time_us,
                                   float nominal_dt, float *dt);

typedef enum {
    MOTOR_TIMING_SAMPLE,           /* SampleAndProtect 整段 */
    MOTOR_TIMING_CURRENT,          /* PrepareCurrentFeedback 整段 */
    MOTOR_TIMING_CONTROL,          /* RunControl 墙钟；内部又记了 OBSERVER 与 TRANSPORT */
    MOTOR_TIMING_OBSERVER,         /* Sensorless_Update 整段，含在 CONTROL 墙钟内 */
    MOTOR_TIMING_OUTPUT,           /* ArbitrateAndApply 整段 */
    MOTOR_TIMING_OBSERVER_BLEND,   /* CLOSED 且 blend<1 时，与 OBSERVER 同一段耗时再记一档 */
    MOTOR_TIMING_OBSERVER_CLOSED,  /* CLOSED 且 blend 已到 1 时，与 OBSERVER 同一段耗时再记一档 */
    MOTOR_TIMING_TRANSPORT,        /* frame_correction 非 0 时的 TransportPi，含在 CONTROL 墙钟内 */
    MOTOR_TIMING_CONFIG,           /* 每拍配置规范化，含在CONTROL内 */
    MOTOR_TIMING_STARTUP,          /* HFI检查与I/F启动推进，含在CONTROL内 */
    MOTOR_TIMING_DRAG_ANGLE,       /* 拖转电流、速度斜坡与强制角推进，不含观测器 */
    MOTOR_TIMING_SPEED_GATE,       /* 速度环就绪、预置或复位，不含观测器 */
    MOTOR_TIMING_IF_CURRENT,       /* RunIf整段：目标、弱磁、电流PI，含TRANSPORT */
    MOTOR_TIMING_CURRENT_PARK,     /* Clarke/Park，含在IF_CURRENT内 */
    MOTOR_TIMING_CURRENT_TARGET,   /* 电流目标计算，含在IF_CURRENT内 */
    MOTOR_TIMING_CURRENT_PI,       /* 双轴PI，不含矢量限幅 */
    MOTOR_TIMING_CURRENT_LIMIT,    /* 矢量限幅及积分抗饱和 */
    MOTOR_TIMING_SENSE_FRAME,
    MOTOR_TIMING_SENSOR_FEEDBACK,
    MOTOR_TIMING_PROTECTION,
    MOTOR_TIMING_STAGE_COUNT
} motor_timing_stage_t;
typedef struct {
    uint32_t count[MOTOR_TIMING_STAGE_COUNT];
    uint32_t sum_us[MOTOR_TIMING_STAGE_COUNT];
    uint32_t max_us[MOTOR_TIMING_STAGE_COUNT];
} motor_profile_snapshot_t;
/** 短临界区复制所有分段原始统计；平均值及打印在调用方关中断区外计算。 */
void Motor_Timing_ProfileSnapshot(motor_profile_snapshot_t *out);

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 推进分段耗时分频，并决定本拍是否采样。
 * @detail: 分频计数是 uint8_t，每次调用加 1，回绕为 0 时返回 true，即每 256 次调用采 1 次。
 *          调用方只在通过控制门控之后调用。本函数不读取耗时，也不改变控制量。
 * @return {bool} true=本拍后续 Record 会累加
 * @author: lkc
 */
bool Motor_Timing_ProfileBegin(void);

/**
 * @description: 返回本拍 Begin 写下的采样标志。
 * @detail: End 会把它清掉。保护失败提前返回时，Begin 可能已为 true，但各段尚未 Record。
 * @return {bool} true=本拍采样仍然有效
 * @author: lkc
 */
bool Motor_Timing_ProfileIsActive(void);
/** 细分计时默认关闭；开启只影响诊断采样，不改变控制周期。 */
void Motor_Timing_ProfileSetDetailed(bool enabled);
bool Motor_Timing_ProfileDetailedIsActive(void);

/**
 * @description: 读取分段计时用的当前时间戳。
 * @detail: 直接返回 Hal_Motor_TimeUs()，不改统计。
 * @return {uint32_t} 时间戳 (us)
 * @author: lkc
 */
uint32_t Motor_Timing_ProfileTimeUs(void);

/**
 * @description: 清除本拍分段采样标志。
 * @detail: 无条件写 false。Begin 返回 false 时再调用，结果仍是 false。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileEnd(void);

/**
 * @description: 把一个阶段从 start_us 到现在的耗时累进当前窗口。
 * @detail: 采样标志为 false、阶段号非法，或耗时大于 1000 us 时直接返回。
 *          该段样本数已经达到 256 时，先清掉该段的 sum/count/max，再计入本拍。
 *          跳过的 ADC 回调不会调用本函数。CONTROL 与 OBSERVER、TRANSPORT 的时间互相包含；
 *          BLEND/CLOSED 与 OBSERVER 是同一段耗时，平均值不能相加。
 * @param {motor_timing_stage_t} stage 阶段编号
 * @param {uint32_t} start_us          阶段开始时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileRecord(motor_timing_stage_t stage, uint32_t start_us);

/**
 * @description: 读出一个阶段当前窗口的样本数和峰值。
 * @detail: 本函数进入临界区。输出指针不可为空并由 COMMON_ASSERT 检查，stage 非法则直接返回。CadenceReset 会清这些数组。
 * @param {motor_timing_stage_t} stage 阶段编号
 * @param {uint32_t *} count           输出样本数，不可为空
 * @param {uint32_t *} maximum         输出峰值 (us)，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileDetail(motor_timing_stage_t stage, uint32_t *count, uint32_t *maximum);

/**
 * @description: 按各段 sum/count 写出平均耗时。
 * @detail: 本函数不加临界区。count 为 0 的段写 0。samples 写出的是 CONTROL 段当前 count，不是 Begin 次数。
 * @param {float[MOTOR_TIMING_STAGE_COUNT]} average_us 输出平均耗时 (us)，不可为空
 * @param {uint32_t *} samples                          输出 CONTROL 段样本数，可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileGet(float average_us[MOTOR_TIMING_STAGE_COUNT], uint32_t *samples);

#define MOTOR_TRACE_CAPACITY 512U

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

/**
 * @description: 清零节拍、回调和分段耗时窗口，并重新允许采集。
 * @detail: 临界区内清零。不清环形诊断，也不清上一次时间基准；之后第一次 Note/CallbackBegin 丢掉首个间隔。
 *          不修改算法 dt。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CadenceReset(void);
/** 清零并只开启速度任务统计，关闭高频性能统计；不改变轨迹模式或控制参数。 */
void Motor_Timing_SpeedTaskReset(void);

/**
 * @description: 停止节拍和 DMA 回调累加。
 * @detail: 临界区内只清两个使能标志。已有统计和环形诊断都保留。
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
} motor_callback_t;

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
 * @description: 拷贝 DMA 回调耗时统计。
 * @detail: 本函数进入临界区。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_callback_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackGet(motor_callback_t *out);

/** 关闭回调统计可省去 BSP 中两次时间戳读取；不影响 IRQ 总占用统计。 */
void Motor_Timing_CallbackStop(void);
bool Motor_Timing_CallbackIsActive(void);

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
/** 停机时武装32字节飞行记录，复用完整缓冲；默认停用性能统计，CadenceReset可显式恢复。
 * 正常每30控制拍写一条；首次CLOSED当拍标记，失锁/故障/任务超10ms留16条尾段，停机立即冻结。
 * flags低2位为状态，高16位为故障码；中间事件位见宏。时间us、角度rad、转速RPM、电流A。
 */
void Motor_Timing_TraceArmCompact(void);
bool Motor_Timing_TraceIsCompact(void);
bool Motor_Timing_TraceCompactIsActive(void);
bool Motor_Timing_TraceCompactDue(uint32_t time_us, unsigned state, unsigned fault, bool running);
void Motor_Timing_TracePushCompact(const motor_trace_compact_sample_t *sample);
/** 仅冻结后可按时间顺序读取；输出空、下标越界或格式不匹配则返回false。 */
bool Motor_Timing_TraceGetCompact(unsigned index, motor_trace_compact_sample_t *sample);

typedef struct {
    uint32_t valid, delay_us, dma_count, dma_us;
} motor_wake_probe_t;

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
/** 显式启用目标tick入口到任务唤醒的DMA计时；probe属于插桩模式。 */
void Motor_Timing_WakeProbeReset(void);
void Motor_Timing_WakeProbeArm(uint32_t deadline_tick);
bool Motor_Timing_WakeProbeArmed(void);
void Motor_Timing_WakeProbeTick(uint32_t next_tick);
bool Motor_Timing_WakeProbeDmaDue(void);
void Motor_Timing_WakeProbeDmaNote(uint32_t start_us, uint32_t end_us);
typedef struct {
    uint32_t count, first_us, last_us, sum_us, max_us, tick_pending, switch_pending;
    uint32_t seen;
} motor_irq_diag_t;
#define MOTOR_TIMING_IRQ_STRIDE 31U
/** IRQ入口调用；诊断开启时统计总次数，每31次允许一次计时。不同kind独立分频，reset清零。 */
bool Motor_Timing_IrqDue(unsigned kind);
/* kind=0 DMA函数体，1 SysTick函数体。pending bit0=SysTick，bit1=PendSV。
 * SysTick包含DMA抢占，不能相加；不包括异常入/出栈及Note自身。 */
void Motor_Timing_IrqNote(unsigned kind, uint32_t start_us, uint32_t end_us, unsigned pending);
void Motor_Timing_IrqGet(unsigned kind, motor_irq_diag_t *out);
/* phase: 1等待周期，2已唤醒，3位置环完成，4速度调用完成；tick由任务传入。 */
void Motor_Timing_SpeedTaskStage(uint32_t time_us, uint32_t tick, uint32_t phase);
/** 唤醒时发布阶段2，并比较实际tick和本轮osDelayUntil截止tick；不改变任务调度。 */
void Motor_Timing_SpeedTaskWake(uint32_t time_us, uint32_t tick, uint32_t deadline_tick);
/* 任务完成一次外环调用后发布；运行门控不是PI实际更新计数。 */
void Motor_Timing_SpeedTaskNote(uint32_t time_us, bool allowed, float target, float feedback, float iq);
void Motor_Timing_SpeedTaskGet(motor_speed_task_diag_t *out);
/** 查询任务统计是否开启，供调用方跳过不必要的诊断参数采集。 */
bool Motor_Timing_SpeedTaskIsActive(void);
/** 每次任务完成都发布心跳，供紧凑记录超时触发；不启用其他统计。 */
void Motor_Timing_SpeedTaskHeartbeat(uint32_t time_us);
/** 任务比较实际PI序号后发布；仅cadence开启时统计，间隔包含跳拍及禁用时间。 */
void Motor_Timing_SpeedPiNote(uint32_t time_us, bool updated);

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
void Motor_Timing_TraceArmLight(void);
/* 复用轻量缓冲：首次接管192条固定保留，余下320条循环记录；离开CLOSED当拍冻结。 */
void Motor_Timing_TraceArmFlight(void);
bool Motor_Timing_TraceIsFlight(void);
unsigned Motor_Timing_TraceFlightEntryCount(void);
bool Motor_Timing_TraceIsLight(void);
void Motor_Timing_TracePushLight(const motor_trace_light_sample_t *sample);
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
/** 首次CLOSED后逐控制拍记录384条，收满冻结；不停止电机。 */
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
 *          没有触发时每 30 次调用返回一次 true；有触发时当拍返回 true，并把 30 次计数清零。
 * @param {unsigned} state 观测器状态，2 为 CLOSED
 * @return {bool} true=本拍应 Push
 * @author: lkc
 */
bool Motor_Timing_TraceDue(unsigned state);

/**
 * @description: 写入一条环形诊断，并附上当前窗口的间隔统计。
 * @detail: sample 不可为空并由 COMMON_ASSERT 检查，active 为 false 时直接返回。写入后用窗口里的 gap/full/pending/up 覆盖样本对应字段并清窗口。
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

/**
 * @description: 更新本拍 ADC-DMA 中断耗时统计（平均与最大值）。
 * @detail: 超过 1000 us 的样本丢弃。逐拍保留整数峰值和 Q16 均值，系数 1/50，读取时换算为 us。
 * @param {uint32_t} start_time_us 本拍开始时的时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_RecordRuntime(uint32_t start_time_us);

/**
 * @description: 计算 ADC/保护用采样周期。
 * @detail: 优先用 Hal_Motor_AdcSampleRateHz()；无效时回退 motor->ol_foc_freq，并写回该字段。
 * @param {MOTOR_PARAM_S *} motor 电机对象，为空时返回 1 s
 * @return {float} 采样周期 (s)，即 1/ADC 触发频率
 * @author: lkc
 */
float Motor_Timing_GetAdcDt(MOTOR_PARAM_S *motor);

/**
 * @description: 计算 FOC 控制环周期（已乘分频）。
 * @detail: dt = ADC 周期 × FOC_CONTROL_LOOP_FREQ_DIVIDER，供斜坡、PI 和观测器使用。
 * @param {MOTOR_PARAM_S *} motor 电机对象
 * @return {float} 控制周期 (s)
 * @author: lkc
 */
float Motor_Timing_GetControlDt(MOTOR_PARAM_S *motor);

/**
 * @description: 记录一次实际执行的 FOC 控制拍位于 PWM 向上或向下计数侧。
 * @detail: 在控制拍调用，只累加计数，不阻塞。
 * @param {bool} upward true=PWM 向上计数侧
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_RecordSampleDirection(bool upward);

/**
 * @description: 读取 FOC 控制拍落在 PWM 向上/向下计数侧的累计次数。
 * @detail: 输出指针可为空，空指针对应的一侧不写出。可在任务中调用。
 * @param {uint32_t *} up 向上计数侧累计次数，可为 NULL
 * @param {uint32_t *} down 向下计数侧累计次数，可为 NULL
 * @return {void} 无
 */
void Motor_DmaInt_GetSampleDirectionCounts(uint32_t *up, uint32_t *down);

/**
 * @description: 返回 ADC-DMA 中断的平均执行时间。
 * @return {float} 平均耗时 (us)
 */
float Motor_DmaInt_GetAverageRuntimeUs(void);

/**
 * @description: 返回 ADC-DMA 中断的历史最长执行时间。
 * @detail: 超过 1000 us 的样本不参与统计。
 * @return {float} 最大耗时 (us)
 */
float Motor_DmaInt_GetMaximumRuntimeUs(void);

/**
 * @description: 返回实际 ADC-DMA 触发频率。
 * @detail: HAL 频率无效时回退 motor.ol_foc_freq。
 * @return {float} ADC 采样频率 (Hz)
 */
float Motor_DmaInt_GetAdcFrequencyHz(void);

/**
 * @description: 返回 FOC 控制环实际运行频率。
 * @detail: 控制频率 = ADC 频率 / FOC_CONTROL_LOOP_FREQ_DIVIDER。
 * @return {float} 控制环频率 (Hz)
 */
float Motor_DmaInt_GetControlFrequencyHz(void);

/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif
