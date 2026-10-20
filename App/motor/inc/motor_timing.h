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
                                   float nominal_dt, float *dt);

typedef enum {
    MOTOR_TIMING_SAMPLE,           /* 采样与保护整段 */
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
    MOTOR_TIMING_SENSE_FRAME,      /* Sense_Update；仅使能的无感 I/F 细分拍，含在 SAMPLE 内 */
    MOTOR_TIMING_SENSOR_FEEDBACK,  /* SensorUpdate_Step；仅使能的无感 I/F 细分拍，含在 SAMPLE 内 */
    MOTOR_TIMING_PROTECTION,       /* Protect_Run；仅使能的无感 I/F 细分拍，含在 SAMPLE 内 */
    MOTOR_TIMING_VOLTAGE_REBUILD,  /* 无感 I/F 的 PWM 平均电压重建，含在 OUTPUT 内；不含反 Park、SVM 和比较值提交 */
    MOTOR_TIMING_STAGE_COUNT       /* 分段总数，仅作数组维度，不参与记录 */
} motor_timing_stage_t;
typedef struct {
    uint32_t count[MOTOR_TIMING_STAGE_COUNT];
    uint32_t sum_us[MOTOR_TIMING_STAGE_COUNT];
    uint32_t max_us[MOTOR_TIMING_STAGE_COUNT];
} motor_profile_snapshot_t;
/**
 * @description: 在短临界区内复制全部分段原始统计。
 * @detail: 平均值和打印由调用方在临界区外计算。out 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_profile_snapshot_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileSnapshot(motor_profile_snapshot_t *out);

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 推进分段耗时分频，并决定本拍是否采样。
 * @detail: 性能统计关闭时返回 false。分频计数是 uint8_t，每次调用加 1，回绕为 0 时返回 true，即每 256 次调用采 1 次。
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
/**
 * @description: 开关电流环内部的细分计时。
 * @detail: 默认关闭。只影响诊断采样，不改变控制周期。总分段采样仍由 ProfileBegin 决定。
 * @param {bool} enabled true=允许细分阶段记录
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileSetDetailed(bool enabled);

/**
 * @description: 查询本拍细分计时是否有效。
 * @detail: 总采样标志和细分开关都为真才返回 true。
 * @return {bool} true=本拍可记录细分阶段
 * @author: lkc
 */
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
 *          SENSE/SENSOR/PROTECTION 只在使能的无感 I/F 细分拍计入，停机拍留在 SAMPLE。
 *          VOLTAGE_REBUILD 含在 OUTPUT 内，不能与 OUTPUT 相加。
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
typedef struct {
    uint32_t count, first_us, last_us, sum_us, max_us, tick_pending, switch_pending;
    uint32_t seen;
} motor_irq_diag_t;
#define MOTOR_TIMING_IRQ_STRIDE 31U
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

/**
 * @brief 在短临界区仅填充轻量轨迹需要的任务字段。
 * @param sample 非空，本拍 time_us 已填写；其余非任务字段保持不变。
 */
void Motor_Timing_SpeedTaskCaptureLight(motor_trace_light_sample_t *sample);
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

/**
 * @description: 更新本拍 ADC-DMA 中断耗时统计（平均与最大值）。
 * @detail: 超过 1000 us 的样本丢弃。逐拍保留整数峰值和 Q16 均值，系数 1/50，读取时换算为 us。
 * @param {uint32_t} start_time_us 本拍开始时的时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_RecordRuntime(uint32_t start_time_us);

#define MOTOR_TIMING_ISR_PROFILE 1U
#define MOTOR_TIMING_ISR_RUNTIME 2U
#define MOTOR_TIMING_ISR_PAIR 4U
#define MOTOR_TIMING_ISR_FAULT 8U
#define MOTOR_TIMING_ISR_COMPACT 16U
#define MOTOR_TIMING_ISR_TRACE 32U

/**
 * @description: 汇总本控制拍需要执行的诊断入口。
 * @detail: 供 ADC 中断读取；仅跳过未开启的诊断，具体记录函数仍负责到期判断与冻结检查。
 * @return {uint32_t} MOTOR_TIMING_ISR_* 位掩码，不改变采集状态
 */
uint32_t Motor_Timing_IsrFeatures(void);

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
