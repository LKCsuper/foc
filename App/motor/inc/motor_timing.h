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
typedef enum {
    MOTOR_TIMING_SAMPLE,           /* SampleAndProtect 整段 */
    MOTOR_TIMING_CURRENT,          /* PrepareCurrentFeedback 整段 */
    MOTOR_TIMING_CONTROL,          /* RunControl 墙钟；内部又记了 OBSERVER 与 TRANSPORT */
    MOTOR_TIMING_OBSERVER,         /* Sensorless_Update 整段，含在 CONTROL 墙钟内 */
    MOTOR_TIMING_OUTPUT,           /* ArbitrateAndApply 整段 */
    MOTOR_TIMING_OBSERVER_BLEND,   /* CLOSED 且 blend<1 时，与 OBSERVER 同一段耗时再记一档 */
    MOTOR_TIMING_OBSERVER_CLOSED,  /* CLOSED 且 blend 已到 1 时，与 OBSERVER 同一段耗时再记一档 */
    MOTOR_TIMING_TRANSPORT,        /* frame_correction 非 0 时的 TransportPi，含在 CONTROL 墙钟内 */
    MOTOR_TIMING_STAGE_COUNT
} motor_timing_stage_t;

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
 * @detail: 本函数进入临界区。stage 非法或输出指针为空则直接返回。CadenceReset 会清这些数组。
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
 * @detail: 本函数进入临界区。out 为空则直接返回。
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
 * @detail: 随 CadenceReset 启用；未启用则立即返回，也不清除 full 标志。启用后先把 full 清成 false，
 *          之后若本回调执行到 CadenceNote 会再标成 true。第一次调用只建立时间基准。
 *          间隔大于 1000 us 只加 gaps。本函数不读 DMA 标志。
 * @param {uint32_t} time_us 回调入口时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackBegin(uint32_t time_us);

/**
 * @description: 结束一次 DMA 回调，按 full/skip 分开累计耗时。
 * @detail: 未启用则立即返回。耗时大于 1000 us 时按 1000 us 计入。本回调执行过 CadenceNote 则计入 full，
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
 * @detail: 本函数进入临界区。out 为空则直接返回。
 * @param {motor_callback_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackGet(motor_callback_t *out);

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
 * @detail: active 为 false 或 sample 为空则直接返回。写入后用窗口里的 gap/full/pending/up 覆盖样本对应字段并清窗口。
 *          失锁触发给 flags 加 bit7(128)，接管触发加 bit8(256)。配额减到 0 时清除 active。
 * @param {const motor_trace_sample_t *} sample 本拍样本；空指针直接返回
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
 * @detail: 超过 1000 us 的样本丢弃，避免计时回绕或异常拉高最大耗时。在中断末尾调用，不阻塞。
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
