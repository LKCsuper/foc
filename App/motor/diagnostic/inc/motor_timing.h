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

/*
 * 分段计时一对宏：BEGIN 记下段起点，END 在本拍计时有效时记入该段。
 * active 为假时不读时基，start 置 0。宏不改控制量；段内提前 return 则该段不计入。
 *   MOTOR_PROFILE_BEGIN(profile, start);
 *   ...本段计算...
 *   MOTOR_PROFILE_END(profile, MOTOR_TIMING_xxx, start);
 */
#define MOTOR_PROFILE_BEGIN(active, start) ((start) = (active) ? Motor_Timing_ProfileTimeUs() : 0U)
#define MOTOR_PROFILE_END(active, stage, start)                                                                        \
    do {                                                                                                               \
        if (active) Motor_Timing_ProfileRecord((stage), (start));                                                      \
    } while (0)

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

/* 诊断子模块声明；只用其中一类的调用方可直接包含对应子头文件 */
#include "motor_diag_handoff.h"
#include "motor_diag_schedule.h"
#include "motor_diag_trace.h"
#endif
