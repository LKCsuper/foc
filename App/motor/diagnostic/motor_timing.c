/*
 * @Description: 控制周期计算、分段计时与 ADC-DMA 中断耗时统计
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

static volatile uint32_t s_runtime_average_q16;
static volatile uint32_t s_runtime_maximum_us;
static volatile uint32_t s_sample_up_count;
static volatile uint32_t s_sample_down_count;
static uint8_t s_profile_divider;
bool s_profile_active;
static bool s_profile_detailed;
bool s_performance_enabled = true;
volatile uint32_t s_profile_sum[MOTOR_TIMING_STAGE_COUNT];
volatile uint32_t s_profile_count[MOTOR_TIMING_STAGE_COUNT];
volatile uint32_t s_profile_max[MOTOR_TIMING_STAGE_COUNT];

/**
 * @description: 一次汇总高频控制路径中已开启的诊断。
 * @detail: 不推进分频、不读取时钟。回调 full 标记在细分诊断中由 CallbackClassify 写入；
 *          非紧凑轨迹的入口统计仍由 ProfileOpen 更新。
 * @return {uint32_t} 本拍诊断位掩码
 */
uint32_t Motor_Timing_IsrFeatures(void)
{
    uint32_t features = 0U;
#if MOTOR_CFG_DIAG
    if (s_performance_enabled || (s_cadence_active && !s_task_only) || (s_trace_active && !s_trace_compact))
        features |= MOTOR_TIMING_ISR_PROFILE;
    if (s_performance_enabled) features |= MOTOR_TIMING_ISR_RUNTIME;
    if (s_pair_mode && s_handoff_active && !s_pair_half) features |= MOTOR_TIMING_ISR_PAIR;
    if (s_trace_active) {
        if (s_trace_fault) features |= MOTOR_TIMING_ISR_FAULT;
        else if (s_trace_compact) features |= MOTOR_TIMING_ISR_COMPACT;
        else features |= MOTOR_TIMING_ISR_TRACE;
    }
#else
    if (s_performance_enabled) features |= MOTOR_TIMING_ISR_PROFILE | MOTOR_TIMING_ISR_RUNTIME;
#endif
    return features;
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
