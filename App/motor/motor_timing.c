/*
 * @Description: ADC/控制周期计算与 ADC-DMA 中断耗时统计
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_timing.h"

#include <stddef.h>
#include <string.h>

#include "hal_motor.h"
#include "motor_crit.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile float s_runtime_average_us;
static volatile float s_runtime_maximum_us;
static volatile uint32_t s_sample_up_count;
static volatile uint32_t s_sample_down_count;
static uint8_t s_profile_divider;
static bool s_profile_active;
static volatile uint32_t s_profile_sum[MOTOR_TIMING_STAGE_COUNT];
static volatile uint32_t s_profile_count[MOTOR_TIMING_STAGE_COUNT];
static volatile uint32_t s_profile_max[MOTOR_TIMING_STAGE_COUNT];
static motor_trace_sample_t s_trace[MOTOR_TRACE_CAPACITY];
static unsigned s_trace_head, s_trace_count, s_trace_divider, s_trace_post;
static bool s_trace_active, s_trace_closed, s_trace_trigger;
static bool s_trace_entry_mode, s_trace_entry_trigger;
static motor_cadence_t s_cadence;
static uint32_t s_cadence_last;
static bool s_cadence_active, s_cadence_seed;
static motor_callback_t s_callback;
static uint32_t s_callback_start, s_callback_last;
static bool s_callback_active, s_callback_seed, s_callback_full;
static uint32_t s_trace_entry_last;
static bool s_trace_entry_seed;
static uint16_t s_trace_gap_max, s_trace_full_max, s_trace_pending, s_trace_up;

/**
 * @description: 标记一次 DMA 回调开始，并统计回调入口间隔。
 * @detail: 随 CadenceReset 启用；未启用则立即返回，也不清除 full 标志。启用后先把 full 清成 false，
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
    if (!s_callback_active) return;
    s_callback_start = time_us;
    s_callback_full = false;
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
 * @detail: 未启用则立即返回。耗时大于 1000 us 时按 1000 us 计入。本回调执行过 CadenceNote 则计入 full，
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
    if (!s_callback_active) return;
    elapsed = time_us - s_callback_start;
    if (elapsed > 1000U) elapsed = 1000U;
    if (s_trace_active) {
        if (s_callback_full && elapsed > s_trace_full_max) s_trace_full_max = (uint16_t)elapsed;
        if (pending && s_trace_pending < UINT16_MAX) s_trace_pending++;
    }
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
    if (s_callback.count >= 1000000U) s_callback_active = false;
}
/**
 * @description: 拷贝 DMA 回调耗时统计。
 * @detail: 本函数进入临界区。out 为空则直接返回。
 * @param {motor_callback_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CallbackGet(motor_callback_t *out)
{
    uint32_t key;
    if (out == NULL) return;
    key = Motor_Crit_Enter();
    *out = s_callback;
    Motor_Crit_Exit(key);
}

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
    memset(&s_cadence, 0, sizeof(s_cadence));
    memset((void *)s_profile_sum, 0, sizeof(s_profile_sum));
    memset((void *)s_profile_count, 0, sizeof(s_profile_count));
    memset((void *)s_profile_max, 0, sizeof(s_profile_max));
    s_cadence_seed = false;
    s_cadence_active = true;
    memset(&s_callback, 0, sizeof(s_callback));
    s_callback_seed = false;
    s_callback_active = true;
    Motor_Crit_Exit(key);
}
/**
 * @description: 停止节拍和 DMA 回调累加。
 * @detail: 临界区内只清两个使能标志。已有统计和环形诊断都保留。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CadenceStop(void)
{
    uint32_t key = Motor_Crit_Enter();
    s_cadence_active = false;
    s_callback_active = false;
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
    if (s_trace_active) {
        uint32_t trace_delta = time_us - s_trace_entry_last;
        s_trace_entry_last = time_us;
        if (s_trace_entry_seed && trace_delta > s_trace_gap_max)
            s_trace_gap_max = (uint16_t)(trace_delta > UINT16_MAX ? UINT16_MAX : trace_delta);
        s_trace_entry_seed = true;
        if (upward && s_trace_up < UINT16_MAX) s_trace_up++;
    }
    if (!s_cadence_active) return;
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
 * @detail: 本函数进入临界区。out 为空则直接返回。
 * @param {motor_cadence_t *} out 输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_CadenceGet(motor_cadence_t *out)
{
    uint32_t key;
    if (out == NULL) return;
    key = Motor_Crit_Enter();
    *out = s_cadence;
    Motor_Crit_Exit(key);
}

/**
 * @description: 复位环形诊断索引并开始记录。
 * @detail: 只清下标、条数和触发标志，不擦除样本数组，也不检查电机是否在转。
 *          未触发时 Due 每 30 次调用放行一次。曾进入 CLOSED(2) 后再离开时触发，触发后连同当拍再收 16 条并停止。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceArm(void)
{
    s_trace_head = s_trace_count = s_trace_divider = s_trace_post = 0U;
    s_trace_closed = s_trace_trigger = false;
    s_trace_entry_mode = s_trace_entry_trigger = false;
    s_trace_entry_seed = false;
    s_trace_gap_max = s_trace_full_max = s_trace_pending = s_trace_up = 0U;
    s_trace_active = true;
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
 * @description: 停止继续写入环形诊断。
 * @detail: 只清除 active。不检查停机，不清样本。停止后 Get 可以读取；收满触发后样本也会自行清除 active。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TraceStop(void) { s_trace_active = false; }
/**
 * @description: 返回当前已写入、尚未被新数据覆盖的条数。
 * @detail: 仍在记录时也会增长，最大为 MOTOR_TRACE_CAPACITY。条数不等于 Get 已经允许读取。
 * @return {unsigned} 条数
 * @author: lkc
 */
unsigned Motor_Timing_TraceCount(void) { return s_trace_count; }
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
bool Motor_Timing_TraceDue(unsigned state)
{
    bool event;
    if (!s_trace_active) return false;
    event = !s_trace_trigger && s_trace_closed && state != 2U;
    if (s_trace_entry_mode && !s_trace_entry_trigger && state == 2U) {
        s_trace_entry_trigger = true;
        s_trace_post = 384U;
        event = true;
    } else if (event) {
        s_trace_trigger = true;
        if (!s_trace_entry_trigger) s_trace_post = 16U;
    }
    if (state == 2U) s_trace_closed = true;
    s_trace_divider++;
    if (!event && s_trace_divider < 30U) return false;
    s_trace_divider = 0U;
    return true;
}
/**
 * @description: 写入一条环形诊断，并附上当前窗口的间隔统计。
 * @detail: active 为 false 或 sample 为空则直接返回。写入后用窗口里的 gap/full/pending/up 覆盖样本对应字段并清窗口。
 *          失锁触发给 flags 加 bit7(128)，接管触发加 bit8(256)。配额减到 0 时清除 active。
 * @param {const motor_trace_sample_t *} sample 本拍样本；空指针直接返回
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_TracePush(const motor_trace_sample_t *sample)
{
    if (!s_trace_active || sample == NULL) return;
    s_trace[s_trace_head] = *sample;
    s_trace[s_trace_head].gap_max_us = s_trace_gap_max;
    s_trace[s_trace_head].full_max_us = s_trace_full_max;
    s_trace[s_trace_head].pending_count = s_trace_pending;
    s_trace[s_trace_head].up_count = s_trace_up;
    s_trace_gap_max = s_trace_full_max = s_trace_pending = s_trace_up = 0U;
    if (s_trace_trigger) s_trace[s_trace_head].flags |= 128U;
    if (s_trace_entry_trigger) s_trace[s_trace_head].flags |= 256U;
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
    if (s_trace_active || sample == NULL || index >= s_trace_count) return false;
    first = (s_trace_head + MOTOR_TRACE_CAPACITY - s_trace_count) % MOTOR_TRACE_CAPACITY;
    *sample = s_trace[(first + index) % MOTOR_TRACE_CAPACITY];
    return true;
}

/**
 * @description: 推进分段耗时分频，并决定本拍是否采样。
 * @detail: 分频计数是 uint8_t，每次调用加 1，回绕为 0 时返回 true，即每 256 次调用采 1 次。
 *          调用方只在通过控制门控之后调用。本函数不读取耗时，也不改变控制量。
 * @return {bool} true=本拍后续 Record 会累加
 * @author: lkc
 */
bool Motor_Timing_ProfileBegin(void)
{
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
 * @detail: 本函数进入临界区。stage 非法或输出指针为空则直接返回。CadenceReset 会清这些数组。
 * @param {motor_timing_stage_t} stage 阶段编号
 * @param {uint32_t *} count           输出样本数，不可为空
 * @param {uint32_t *} maximum         输出峰值 (us)，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_ProfileDetail(motor_timing_stage_t stage, uint32_t *count, uint32_t *maximum)
{
    uint32_t key;
    if ((unsigned)stage >= MOTOR_TIMING_STAGE_COUNT || count == NULL || maximum == NULL) return;
    key = Motor_Crit_Enter();
    *count = s_profile_count[stage];
    *maximum = s_profile_max[stage];
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
    if (average_us == NULL) return;
    for (i = 0U; i < MOTOR_TIMING_STAGE_COUNT; ++i) {
        uint32_t count = s_profile_count[i];
        average_us[i] = count ? (float)s_profile_sum[i] / (float)count : 0.0f;
    }
    if (samples != NULL) *samples = s_profile_count[MOTOR_TIMING_CONTROL];
}
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 更新本拍 ADC-DMA 中断耗时统计（平均与最大值）。
 * @detail: 超过 1000 us 的样本丢弃，避免计时回绕或异常拉高最大耗时。平均系数 0.02。
 * @param {uint32_t} start_time_us 本拍开始时的时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_RecordRuntime(uint32_t start_time_us)
{
    uint32_t elapsed_us = Hal_Motor_TimeUs() - start_time_us;
    float elapsed = (float)elapsed_us;
    if (elapsed_us > 1000U) {
        return;
    }
    if (s_runtime_average_us <= 0.0f) {
        s_runtime_average_us = elapsed;
    } else {
        s_runtime_average_us += 0.02f * (elapsed - s_runtime_average_us);
    }
    if (elapsed > s_runtime_maximum_us) {
        s_runtime_maximum_us = elapsed;
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
    if (motor == NULL) {
        return 1.0f;
    }
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
 * @param {uint32_t *} up 向上计数侧累计次数，可为 NULL
 * @param {uint32_t *} down 向下计数侧累计次数，可为 NULL
 * @return {void} 无
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
 * @return {float} 平均耗时 (us)
 * @author: lkc
 */
float Motor_DmaInt_GetAverageRuntimeUs(void) { return s_runtime_average_us; }

/**
 * @description: 返回 ADC-DMA 中断的历史最长执行时间。
 * @return {float} 最大耗时 (us)
 * @author: lkc
 */
float Motor_DmaInt_GetMaximumRuntimeUs(void) { return s_runtime_maximum_us; }

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
