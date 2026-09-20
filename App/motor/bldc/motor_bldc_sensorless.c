/*
 * @Description: 传统 BLDC 反电动势积分无感六步控制器
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>
#include "motor_bldc_sensorless.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_BLDC_STARTUP_SYNC_SECTORS          6U
#define MOTOR_BLDC_STARTUP_TAKEOVER_PERIOD_RATIO 2.0f
#define MOTOR_BLDC_MAX_CONSECUTIVE_FORCED_SECTORS 6U
#define MOTOR_BLDC_INTEGRAL_DETECT_SAMPLES        12U
#define MOTOR_BLDC_INTEGRAL_DETECT_SETTLE_SECTORS 12U
#define MOTOR_BLDC_DUTY_SETTLED_TOLERANCE          0.0001f
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 正转 step 0..5 的悬空相依次为 C/B/A/C/B/A。 */
static const uint8_t s_floating_phase[6] = {2U, 1U, 0U, 2U, 1U, 0U};
/* Private function prototypes -----------------------------------------------*/
static bool Motor_BldcSensorless_IsFinite(float value);
static bool Motor_BldcSensorless_ConfigValid(const motor_bldc_sensorless_config_t *config);
static void Motor_BldcSensorless_UpdateDuty(motor_bldc_sensorless_t *control, float dt_s);
static void Motor_BldcSensorless_ResetSector(motor_bldc_sensorless_t *control);
static void Motor_BldcSensorless_Trip(motor_bldc_sensorless_t *control,
                                      motor_bldc_sensorless_fault_t fault);
static bool Motor_BldcSensorless_Commutate(motor_bldc_sensorless_t *control, uint8_t step);
static uint8_t Motor_BldcSensorless_NextStep(const motor_bldc_sensorless_t *control);
static float Motor_BldcSensorless_StartupPeriod(const motor_bldc_sensorless_t *control);
static bool Motor_BldcSensorless_ExpectedRising(const motor_bldc_sensorless_t *control);
static float Motor_BldcSensorless_UpdateBemf(motor_bldc_sensorless_t *control,
                                              const float phase_voltage[3]);
static void Motor_BldcSensorless_UpdateZeroCandidate(motor_bldc_sensorless_t *control,
                                                      const float phase_voltage[3],
                                                      float bus_voltage);
static void Motor_BldcSensorless_AccumulateIntegral(motor_bldc_sensorless_t *control,
                                                    const float phase_voltage[3], float dt_s);
static bool Motor_BldcSensorless_IntegralSampleValid(motor_bldc_sensorless_t *control,
                                                     const float phase_voltage[3],
                                                     float bus_voltage);
static float Motor_BldcSensorless_GetMinimumPeriod(const motor_bldc_sensorless_t *control);
static float Motor_BldcSensorless_GetIntegralThreshold(
    const motor_bldc_sensorless_t *control);
static void Motor_BldcSensorless_DetectIntegralThreshold(
    motor_bldc_sensorless_t *control);
static void Motor_BldcSensorless_ResetIntegralDetection(
    motor_bldc_sensorless_t *control);
static void Motor_BldcSensorless_RecordWaveform(motor_bldc_sensorless_t *control, float bemf_v);
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 判断浮点值是否为有限数。
 * @param {float} value 待检查值
 * @return {bool} 有限值返回 true
 * @author: lkc
 */
static bool Motor_BldcSensorless_IsFinite(float value)
{
    /* 不依赖 libm：NaN 是唯一不等于自身的浮点值。 */
    return value == value && value < 3.4028234e38f && value > -3.4028234e38f;
}

/**
 * @description: 校验无感启动与 BEMF 积分配置。
 * @param {const motor_bldc_sensorless_config_t *} config 待校验配置
 * @return {bool} 配置有效返回 true
 * @author: lkc
 */
static bool Motor_BldcSensorless_ConfigValid(const motor_bldc_sensorless_config_t *config)
{
    bool duty_period_valid;

    if (config == NULL) {
        return false;
    }
    duty_period_valid = !config->duty_period_enabled ||
                        (config->duty_period_low_duty >= 0.0f &&
                         config->duty_period_high_duty > config->duty_period_low_duty &&
                         config->duty_period_high_duty <= 1.0f &&
                         config->duty_period_low_s > 0.0f &&
                         config->duty_period_high_s > 0.0f &&
                         config->duty_period_low_s >= config->duty_period_high_s);
    return duty_period_valid && config->align_time_s >= 0.0f &&
           config->startup_duty > 0.0f && config->startup_duty <= 1.0f &&
           config->duty_slew_rate_per_s > 0.0f &&
           config->startup_initial_period_s > 0.0f && config->startup_final_period_s > 0.0f &&
           config->startup_initial_period_s >= config->startup_final_period_s &&
           config->startup_ramp_time_s > 0.0f && config->startup_timeout_s > config->align_time_s &&
           config->commutation_blanking_s >= 0.0f &&
           config->bemf_filter_alpha > 0.0f && config->bemf_filter_alpha <= 1.0f &&
           config->period_filter_alpha > 0.0f && config->period_filter_alpha <= 1.0f &&
           config->period_change_limit > 0.0f && config->period_change_limit < 1.0f &&
           config->closed_loop_min_period_s > 0.0f &&
           config->closed_loop_timeout_s > 0.0f &&
           config->bemf_integral_threshold_vs > 0.0f && config->bemf_integral_noise_v >= 0.0f &&
           config->phase_valid_margin_ratio >= 0.0f && config->phase_valid_margin_ratio <= 0.5f &&
           config->integral_detect_scale > 0.0f && config->integral_detect_scale <= 1.0f;
}

/**
 * @description: 根据实际占空比计算当前闭环最短换相周期。
 * @detail: 仅在两个已验证工作点间线性插值；区间外钳位，不作外推。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {float} 当前闭环最短 60° 换相周期 (s)
 * @author: lkc
 */
static float Motor_BldcSensorless_GetMinimumPeriod(const motor_bldc_sensorless_t *control)
{
    float duty;
    float ratio;

    if (!control->config.duty_period_enabled) {
        return control->config.closed_loop_min_period_s;
    }
    duty = control->commutator->duty;
    if (duty <= control->config.duty_period_low_duty) {
        return control->config.duty_period_low_s;
    }
    if (duty >= control->config.duty_period_high_duty) {
        return control->config.duty_period_high_s;
    }
    ratio = (duty - control->config.duty_period_low_duty) /
            (control->config.duty_period_high_duty - control->config.duty_period_low_duty);
    return control->config.duty_period_low_s +
           ratio * (control->config.duty_period_high_s - control->config.duty_period_low_s);
}

/**
 * @description: 返回当前换相步实际使用的 BEMF 积分阈值。
 * @detail: 六步全部检测成功后使用各步独立阈值；检测期间使用配置阈值。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {float} 当前步 BEMF 积分阈值 (V*s)
 * @author: lkc
 */
static float Motor_BldcSensorless_GetIntegralThreshold(
    const motor_bldc_sensorless_t *control)
{
    uint8_t mask = (uint8_t)(1U << control->step);

    if (control->config.integral_auto_detect_enabled &&
        control->detected_integral_mask == 0x3FU &&
        (control->detected_integral_mask & mask) != 0U) {
        return control->step_detected_integral_vs[control->step];
    }
    return control->config.bemf_integral_threshold_vs;
}

/**
 * @description: 用受限闭环扇区的 BEMF 面积更新当前步闭环阈值。
 * @detail: 目标占空比稳定后，每步平均多个受限闭环扇区；六步全部完成后才
 *          解除捕获周期限制，避免最小值统计造成持续提前换相偏置。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSensorless_DetectIntegralThreshold(
    motor_bldc_sensorless_t *control)
{
    float threshold = control->bemf_integral_vs * control->config.integral_detect_scale;
    uint8_t samples = control->step_integral_detect_samples[control->step];

    if (threshold < control->config.bemf_integral_threshold_vs) {
        threshold = control->config.bemf_integral_threshold_vs;
    }
    if (samples < MOTOR_BLDC_INTEGRAL_DETECT_SAMPLES) {
        control->step_detected_integral_vs[control->step] +=
            (threshold - control->step_detected_integral_vs[control->step]) /
            (float)(samples + 1U);
        samples++;
        control->step_integral_detect_samples[control->step] = samples;
    }
    if (samples >= MOTOR_BLDC_INTEGRAL_DETECT_SAMPLES) {
        control->detected_integral_mask |= (uint8_t)(1U << control->step);
    }
}

/**
 * @description: 清除目标占空比变化期间采集到的 BEMF 阈值样本。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSensorless_ResetIntegralDetection(
    motor_bldc_sensorless_t *control)
{
    control->detected_integral_mask = 0U;
    control->integral_detect_settle_sectors = 0U;
    memset(control->step_integral_detect_samples, 0,
           sizeof(control->step_integral_detect_samples));
    memset(control->step_detected_integral_vs, 0,
           sizeof(control->step_detected_integral_vs));
}

/**
 * @description: 闭环接管后将实际占空比平滑移动到用户目标值。
 * @detail: 启动阶段固定使用已验证占空比，避免目标占空比同时改变启动转矩和 BEMF 条件。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {float} dt_s 本控制周期时间 (s)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSensorless_UpdateDuty(motor_bldc_sensorless_t *control, float dt_s)
{
    float duty = control->commutator->duty;
    float maximum_change = control->config.duty_slew_rate_per_s * dt_s;
    float previous_duty = duty;

    if (duty < control->target_duty) {
        duty += maximum_change;
        if (duty > control->target_duty) {
            duty = control->target_duty;
        }
    } else if (duty > control->target_duty) {
        duty -= maximum_change;
        if (duty < control->target_duty) {
            duty = control->target_duty;
        }
    }
    if (duty != previous_duty) {
        Motor_BldcSixStep_SetDuty(control->commutator, duty);
    }
}

/**
 * @description: 复位当前 60° 扇区的 BEMF 积分状态。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSensorless_ResetSector(motor_bldc_sensorless_t *control)
{
    uint8_t phase;

    control->sector_elapsed_s = 0.0f;
    control->filtered_bemf_v = 0.0f;
    control->bemf_integral_vs = 0.0f;
    control->bemf_reverse_integral_vs = 0.0f;
    for (phase = 0U; phase < 3U; phase++) {
        control->candidate_min_abs_v[phase] = 3.4028234e38f;
        control->candidate_min_time_s[phase] = 0.0f;
        control->bus_candidate_min_abs_v[phase] = 3.4028234e38f;
        control->bus_candidate_min_time_s[phase] = 0.0f;
    }
}

/**
 * @description: 锁存无感故障并停止关联六步输出。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {motor_bldc_sensorless_fault_t} fault 要锁存的故障码
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSensorless_Trip(motor_bldc_sensorless_t *control,
                                      motor_bldc_sensorless_fault_t fault)
{
    control->fault = fault;
    control->mode = MOTOR_BLDC_SENSORLESS_FAULT;
    Motor_BldcSixStep_Stop(control->commutator);
}

/**
 * @description: 发布一个换相步并复位扇区检测状态。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {uint8_t} step 换相步，范围 0..5
 * @return {bool} 发布成功返回 true
 * @author: lkc
 */
static bool Motor_BldcSensorless_Commutate(motor_bldc_sensorless_t *control, uint8_t step)
{
    uint8_t phase;

    if (control->sector_elapsed_s >= control->config.commutation_blanking_s &&
        (control->mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP_FIXED ||
         control->mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP)) {
        for (phase = 0U; phase < 3U; phase++) {
            control->step_candidate_min_abs_v[control->step][phase] =
                control->candidate_min_abs_v[phase];
            control->step_candidate_min_time_s[control->step][phase] =
                control->candidate_min_time_s[phase];
            control->step_bus_candidate_min_abs_v[control->step][phase] =
                control->bus_candidate_min_abs_v[phase];
            control->step_bus_candidate_min_time_s[control->step][phase] =
                control->bus_candidate_min_time_s[phase];
        }
    }
    if (control->mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP && control->sector_elapsed_s > 0.0f) {
        if (control->minimum_commutation_period_s <= 0.0f ||
            control->sector_elapsed_s < control->minimum_commutation_period_s) {
            control->minimum_commutation_period_s = control->sector_elapsed_s;
        }
        if (control->sector_elapsed_s > control->maximum_commutation_period_s) {
            control->maximum_commutation_period_s = control->sector_elapsed_s;
        }
        control->step_last_period_s[control->step] = control->sector_elapsed_s;
    }
    control->step = (uint8_t)(step % 6U);
    Motor_BldcSensorless_ResetSector(control);
    if (!Motor_BldcSixStep_CommutateStep(control->commutator, control->step)) {
        Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_COMMUTATOR);
        return false;
    }
    return true;
}

/**
 * @description: 根据转向计算下一个电气换相步。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {uint8_t} 下一个换相步，范围 0..5
 * @author: lkc
 */
static uint8_t Motor_BldcSensorless_NextStep(const motor_bldc_sensorless_t *control)
{
    if (control->direction == MOTOR_BLDC_DIRECTION_CCW) {
        return control->step == 0U ? 5U : (uint8_t)(control->step - 1U);
    }
    return (uint8_t)((control->step + 1U) % 6U);
}

/**
 * @description: 计算当前开环斜坡的 60° 换相周期。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {float} 当前换相周期 (s)
 * @author: lkc
 */
static float Motor_BldcSensorless_StartupPeriod(const motor_bldc_sensorless_t *control)
{
    float ratio = control->startup_elapsed_s / control->config.startup_ramp_time_s;
    float smooth_ratio;
    float final_period_s;

    if (control->fixed_open_loop_enabled) {
        final_period_s = control->fixed_open_loop_period_s;
    } else if (control->config.duty_period_enabled) {
        final_period_s = Motor_BldcSensorless_GetMinimumPeriod(control);
    } else {
        final_period_s = control->config.startup_final_period_s;
    }

    if (ratio > 1.0f) {
        ratio = 1.0f;
    }
    /* 三次平滑插值使斜坡末端减速收敛，避免线性压缩周期在末端产生突变加速度。 */
    smooth_ratio = ratio * ratio * (3.0f - 2.0f * ratio);
    return control->config.startup_initial_period_s -
           smooth_ratio * (control->config.startup_initial_period_s - final_period_s);
}

/**
 * @description: 计算当前扇区期望的 BEMF 积分极性。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {bool} 正向为上升极性返回 true
 * @author: lkc
 */
static bool Motor_BldcSensorless_ExpectedRising(const motor_bldc_sensorless_t *control)
{
    bool rising = control->config.step0_rising ? (control->step & 1U) == 0U
                                               : (control->step & 1U) != 0U;

    /*
     * CCW 的六步输出已同时交换 PWM/LOW 并反向遍历 step，两者使当前 step 的
     * 期望边沿仍与 CW 相同；此处不能再次按方向翻转。
     */
    return rising;
}

/**
 * @description: 计算、滤波并记录当前悬空相 BEMF。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {const float [3]} phase_voltage 逻辑 A/B/C 相端电压 (V)
 * @param {float} bus_voltage 直流母线电压 (V)
 * @return {float} 滤波后的悬空相对软件中性点电压 (V)
 * @author: lkc
 */
static float Motor_BldcSensorless_UpdateBemf(motor_bldc_sensorless_t *control,
                                              const float phase_voltage[3])
{
    uint8_t floating = s_floating_phase[control->step];
    /* 与 VESC 低占空比分支一致：三相端电压平均值作为电机虚拟中性点。 */
    float neutral = (phase_voltage[0] + phase_voltage[1] + phase_voltage[2]) / 3.0f;
    float bemf = phase_voltage[floating] - neutral;

    control->filtered_bemf_v += control->config.bemf_filter_alpha *
                                (bemf - control->filtered_bemf_v);
    if (control->filtered_bemf_v < control->startup_bemf_min_v) {
        control->startup_bemf_min_v = control->filtered_bemf_v;
    }
    if (control->filtered_bemf_v > control->startup_bemf_max_v) {
        control->startup_bemf_max_v = control->filtered_bemf_v;
    }
    if (control->filtered_bemf_v < control->step_bemf_min_v[control->step]) {
        control->step_bemf_min_v[control->step] = control->filtered_bemf_v;
    }
    if (control->filtered_bemf_v > control->step_bemf_max_v[control->step]) {
        control->step_bemf_max_v[control->step] = control->filtered_bemf_v;
    }
    return control->filtered_bemf_v;
}

/**
 * @description: 记录三相相对虚拟中性点最接近零点的时刻。
 * @detail: 该诊断独立于积分有效性门控，只观测数据，不改变换相、PWM 或外设状态。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {const float [3]} phase_voltage 逻辑 A/B/C 相端电压 (V)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSensorless_UpdateZeroCandidate(motor_bldc_sensorless_t *control,
                                                      const float phase_voltage[3],
                                                      float bus_voltage)
{
    float neutral = (phase_voltage[0] + phase_voltage[1] + phase_voltage[2]) / 3.0f;
    float bus_neutral = bus_voltage * 0.5f;
    float period_s = 0.0f;
    uint16_t count;
    uint8_t bin;
    uint8_t phase;

    for (phase = 0U; phase < 3U; phase++) {
        float candidate = phase_voltage[phase] - neutral;
        float absolute = candidate < 0.0f ? -candidate : candidate;
        float bus_candidate = phase_voltage[phase] - bus_neutral;
        float bus_absolute = bus_candidate < 0.0f ? -bus_candidate : bus_candidate;

        if (absolute < control->candidate_min_abs_v[phase]) {
            control->candidate_min_abs_v[phase] = absolute;
            control->candidate_min_time_s[phase] = control->sector_elapsed_s;
        }
        if (bus_absolute < control->bus_candidate_min_abs_v[phase]) {
            control->bus_candidate_min_abs_v[phase] = bus_absolute;
            control->bus_candidate_min_time_s[phase] = control->sector_elapsed_s;
        }
    }

    if (control->mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP_FIXED) {
        period_s = control->fixed_open_loop_period_s;
        if (control->filtered_commutation_period_s > period_s * 1.02f) {
            return;
        }
    } else if (control->mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP) {
        period_s = control->filtered_commutation_period_s;
    }
    if (period_s <= 0.0f) {
        return;
    }
    bin = (uint8_t)(control->sector_elapsed_s * (float)MOTOR_BLDC_BEMF_BIN_COUNT / period_s);
    if (bin >= MOTOR_BLDC_BEMF_BIN_COUNT) {
        bin = MOTOR_BLDC_BEMF_BIN_COUNT - 1U;
    }
    count = control->step_component_waveform_count[control->step][bin];
    if (count < 65535U) {
        count++;
        control->step_component_waveform_count[control->step][bin] = count;
    }
    control->step_floating_waveform_v[control->step][bin] +=
        (phase_voltage[s_floating_phase[control->step]] -
         control->step_floating_waveform_v[control->step][bin]) /
        (float)count;
    control->step_neutral_waveform_v[control->step][bin] +=
        (neutral - control->step_neutral_waveform_v[control->step][bin]) / (float)count;
}

/**
 * @description: 按扇区归一化时间记录固定开环稳态 BEMF 波形。
 * @detail: 每个 step 分成八段累计均值，仅用于诊断，不参与换相决策。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {float} bemf_v 当前滤波 BEMF (V)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSensorless_RecordWaveform(motor_bldc_sensorless_t *control, float bemf_v)
{
    float period_s;
    uint16_t count;
    uint8_t bin;

    if (control->mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP_FIXED) {
        period_s = control->fixed_open_loop_period_s;
        if (period_s <= 0.0f || control->filtered_commutation_period_s > period_s * 1.02f) {
            return;
        }
    } else if (control->mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP) {
        period_s = control->filtered_commutation_period_s;
    } else {
        return;
    }
    if (period_s <= 0.0f) {
        return;
    }
    bin = (uint8_t)(control->sector_elapsed_s * (float)MOTOR_BLDC_BEMF_BIN_COUNT / period_s);
    if (bin >= MOTOR_BLDC_BEMF_BIN_COUNT) {
        bin = MOTOR_BLDC_BEMF_BIN_COUNT - 1U;
    }
    count = control->step_bemf_waveform_count[control->step][bin];
    if (count < 65535U) {
        count++;
        control->step_bemf_waveform_count[control->step][bin] = count;
    }
    control->step_bemf_waveform_v[control->step][bin] +=
        (bemf_v - control->step_bemf_waveform_v[control->step][bin]) / (float)count;
}

/**
 * @description: 累计当前扇区正确极性的 BEMF 面积。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {const float [3]} phase_voltage 逻辑 A/B/C 相端电压 (V)
 * @param {float} dt_s 本控制周期时间 (s)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSensorless_AccumulateIntegral(motor_bldc_sensorless_t *control,
                                                    const float phase_voltage[3], float dt_s)
{
    float signed_bemf = Motor_BldcSensorless_UpdateBemf(control, phase_voltage);

    Motor_BldcSensorless_RecordWaveform(control, signed_bemf);

    if (!Motor_BldcSensorless_ExpectedRising(control)) {
        signed_bemf = -signed_bemf;
    }
    if (signed_bemf > control->config.bemf_integral_noise_v) {
        control->bemf_integral_vs +=
            (signed_bemf - control->config.bemf_integral_noise_v) * dt_s;
    } else if (signed_bemf < -control->config.bemf_integral_noise_v) {
        control->bemf_reverse_integral_vs +=
            (-signed_bemf - control->config.bemf_integral_noise_v) * dt_s;
    }
}

/**
 * @description: 判断闭环 BEMF 积分样本是否避开功率桥钳位区。
 * @detail: 对齐 VESC COMM_MODE_INTEGRATE：前半扇区仅接收远离 0/Vbus 的悬空相；
 *          后半扇区允许积分，避免真实换相点靠近母线轨时永久漏检。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {const float [3]} phase_voltage 逻辑 A/B/C 原始相端电压 (V)
 * @param {float} bus_voltage 直流母线电压 (V)
 * @return {bool} 当前样本允许参与积分返回 true
 * @author: lkc
 */
static bool Motor_BldcSensorless_IntegralSampleValid(motor_bldc_sensorless_t *control,
                                                     const float phase_voltage[3],
                                                     float bus_voltage)
{
    float margin = (1.0f - control->commutator->duty) * bus_voltage *
                   control->config.phase_valid_margin_ratio;
    float maximum_margin = bus_voltage * 0.25f;
    float raw_phase = phase_voltage[s_floating_phase[control->step]];

    if (margin > maximum_margin) {
        margin = maximum_margin;
    }
    if (raw_phase > margin && raw_phase < bus_voltage - margin) {
        control->phase_valid_samples++;
        return true;
    }
    if (control->filtered_commutation_period_s > 0.0f &&
        control->sector_elapsed_s > control->filtered_commutation_period_s * 0.5f) {
        control->phase_late_samples++;
        return true;
    }
    control->phase_blocked_samples++;
    return false;
}

/**
 * @description: 填充无感六步默认配置。
 * @param {motor_bldc_sensorless_config_t *} config 输出配置；NULL 时忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSensorless_DefaultConfig(motor_bldc_sensorless_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->align_time_s = 0.20f;
    config->startup_duty = 0.08f;
    config->duty_slew_rate_per_s = 0.05f;
    config->startup_initial_period_s = 0.050f;
    config->startup_final_period_s = 0.003f;
    config->startup_ramp_time_s = 1.50f;
    config->startup_timeout_s = 3.0f;
    config->commutation_blanking_s = 0.00020f;
    config->bemf_filter_alpha = 0.20f;
    config->period_filter_alpha = 0.25f;
    config->period_change_limit = 0.25f;
    config->closed_loop_min_period_s = 0.001f;
    config->closed_loop_timeout_s = 0.10f;
    config->bemf_integral_threshold_vs = 0.0015f;
    config->bemf_integral_noise_v = 0.05f;
    config->phase_valid_margin_ratio = 0.30f;
    config->integral_auto_detect_enabled = false;
    config->integral_detect_scale = 1.0f;
    config->duty_period_enabled = false;
    config->duty_period_low_duty = 0.08f;
    config->duty_period_high_duty = 0.10f;
    config->duty_period_low_s = 0.012f;
    config->duty_period_high_s = 0.011f;
}

/**
 * @description: 初始化无感六步控制实例。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {motor_bldc_six_step_t *} commutator 六步输出实例，不可为空
 * @param {const motor_bldc_sensorless_config_t *} config 无感配置，不可为空
 * @return {bool} 初始化成功返回 true
 * @author: lkc
 */
bool Motor_BldcSensorless_Init(motor_bldc_sensorless_t *control,
                               motor_bldc_six_step_t *commutator,
                               const motor_bldc_sensorless_config_t *config)
{
    if (control == NULL) {
        return false;
    }
    memset(control, 0, sizeof(*control));
    control->commutator = commutator;
    if (commutator == NULL || !Motor_BldcSensorless_ConfigValid(config)) {
        control->fault = MOTOR_BLDC_SENSORLESS_FAULT_INVALID_CONFIG;
        control->mode = MOTOR_BLDC_SENSORLESS_FAULT;
        return false;
    }
    control->config = *config;
    control->initialized = true;
    return true;
}

/**
 * @description: 启动无感六步定向与开环启动流程。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {motor_bldc_direction_t} direction 转向
 * @param {float} duty PWM 占空比
 * @return {bool} 成功进入定向状态返回 true
 * @author: lkc
 */
bool Motor_BldcSensorless_Start(motor_bldc_sensorless_t *control,
                                motor_bldc_direction_t direction, float duty)
{
    if (control == NULL || !control->initialized || control->fault != MOTOR_BLDC_SENSORLESS_FAULT_NONE ||
        !Motor_BldcSensorless_IsFinite(duty) || duty <= 0.0f || duty > 1.0f ||
        !Motor_BldcSixStep_Start(control->commutator, direction, control->config.startup_duty)) {
        return false;
    }
    control->direction = direction;
    control->target_duty = duty;
    control->mode = MOTOR_BLDC_SENSORLESS_ALIGN;
    control->state_elapsed_s = 0.0f;
    control->startup_elapsed_s = 0.0f;
    control->filtered_commutation_period_s = 0.0f;
    control->minimum_commutation_period_s = 0.0f;
    control->maximum_commutation_period_s = 0.0f;
    control->fixed_open_loop_period_s = 0.0f;
    control->active_minimum_period_s = control->config.closed_loop_min_period_s;
    control->integral_commutations = 0U;
    control->forced_commutations = 0U;
    control->phase_valid_samples = 0U;
    control->phase_late_samples = 0U;
    control->phase_blocked_samples = 0U;
    control->detected_integral_mask = 0U;
    control->integral_detect_settle_sectors = 0U;
    control->consecutive_forced_commutations = 0U;
    control->startup_valid_sectors = 0U;
    control->startup_invalid_sectors = 0U;
    control->startup_sync_resets = 0U;
    control->startup_takeover_time_s = 0.0f;
    control->startup_takeover_period_s = 0.0f;
    control->startup_sync_sectors = 0U;
    memset(control->step_integral_commutations, 0,
           sizeof(control->step_integral_commutations));
    memset(control->step_forced_commutations, 0,
           sizeof(control->step_forced_commutations));
    memset(control->step_integral_detect_samples, 0,
           sizeof(control->step_integral_detect_samples));
    memset(control->step_last_period_s, 0, sizeof(control->step_last_period_s));
    memset(control->step_last_integral_vs, 0, sizeof(control->step_last_integral_vs));
    memset(control->step_detected_integral_vs, 0,
           sizeof(control->step_detected_integral_vs));
    memset(control->step_startup_integral_vs, 0,
           sizeof(control->step_startup_integral_vs));
    memset(control->step_startup_reverse_integral_vs, 0,
           sizeof(control->step_startup_reverse_integral_vs));
    memset(control->step_bemf_min_v, 0, sizeof(control->step_bemf_min_v));
    memset(control->step_bemf_max_v, 0, sizeof(control->step_bemf_max_v));
    memset(control->step_bemf_waveform_v, 0, sizeof(control->step_bemf_waveform_v));
    memset(control->step_bemf_waveform_count, 0, sizeof(control->step_bemf_waveform_count));
    memset(control->step_floating_waveform_v, 0,
           sizeof(control->step_floating_waveform_v));
    memset(control->step_neutral_waveform_v, 0,
           sizeof(control->step_neutral_waveform_v));
    memset(control->step_component_waveform_count, 0,
           sizeof(control->step_component_waveform_count));
    control->fixed_open_loop_enabled = false;
    control->startup_bemf_min_v = 0.0f;
    control->startup_bemf_max_v = 0.0f;
    return Motor_BldcSensorless_Commutate(control, 0U);
}

/**
 * @description: 启动固定周期的开环六步诊断。
 * @detail: 复用正常启动的定向和安全检查，但禁止 BEMF 检测及闭环接管。
 * @param {motor_bldc_sensorless_t *} control 控制实例，不可为空
 * @param {motor_bldc_direction_t} direction 转向，只允许 CW 或 CCW
 * @param {float} duty PWM 占空比，自动限制到 [0,1]
 * @param {float} sector_period_s 每个 60° 电气扇区的固定周期 (s)
 * @return {bool} 成功进入定向状态返回 true
 * @author: lkc
 */
bool Motor_BldcSensorless_StartOpenLoop(motor_bldc_sensorless_t *control,
                                        motor_bldc_direction_t direction, float duty,
                                        float sector_period_s)
{
    if (!Motor_BldcSensorless_IsFinite(sector_period_s) ||
        sector_period_s < 0.0005f || sector_period_s > 0.5f) {
        return false;
    }
    if (!Motor_BldcSensorless_Start(control, direction, duty)) {
        return false;
    }
    control->fixed_open_loop_period_s = sector_period_s;
    control->filtered_commutation_period_s = control->config.startup_initial_period_s;
    control->fixed_open_loop_enabled = true;
    Motor_BldcSixStep_SetDuty(control->commutator, duty);
    return true;
}

/**
 * @description: 停止无感六步状态机及关联输出。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例；NULL 时忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSensorless_Stop(motor_bldc_sensorless_t *control)
{
    if (control == NULL) {
        return;
    }
    if (control->commutator != NULL) {
        Motor_BldcSixStep_Stop(control->commutator);
    }
    control->mode = MOTOR_BLDC_SENSORLESS_IDLE;
    control->state_elapsed_s = 0.0f;
    Motor_BldcSensorless_ResetSector(control);
}

/**
 * @description: 推进一个无感六步控制周期。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {const float [3]} phase_voltage 逻辑 A/B/C 相端电压 (V)
 * @param {float} bus_voltage 直流母线电压 (V)
 * @param {float} dt_s 本控制周期时间 (s)
 * @return {bool} 本周期保持有效运行返回 true
 * @author: lkc
 */
bool Motor_BldcSensorless_Step(motor_bldc_sensorless_t *control,
                               const float phase_voltage[3], float bus_voltage, float dt_s)
{
    float period_minimum;
    float period_maximum;
    float period;
    uint8_t i;

    if (control == NULL || !control->initialized || phase_voltage == NULL ||
        (control->mode != MOTOR_BLDC_SENSORLESS_ALIGN &&
         control->mode != MOTOR_BLDC_SENSORLESS_OPEN_LOOP &&
         control->mode != MOTOR_BLDC_SENSORLESS_OPEN_LOOP_FIXED &&
         control->mode != MOTOR_BLDC_SENSORLESS_CLOSED_LOOP)) {
        return false;
    }
    if (!Motor_BldcSensorless_IsFinite(dt_s) || dt_s <= 0.0f ||
        !Motor_BldcSensorless_IsFinite(bus_voltage) || bus_voltage <= 0.0f) {
        Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_INVALID_SAMPLE);
        return false;
    }
    for (i = 0U; i < 3U; i++) {
        if (!Motor_BldcSensorless_IsFinite(phase_voltage[i])) {
            Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_INVALID_SAMPLE);
            return false;
        }
    }

    control->state_elapsed_s += dt_s;
    control->sector_elapsed_s += dt_s;
    if (control->mode == MOTOR_BLDC_SENSORLESS_ALIGN) {
        if (control->state_elapsed_s >= control->config.align_time_s) {
            control->mode = control->fixed_open_loop_enabled
                                ? MOTOR_BLDC_SENSORLESS_OPEN_LOOP_FIXED
                                : MOTOR_BLDC_SENSORLESS_OPEN_LOOP;
            control->state_elapsed_s = 0.0f;
            control->startup_elapsed_s = 0.0f;
            return Motor_BldcSensorless_Commutate(control, Motor_BldcSensorless_NextStep(control));
        }
        return true;
    }

    if (control->mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP_FIXED) {
        control->startup_elapsed_s += dt_s;
        period = Motor_BldcSensorless_StartupPeriod(control);
        control->filtered_commutation_period_s = period;
        /* 固定开环只记录 BEMF 波形，采样结果不改变换相时刻。 */
        if (control->sector_elapsed_s >= control->config.commutation_blanking_s) {
            Motor_BldcSensorless_UpdateZeroCandidate(control, phase_voltage, bus_voltage);
            Motor_BldcSensorless_AccumulateIntegral(control, phase_voltage, dt_s);
        }
        if (control->sector_elapsed_s >= period) {
            return Motor_BldcSensorless_Commutate(control, Motor_BldcSensorless_NextStep(control));
        }
        return true;
    }

    control->startup_elapsed_s += dt_s;
    if (control->mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP &&
        control->startup_elapsed_s >= control->config.startup_timeout_s) {
        Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_STARTUP_TIMEOUT);
        return false;
    }

    if (control->mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP) {
        control->filtered_commutation_period_s = Motor_BldcSensorless_StartupPeriod(control);
        if (control->sector_elapsed_s >= control->config.commutation_blanking_s) {
            Motor_BldcSensorless_UpdateZeroCandidate(control, phase_voltage, bus_voltage);
            if (Motor_BldcSensorless_IntegralSampleValid(control, phase_voltage, bus_voltage)) {
                Motor_BldcSensorless_AccumulateIntegral(control, phase_voltage, dt_s);
            }
        }
    }
    if (control->mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP) {
        float integral_threshold;

        Motor_BldcSensorless_UpdateDuty(control, dt_s);
        /* 检测完成前维持已验证捕获周期；完成后只保留绝对安全下限。 */
        control->active_minimum_period_s =
            control->config.duty_period_enabled &&
                    (!control->config.integral_auto_detect_enabled ||
                     control->detected_integral_mask != 0x3FU)
                ? Motor_BldcSensorless_GetMinimumPeriod(control)
                : control->config.closed_loop_min_period_s;
        period_maximum = control->config.closed_loop_timeout_s;
        if (control->filtered_commutation_period_s > 0.0f) {
            period = control->filtered_commutation_period_s *
                     (1.0f + control->config.period_change_limit);
            if (period < period_maximum) {
                period_maximum = period;
            }
        }
        if (control->sector_elapsed_s >= control->config.commutation_blanking_s) {
            Motor_BldcSensorless_UpdateZeroCandidate(control, phase_voltage, bus_voltage);
            /* VESC COMM_MODE_INTEGRATE：忽略负向区，只累计正确方向的 BEMF 面积。 */
            if (Motor_BldcSensorless_IntegralSampleValid(control, phase_voltage, bus_voltage)) {
                Motor_BldcSensorless_AccumulateIntegral(control, phase_voltage, dt_s);
            }
            period_minimum = control->active_minimum_period_s;
            if (control->filtered_commutation_period_s > 0.0f) {
                period = control->filtered_commutation_period_s *
                         (1.0f - control->config.period_change_limit);
                if (period > period_minimum) {
                    period_minimum = period;
                }
            }
            integral_threshold = Motor_BldcSensorless_GetIntegralThreshold(control);
            if (control->bemf_integral_vs >= integral_threshold &&
                control->sector_elapsed_s >= period_minimum) {
                control->step_last_integral_vs[control->step] = control->bemf_integral_vs;
                if (control->config.integral_auto_detect_enabled &&
                    control->detected_integral_mask != 0x3FU) {
                    float duty_error = control->commutator->duty - control->target_duty;

                    if (duty_error < 0.0f) {
                        duty_error = -duty_error;
                    }
                    if (duty_error > MOTOR_BLDC_DUTY_SETTLED_TOLERANCE) {
                        Motor_BldcSensorless_ResetIntegralDetection(control);
                    } else if (control->integral_detect_settle_sectors <
                               MOTOR_BLDC_INTEGRAL_DETECT_SETTLE_SECTORS) {
                        control->integral_detect_settle_sectors++;
                    } else {
                        Motor_BldcSensorless_DetectIntegralThreshold(control);
                    }
                }
                if (control->filtered_commutation_period_s <= 0.0f) {
                    control->filtered_commutation_period_s = control->sector_elapsed_s;
                } else {
                    control->filtered_commutation_period_s +=
                        control->config.period_filter_alpha *
                        (control->sector_elapsed_s - control->filtered_commutation_period_s);
                }
                control->integral_commutations++;
                control->consecutive_forced_commutations = 0U;
                if (control->step_integral_commutations[control->step] < 65535U) {
                    control->step_integral_commutations[control->step]++;
                }
                return Motor_BldcSensorless_Commutate(control,
                                                       Motor_BldcSensorless_NextStep(control));
            }
        }
        if (control->sector_elapsed_s >= period_maximum) {
            /*
             * 无论当前扇区是否检测到正确极性，预测最长周期都必须生效；否则
             * 漏检时会无限等待到错误边沿。保护换相不写回正常周期估计。
            */
            control->forced_commutations++;
            control->step_last_integral_vs[control->step] = control->bemf_integral_vs;
            if (control->step_forced_commutations[control->step] < 65535U) {
                control->step_forced_commutations[control->step]++;
            }
            if (control->consecutive_forced_commutations < 255U) {
                control->consecutive_forced_commutations++;
            }
            if (control->consecutive_forced_commutations >=
                MOTOR_BLDC_MAX_CONSECUTIVE_FORCED_SECTORS) {
                Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_BEMF_LOST);
                return false;
            }
            return Motor_BldcSensorless_Commutate(control,
                                                   Motor_BldcSensorless_NextStep(control));
        }
        if (control->sector_elapsed_s >= control->config.closed_loop_timeout_s) {
            Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_BEMF_LOST);
            return false;
        }
        return true;
    }

    period = Motor_BldcSensorless_StartupPeriod(control);
    if (control->sector_elapsed_s >= period) {
        /*
         * 仅在接近已验证工作周期后检查完整扇区的 BEMF 面积。连续六步均
         * 达标才说明转子跟住旋转磁场；任一步极性错误都重新开始确认。
         */
        if (period <= control->config.startup_final_period_s *
                          MOTOR_BLDC_STARTUP_TAKEOVER_PERIOD_RATIO) {
            control->step_startup_integral_vs[control->step] = control->bemf_integral_vs;
            control->step_startup_reverse_integral_vs[control->step] =
                control->bemf_reverse_integral_vs;
            if (control->bemf_integral_vs >= control->config.bemf_integral_threshold_vs &&
                control->bemf_integral_vs > control->bemf_reverse_integral_vs) {
                control->startup_valid_sectors++;
                if (control->startup_sync_sectors < MOTOR_BLDC_STARTUP_SYNC_SECTORS) {
                    control->startup_sync_sectors++;
                }
            } else {
                control->startup_invalid_sectors++;
                if (control->startup_sync_sectors > 0U) {
                    control->startup_sync_resets++;
                }
                control->startup_sync_sectors = 0U;
            }
        }
        if (control->startup_sync_sectors >= MOTOR_BLDC_STARTUP_SYNC_SECTORS) {
            control->mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
            control->state_elapsed_s = 0.0f;
            control->filtered_commutation_period_s = period;
            control->startup_takeover_time_s = control->startup_elapsed_s;
            control->startup_takeover_period_s = period;
        }
        return Motor_BldcSensorless_Commutate(control, Motor_BldcSensorless_NextStep(control));
    }
    return true;
}

/**
 * @description: 获取无感六步运行状态。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例
 * @return {motor_bldc_sensorless_mode_t} 当前运行状态
 * @author: lkc
 */
motor_bldc_sensorless_mode_t Motor_BldcSensorless_GetMode(const motor_bldc_sensorless_t *control)
{
    return control == NULL ? MOTOR_BLDC_SENSORLESS_FAULT : control->mode;
}

/**
 * @description: 获取无感六步故障码。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例
 * @return {motor_bldc_sensorless_fault_t} 当前故障码
 * @author: lkc
 */
motor_bldc_sensorless_fault_t Motor_BldcSensorless_GetFault(const motor_bldc_sensorless_t *control)
{
    return control == NULL ? MOTOR_BLDC_SENSORLESS_FAULT_INVALID_CONFIG : control->fault;
}
#ifdef __cplusplus
}
#endif
