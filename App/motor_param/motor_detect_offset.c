/*
 * @Description: 电流/电压偏移校准非阻塞核心（收样本，不延时、不写 PWM）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "common_assert.h"
#include <math.h>
#include <string.h>
#include "motor_detect_offset.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 停机时仅在 RAM 中重放三相电流零偏。
 * @detail: 各相相对当前值最多 1 个 ADC count，且须落在已存偏移范围内。电机运行中拒绝。
 *          不写 Flash、不启动采样。调用方负责与校准流程串行。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 currentOffset
 * @param {const float *} values  三相零偏 (ADC count)，长度 3
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Detect_OffsetSetCurrentRam(MOTOR_PARAM_S *motor, const float values[3])
{
    unsigned i;
    if (motor == NULL || values == NULL || motor->ol_enable) return false;
    for (i = 0U; i < 3U; i++) {
        if (!isfinite(values[i]) || !isfinite(motor->currentOffset[i]) ||
            values[i] < MOTOR_DETECT_OFFSET_STORED_MIN || values[i] > MOTOR_DETECT_OFFSET_STORED_MAX ||
            fabsf(values[i] - motor->currentOffset[i]) > 1.0f) return false;
    }
    for (i = 0U; i < 3U; i++) motor->currentOffset[i] = values[i];
    return true;
}

/**
 * @description: 复位偏移校准状态，准备接收样本
 * @param {motor_detect_offset_t} *st     状态，不可为空
 * @param {bool} cal_undriven             true=驱动相结束后继续采未驱动相电压
 * @param {uint32_t} samples_per_phase    每相样本数；0 则用默认 1000
 * @param {float} adc_full_scale          ADC 满量程；<=0 则用 4096
 * @return {*}
 * @author: lkc
 */
void Motor_Detect_OffsetReset(motor_detect_offset_t *st, bool cal_undriven, uint32_t samples_per_phase,
                              float adc_full_scale)
{
    COMMON_ASSERT(st != NULL);
    memset(st, 0, sizeof(*st));
    st->cal_undriven = cal_undriven;
    st->samples_per_phase =
        (samples_per_phase == 0U) ? MOTOR_DETECT_OFFSET_DEFAULT_SAMPLES : samples_per_phase;
    st->adc_full_scale = (adc_full_scale <= 0.0f) ? MOTOR_DETECT_OFFSET_DEFAULT_ADC_SCALE : adc_full_scale;
    st->phase = 0U;
}

/**
 * @description: 喂入当前驱动相的一个电流/电压样本（非阻塞）
 * @param {motor_detect_offset_t} *st  状态
 * @param {float} i_adc                相电流 ADC 计数
 * @param {float} v_sense              对应相采样点电压 (V)
 * @return {bool} true=本相样本数已满，应切换相或 FinalizeDriven
 * @author: lkc
 */
bool Motor_Detect_OffsetFeedDriven(motor_detect_offset_t *st, float i_adc, float v_sense)
{
    uint8_t p;

    if (st == NULL || st->phase > 2U) return false;
    p = st->phase;
    st->i_sum[p] += i_adc;
    st->v_sum[p] += v_sense;
    st->count++;
    return st->count >= st->samples_per_phase;
}

/**
 * @description: 同一拍喂入三相电流/电压样本（非阻塞）
 * @param {motor_detect_offset_t} *st  状态
 * @param {float} i0                   CURR1 ADC 计数
 * @param {float} i1                   CURR2 ADC 计数
 * @param {float} i2                   CURR3 ADC 计数
 * @param {float} v0                   SENS1 (V)
 * @param {float} v1                   SENS2 (V)
 * @param {float} v2                   SENS3 (V)
 * @return {bool} true=样本数已满，应 FinalizeDriven
 */
bool Motor_Detect_OffsetFeedDrivenTriple(motor_detect_offset_t *st, float i0, float i1, float i2, float v0,
                                         float v1, float v2)
{
    float sum;

    if (st == NULL) return false;
    sum = i0 + i1 + i2;
    if (st->count == 0U) {
        st->i_min[0] = i0;
        st->i_max[0] = i0;
        st->i_min[1] = i1;
        st->i_max[1] = i1;
        st->i_min[2] = i2;
        st->i_max[2] = i2;
        st->sum_min = sum;
        st->sum_max = sum;
    } else {
        if (i0 < st->i_min[0]) {
            st->i_min[0] = i0;
        }
        if (i0 > st->i_max[0]) {
            st->i_max[0] = i0;
        }
        if (i1 < st->i_min[1]) {
            st->i_min[1] = i1;
        }
        if (i1 > st->i_max[1]) {
            st->i_max[1] = i1;
        }
        if (i2 < st->i_min[2]) {
            st->i_min[2] = i2;
        }
        if (i2 > st->i_max[2]) {
            st->i_max[2] = i2;
        }
        if (sum < st->sum_min) {
            st->sum_min = sum;
        }
        if (sum > st->sum_max) {
            st->sum_max = sum;
        }
    }
    st->i_sum[0] += i0;
    st->i_sum[1] += i1;
    st->i_sum[2] += i2;
    st->v_sum[0] += v0;
    st->v_sum[1] += v1;
    st->v_sum[2] += v2;
    st->count++;
    return st->count >= st->samples_per_phase;
}

/**
 * @description: 驱动相采样满后切换到下一相
 * @param {motor_detect_offset_t} *st  状态
 * @return {bool} true=仍有下一驱动相；false=三相已完成
 * @author: lkc
 */
bool Motor_Detect_OffsetNextPhase(motor_detect_offset_t *st)
{
    if (st == NULL) return false;
    if (st->phase >= 2U) return false;
    st->phase++;
    st->count = 0U;
    return true;
}

/**
 * @description: 由三相累加和计算电流零点与驱动相电压偏置
 * @param {motor_detect_offset_t} *st  状态（写入 current_offset/volt_offset/offset_warn）
 * @return {*}
 * @author: lkc
 */
void Motor_Detect_OffsetFinalizeDriven(motor_detect_offset_t *st)
{
    float n;
    float v_avg;
    float mid;
    float abs_max;
    uint8_t i;

    COMMON_ASSERT(st != NULL);
    if (st->samples_per_phase == 0U) return;

    n = (float)st->samples_per_phase;
    for (i = 0U; i < 3U; i++) {
        st->current_offset[i] = st->i_sum[i] / n;
        st->v_sum[i] /= n;
    }
    v_avg = (st->v_sum[0] + st->v_sum[1] + st->v_sum[2]) / 3.0f;
    st->volt_offset[0] = st->v_sum[0] - v_avg;
    st->volt_offset[1] = st->v_sum[1] - v_avg;
    st->volt_offset[2] = st->v_sum[2] - v_avg;

    mid = st->adc_full_scale * 0.5f;
    abs_max = fabsf(st->current_offset[0] - mid);
    if (fabsf(st->current_offset[1] - mid) > abs_max) {
        abs_max = fabsf(st->current_offset[1] - mid);
    }
    if (fabsf(st->current_offset[2] - mid) > abs_max) {
        abs_max = fabsf(st->current_offset[2] - mid);
    }
    st->offset_warn = abs_max > (st->adc_full_scale * MOTOR_DETECT_OFFSET_WARN_PERCENT / 100.0f);
}

/**
 * @description: 喂入一拍未驱动三相电压，累加相对均值的偏置（非阻塞）
 * @param {motor_detect_offset_t} *st  状态
 * @param {float} v0                   SENS1 (V)
 * @param {float} v1                   SENS2 (V)
 * @param {float} v2                   SENS3 (V)
 * @return {bool} true=未驱动样本数已满
 * @author: lkc
 */
bool Motor_Detect_OffsetFeedUndriven(motor_detect_offset_t *st, float v0, float v1, float v2)
{
    float v_avg;

    if (st == NULL) return false;
    v_avg = (v0 + v1 + v2) / 3.0f;
    st->u_sum[0] += v0 - v_avg;
    st->u_sum[1] += v1 - v_avg;
    st->u_sum[2] += v2 - v_avg;
    st->undriven_count++;
    return st->undriven_count >= st->samples_per_phase;
}

/**
 * @description: 计算未驱动相电压偏移
 * @param {motor_detect_offset_t} *st  状态
 * @return {*}
 * @author: lkc
 */
void Motor_Detect_OffsetFinalizeUndriven(motor_detect_offset_t *st)
{
    float n;
    uint8_t i;

    COMMON_ASSERT(st != NULL);
    if (st->samples_per_phase == 0U) return;
    n = (float)st->samples_per_phase;
    for (i = 0U; i < 3U; i++) {
        st->volt_undriven_offset[i] = st->u_sum[i] / n;
    }
}

/**
 * @description: 将校准结果写入电机对象的偏移字段
 * @param {const motor_detect_offset_t} *st  已 Finalize 的状态
 * @param {MOTOR_PARAM_S} *motor             电机对象
 * @param {bool} apply_undriven              true=同时写 voltundrivenOffset
 * @return {*}
 * @author: lkc
 */
void Motor_Detect_OffsetApply(const motor_detect_offset_t *st, MOTOR_PARAM_S *motor, bool apply_undriven)
{
    uint8_t i;

    COMMON_ASSERT(st != NULL && motor != NULL);
    for (i = 0U; i < 3U; i++) {
        motor->currentOffset[i] = st->current_offset[i];
        motor->voltOffset[i] = st->volt_offset[i];
        if (apply_undriven) {
            motor->voltundrivenOffset[i] = st->volt_undriven_offset[i];
        }
    }
}

/**
 * @description: 判断电机对象是否已有可复用的电流偏移
 * @param {const MOTOR_PARAM_S} *motor 电机对象
 * @return {bool} true=三相电流零点均在可用范围内
 * @author: lkc
 */
bool Motor_Detect_OffsetIsStored(const MOTOR_PARAM_S *motor)
{
    uint8_t i;

    if (motor == NULL) return false;
    for (i = 0U; i < 3U; i++) {
        if (motor->currentOffset[i] < MOTOR_DETECT_OFFSET_STORED_MIN ||
            motor->currentOffset[i] > MOTOR_DETECT_OFFSET_STORED_MAX) {
            return false;
        }
    }
    return true;
}

#ifdef __cplusplus
}
#endif
