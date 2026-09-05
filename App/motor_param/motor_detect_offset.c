/*
 * @Description: 电流/电压偏移校准非阻塞核心（收样本，不延时、不写 PWM）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <math.h>
#include <string.h>
#include "motor_detect_offset.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 复位偏移校准状态，准备接收样本。
 */
void Motor_Detect_OffsetReset(motor_detect_offset_t *st, bool cal_undriven, uint32_t samples_per_phase,
                              float adc_full_scale)
{
    if (st == NULL) {
        return;
    }
    memset(st, 0, sizeof(*st));
    st->cal_undriven = cal_undriven;
    st->samples_per_phase =
        (samples_per_phase == 0U) ? MOTOR_DETECT_OFFSET_DEFAULT_SAMPLES : samples_per_phase;
    st->adc_full_scale = (adc_full_scale <= 0.0f) ? MOTOR_DETECT_OFFSET_DEFAULT_ADC_SCALE : adc_full_scale;
    st->phase = 0U;
}

/**
 * @description: 喂入当前驱动相的一个电流/电压样本。
 * @return {bool} true=本相已满
 */
bool Motor_Detect_OffsetFeedDriven(motor_detect_offset_t *st, float i_adc, float v_sense)
{
    uint8_t p;

    if (st == NULL || st->phase > 2U) {
        return false;
    }
    p = st->phase;
    st->i_sum[p] += i_adc;
    st->v_sum[p] += v_sense;
    st->count++;
    return st->count >= st->samples_per_phase;
}

/**
 * @description: 切换到下一驱动相。
 * @return {bool} true=还有下一相
 */
bool Motor_Detect_OffsetNextPhase(motor_detect_offset_t *st)
{
    if (st == NULL) {
        return false;
    }
    if (st->phase >= 2U) {
        return false;
    }
    st->phase++;
    st->count = 0U;
    return true;
}

/**
 * @description: 计算驱动相电流零点与电压偏置，并置 offset_warn。
 */
void Motor_Detect_OffsetFinalizeDriven(motor_detect_offset_t *st)
{
    float n;
    float v_avg;
    float mid;
    float abs_max;
    uint8_t i;

    if (st == NULL || st->samples_per_phase == 0U) {
        return;
    }

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
 * @description: 喂入一拍未驱动三相电压。
 * @return {bool} true=样本已满
 */
bool Motor_Detect_OffsetFeedUndriven(motor_detect_offset_t *st, float v0, float v1, float v2)
{
    float v_avg;

    if (st == NULL) {
        return false;
    }
    v_avg = (v0 + v1 + v2) / 3.0f;
    st->u_sum[0] += v0 - v_avg;
    st->u_sum[1] += v1 - v_avg;
    st->u_sum[2] += v2 - v_avg;
    st->undriven_count++;
    return st->undriven_count >= st->samples_per_phase;
}

/**
 * @description: 计算未驱动相电压偏移。
 */
void Motor_Detect_OffsetFinalizeUndriven(motor_detect_offset_t *st)
{
    float n;
    uint8_t i;

    if (st == NULL || st->samples_per_phase == 0U) {
        return;
    }
    n = (float)st->samples_per_phase;
    for (i = 0U; i < 3U; i++) {
        st->volt_undriven_offset[i] = st->u_sum[i] / n;
    }
}

/**
 * @description: 将校准结果写入电机偏移字段。
 */
void Motor_Detect_OffsetApply(const motor_detect_offset_t *st, MOTOR_PARAM_S *motor, bool apply_undriven)
{
    uint8_t i;

    if (st == NULL || motor == NULL) {
        return;
    }
    for (i = 0U; i < 3U; i++) {
        motor->currentOffset[i] = st->current_offset[i];
        motor->voltOffset[i] = st->volt_offset[i];
        if (apply_undriven) {
            motor->voltundrivenOffset[i] = st->volt_undriven_offset[i];
        }
    }
}

#ifdef __cplusplus
}
#endif
