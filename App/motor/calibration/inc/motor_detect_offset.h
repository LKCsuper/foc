/*
 * @Description: 电流/电压偏移校准非阻塞核心（收样本，不延时、不写 PWM）
 * @Author: lkc
 */
#ifndef MOTOR_DETECT_OFFSET_H
#define MOTOR_DETECT_OFFSET_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_DETECT_OFFSET_DEFAULT_SAMPLES (1000U)
#define MOTOR_DETECT_OFFSET_DEFAULT_ADC_SCALE (4096.0f)
#define MOTOR_DETECT_OFFSET_WARN_PERCENT (5.0f)
/* 与 Motor_Sense 可用电流零点范围一致；全 0 表示尚未校准 */
#define MOTOR_DETECT_OFFSET_STORED_MIN (1000.0f)
#define MOTOR_DETECT_OFFSET_STORED_MAX (3000.0f)
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 偏移校准累加状态（由调用方持有，可多实例）
 */
typedef struct {
    bool cal_undriven;
    uint32_t samples_per_phase;
    float adc_full_scale; /* ADC 满量程计数，用于中点偏离告警 */

    uint8_t phase;  /* 当前驱动相 0..2 */
    uint32_t count; /* 当前相已累计样本数 */
    float i_sum[3];
    float i_min[3]; /* 三相电流 ADC 计数最小值 */
    float i_max[3]; /* 三相电流 ADC 计数最大值 */
    float sum_min;  /* CURR1+2+3 最小和，ADC 计数 */
    float sum_max;  /* CURR1+2+3 最大和，ADC 计数 */
    float v_sum[3];
    float u_sum[3]; /* 未驱动相电压相对均值的累加 */
    uint32_t undriven_count;

    float current_offset[3];
    float volt_offset[3];
    float volt_undriven_offset[3];
    bool offset_warn; /* 电流零点偏离中点超过阈值 */
} motor_detect_offset_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 复位偏移校准状态，准备接收样本
 * @param {motor_detect_offset_t} *st     状态，不可为空
 * @param {bool} cal_undriven             true=驱动相结束后继续采未驱动相电压
 * @param {uint32_t} samples_per_phase    每相样本数；0 则用默认 1000
 * @param {float} adc_full_scale          ADC 满量程；<=0 则用 4096
 * @return {void} 无
 * @author: lkc
 */
void Motor_Detect_OffsetReset(motor_detect_offset_t *st, bool cal_undriven, uint32_t samples_per_phase,
                              float adc_full_scale);

/**
 * @description: 喂入当前驱动相的一个电流/电压样本（非阻塞）
 * @param {motor_detect_offset_t} *st  状态
 * @param {float} i_adc                相电流 ADC 计数
 * @param {float} v_sense              对应相采样点电压 (V)
 * @return {bool} true=本相样本数已满，应切换相或 FinalizeDriven
 * @author: lkc
 */
bool Motor_Detect_OffsetFeedDriven(motor_detect_offset_t *st, float i_adc, float v_sense);

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
                                         float v1, float v2);

/**
 * @description: 驱动相采样满后切换到下一相
 * @param {motor_detect_offset_t} *st  状态
 * @return {bool} true=仍有下一驱动相；false=三相已完成
 * @author: lkc
 */
bool Motor_Detect_OffsetNextPhase(motor_detect_offset_t *st);

/**
 * @description: 由三相累加和计算电流零点与驱动相电压偏置
 * @param {motor_detect_offset_t} *st  状态（写入 current_offset/volt_offset/offset_warn）
 * @return {void} 无
 * @author: lkc
 */
void Motor_Detect_OffsetFinalizeDriven(motor_detect_offset_t *st);

/**
 * @description: 喂入一拍未驱动三相电压，累加相对均值的偏置（非阻塞）
 * @param {motor_detect_offset_t} *st  状态
 * @param {float} v0                   SENS1 (V)
 * @param {float} v1                   SENS2 (V)
 * @param {float} v2                   SENS3 (V)
 * @return {bool} true=未驱动样本数已满
 * @author: lkc
 */
bool Motor_Detect_OffsetFeedUndriven(motor_detect_offset_t *st, float v0, float v1, float v2);

/**
 * @description: 计算未驱动相电压偏移
 * @param {motor_detect_offset_t} *st  状态
 * @return {void} 无
 * @author: lkc
 */
void Motor_Detect_OffsetFinalizeUndriven(motor_detect_offset_t *st);

/**
 * @description: 将校准结果写入电机对象的偏移字段
 * @param {const motor_detect_offset_t} *st  已 Finalize 的状态
 * @param {MOTOR_PARAM_S} *motor             电机对象
 * @param {bool} apply_undriven              true=同时写 voltundrivenOffset
 * @return {void} 无
 * @author: lkc
 */
void Motor_Detect_OffsetApply(const motor_detect_offset_t *st, MOTOR_PARAM_S *motor, bool apply_undriven);

/**
 * @description: 判断电机对象是否已有可复用的电流偏移（Flash/上次校准）
 * @detail: 三相 currentOffset 均在 STORED_MIN..MAX（ADC 计数）内视为已校准。
 *          默认全 0 或越界值返回 false，上电应重新检测。
 * @param {const MOTOR_PARAM_S} *motor 电机对象
 * @return {bool} true=可跳过上电检测，直接使用已存偏移
 * @author: lkc
 */
bool Motor_Detect_OffsetIsStored(const MOTOR_PARAM_S *motor);
/**
 * @description: 停机时仅在 RAM 中重放三相电流零偏。
 * @detail: 各相相对当前值最多 1 个 ADC count，且须落在已存偏移范围内。电机运行中拒绝。
 *          不写 Flash、不启动采样。调用方负责与校准流程串行。
 * @param {MOTOR_PARAM_S *} motor    电机对象，写入 currentOffset
 * @param {const float *} values     三相零偏 (ADC count)，长度 3
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Detect_OffsetSetCurrentRam(MOTOR_PARAM_S *motor, const float values[3]);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
