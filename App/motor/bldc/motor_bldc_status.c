/*
 * @Description: BLDC 控制状态快照组装
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_bldc_status.h"

#include <stddef.h>
#include <string.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 从 BLDC 控制实例生成对外只读状态快照。
 * @detail: 纯数据复制，不访问硬件、不加锁。时间单位为 s，电压为 V，积分为 V·s。
 * @param {motor_bldc_status_t *} status 输出状态，不可为空
 * @param {bool} selected BLDC 是否占用主 PWM 出口
 * @param {bool} sample_upward BEMF 是否使用 PWM 向上计数侧采样
 * @param {const motor_bldc_six_step_t *} commutator 六步控制实例；未初始化时可为空
 * @param {const motor_bldc_sensorless_t *} sensorless 无感控制实例；未初始化时可为空
 * @param {mc_fault_code} motor_fault 电机保护故障快照
 * @param {uint32_t} bemf_sample_count BEMF 硬件采样计数
 * @return {bool} 输出参数有效返回 true
 */
bool Motor_BldcStatus_Fill(motor_bldc_status_t *status, bool selected, bool sample_upward,
                           const motor_bldc_six_step_t *commutator,
                           const motor_bldc_sensorless_t *sensorless, mc_fault_code motor_fault,
                           uint32_t bemf_sample_count)
{
    if (status == NULL) return false;

    memset(status, 0, sizeof(*status));
    status->selected = selected;
    if (commutator == NULL || sensorless == NULL) return true;

    status->running = Motor_BldcSixStep_IsRunning(commutator);
    status->duty = commutator->duty;
    status->target_duty = sensorless->target_duty;
    status->step = sensorless->step;
    status->direction = sensorless->direction;
    status->mode = sensorless->mode;
    status->fault = sensorless->fault;
    status->motor_fault = motor_fault;
    status->bemf_v = sensorless->filtered_bemf_v;
    status->bemf_min_v = sensorless->startup_bemf_min_v;
    status->bemf_max_v = sensorless->startup_bemf_max_v;
    status->sector_elapsed_s = sensorless->sector_elapsed_s;
    status->commutation_period_s = sensorless->filtered_commutation_period_s;
    status->minimum_commutation_period_s = sensorless->minimum_commutation_period_s;
    status->maximum_commutation_period_s = sensorless->maximum_commutation_period_s;
    status->active_minimum_period_s = sensorless->active_minimum_period_s;
    status->bemf_integral_vs = sensorless->bemf_integral_vs;
    status->bemf_reverse_integral_vs = sensorless->bemf_reverse_integral_vs;
    status->bemf_integral_threshold_vs = sensorless->config.bemf_integral_threshold_vs;
    if (sensorless->config.integral_auto_detect_enabled &&
        sensorless->detected_integral_mask == 0x3FU &&
        (sensorless->detected_integral_mask & (uint8_t)(1U << sensorless->step)) != 0U) {
        status->bemf_integral_threshold_vs = sensorless->step_detected_integral_vs[sensorless->step];
    }
    status->bemf_sample_count = bemf_sample_count;
    status->integral_commutations = sensorless->integral_commutations;
    status->forced_commutations = sensorless->forced_commutations;
    status->phase_valid_samples = sensorless->phase_valid_samples;
    status->phase_late_samples = sensorless->phase_late_samples;
    status->phase_blocked_samples = sensorless->phase_blocked_samples;
    status->detected_integral_mask = sensorless->detected_integral_mask;
    status->integral_detect_settle_sectors = sensorless->integral_detect_settle_sectors;
    status->duty_period_enabled = sensorless->config.duty_period_enabled;
    status->integral_auto_detect_enabled = sensorless->config.integral_auto_detect_enabled;
    status->sample_upward = sample_upward;
    status->consecutive_forced_commutations = sensorless->consecutive_forced_commutations;
    status->startup_valid_sectors = sensorless->startup_valid_sectors;
    status->startup_invalid_sectors = sensorless->startup_invalid_sectors;
    status->startup_sync_resets = sensorless->startup_sync_resets;
    status->startup_takeover_time_s = sensorless->startup_takeover_time_s;
    status->startup_takeover_period_s = sensorless->startup_takeover_period_s;
    status->startup_sync_sectors = sensorless->startup_sync_sectors;
    memcpy(status->step_integral_commutations, sensorless->step_integral_commutations,
           sizeof(status->step_integral_commutations));
    memcpy(status->step_forced_commutations, sensorless->step_forced_commutations,
           sizeof(status->step_forced_commutations));
    memcpy(status->step_integral_detect_samples, sensorless->step_integral_detect_samples,
           sizeof(status->step_integral_detect_samples));
    memcpy(status->step_last_period_s, sensorless->step_last_period_s,
           sizeof(status->step_last_period_s));
    memcpy(status->step_last_integral_vs, sensorless->step_last_integral_vs,
           sizeof(status->step_last_integral_vs));
    memcpy(status->step_detected_integral_vs, sensorless->step_detected_integral_vs,
           sizeof(status->step_detected_integral_vs));
    memcpy(status->step_startup_integral_vs, sensorless->step_startup_integral_vs,
           sizeof(status->step_startup_integral_vs));
    memcpy(status->step_startup_reverse_integral_vs, sensorless->step_startup_reverse_integral_vs,
           sizeof(status->step_startup_reverse_integral_vs));
    memcpy(status->step_bemf_min_v, sensorless->step_bemf_min_v, sizeof(status->step_bemf_min_v));
    memcpy(status->step_bemf_max_v, sensorless->step_bemf_max_v, sizeof(status->step_bemf_max_v));
    memcpy(status->step_bemf_waveform_v, sensorless->step_bemf_waveform_v,
           sizeof(status->step_bemf_waveform_v));
    memcpy(status->step_floating_waveform_v, sensorless->step_floating_waveform_v,
           sizeof(status->step_floating_waveform_v));
    memcpy(status->step_neutral_waveform_v, sensorless->step_neutral_waveform_v,
           sizeof(status->step_neutral_waveform_v));
    memcpy(status->step_candidate_min_abs_v, sensorless->step_candidate_min_abs_v,
           sizeof(status->step_candidate_min_abs_v));
    memcpy(status->step_candidate_min_time_s, sensorless->step_candidate_min_time_s,
           sizeof(status->step_candidate_min_time_s));
    memcpy(status->step_bus_candidate_min_abs_v, sensorless->step_bus_candidate_min_abs_v,
           sizeof(status->step_bus_candidate_min_abs_v));
    memcpy(status->step_bus_candidate_min_time_s, sensorless->step_bus_candidate_min_time_s,
           sizeof(status->step_bus_candidate_min_time_s));
    return true;
}
