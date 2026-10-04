/*
 * @Description: BLDC 控制状态快照
 * @Author: lkc
 */
#ifndef MOTOR_BLDC_STATUS_H
#define MOTOR_BLDC_STATUS_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "motor_bldc_sensorless.h"
#include "motor_types.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/**
 * @description: BLDC 控制及 BEMF 诊断状态的只读快照。
 * @detail: 由 Motor_Bldc_GetStatus 填充。时间单位为 s，电压为 V，积分为 V·s，占空比无量纲。
 *          调用者不得据此直接修改控制器内部状态。
 */
typedef struct {
    bool selected;
    bool running;
    float duty;
    float target_duty;
    uint8_t step;
    motor_bldc_direction_t direction;
    motor_bldc_sensorless_mode_t mode;
    motor_bldc_sensorless_fault_t fault;
    mc_fault_code motor_fault;
    float bemf_v;
    float bemf_min_v;
    float bemf_max_v;
    float sector_elapsed_s;
    float commutation_period_s;
    float minimum_commutation_period_s;
    float maximum_commutation_period_s;
    float active_minimum_period_s;
    float bemf_integral_vs;
    float bemf_reverse_integral_vs;
    float bemf_integral_threshold_vs;
    uint32_t bemf_sample_count;
    uint32_t integral_commutations;
    uint32_t forced_commutations;
    uint32_t phase_valid_samples;
    uint32_t phase_late_samples;
    uint32_t phase_blocked_samples;
    uint8_t detected_integral_mask;
    uint8_t integral_detect_settle_sectors;
    bool duty_period_enabled;
    bool integral_auto_detect_enabled;
    bool sample_upward;
    uint8_t consecutive_forced_commutations;
    uint32_t startup_valid_sectors;
    uint32_t startup_invalid_sectors;
    uint32_t startup_sync_resets;
    float startup_takeover_time_s;
    float startup_takeover_period_s;
    uint8_t startup_sync_sectors;
    uint16_t step_integral_commutations[6];
    uint16_t step_forced_commutations[6];
    uint8_t step_integral_detect_samples[6];
    float step_last_period_s[6];
    float step_last_integral_vs[6];
    float step_detected_integral_vs[6];
    float step_startup_integral_vs[6];
    float step_startup_reverse_integral_vs[6];
    float step_bemf_min_v[6];
    float step_bemf_max_v[6];
    float step_bemf_waveform_v[6][MOTOR_BLDC_BEMF_BIN_COUNT];
    float step_floating_waveform_v[6][MOTOR_BLDC_BEMF_BIN_COUNT];
    float step_neutral_waveform_v[6][MOTOR_BLDC_BEMF_BIN_COUNT];
    float step_candidate_min_abs_v[6][3];
    float step_candidate_min_time_s[6][3];
    float step_bus_candidate_min_abs_v[6][3];
    float step_bus_candidate_min_time_s[6][3];
} motor_bldc_status_t;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 从 BLDC 控制实例生成对外只读状态快照。
 * @detail: 纯数据复制，不访问硬件、不加锁；调用者负责保证输入在复制期间一致。
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
                           uint32_t bemf_sample_count);

/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
