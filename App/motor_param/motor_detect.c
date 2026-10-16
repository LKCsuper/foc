/*
 * @Description: 电流/电压偏移校准与仅返回 RAM 结果的 R/L/磁链检测。
 * @Author: lkc
 *
 * 偏移累加/结算在 motor_detect_offset（无 RTOS）；本文件包装 PWM、延时与日志。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "motor_detect.h"
#include "motor_detect_offset.h"
#include "flash_storage.h"
#include "motor_param.h"
#include "motor_sensorless.h"
#include "motor_hall.h"
#include "hal_motor.h"
#include "motor_protect.h"
#include "printf_color.h"
#include "motor.h"
#include "motor_current.h"
#include "motor_crit.h"
#include "motor_output.h"
#include "motor_timing.h"
#include "motor_foc.h"
#include "motor_bldc.h"
#include "motor_math.h"
#include "motor_encoder.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DETECT_OFFSET_ISR_TIMEOUT_MS (2000U)
#define MOTOR_DETECT_CURRENT_LIMIT_A (4.0f)
#define MOTOR_DETECT_CURRENT_HARD_LIMIT_A (8.0f)
#define MOTOR_DETECT_CURRENT_TRIP_S (0.001f)
#define MOTOR_DETECT_MIN_BUS_VOLTAGE_V (8.0f)
#define MOTOR_DETECT_MIN_CURRENT_A (0.3f)
#define MOTOR_DETECT_MAX_CURRENT_A (2.0f)
#define MOTOR_DETECT_RESISTANCE_VOLTAGE_V (5.0f)
#define MOTOR_DETECT_RESISTANCE_RAMP_S (0.8f)
/* 2804高电阻、保守Ki下0.2s等待仍有建流误差；延长等待后再取同长度均值。 */
/* 小电机低电流检测延长稳态与平均窗口，保留原漂移/斜率验收门槛。 */
#define MOTOR_DETECT_RESISTANCE_SETTLE_S (1.5f)
#define MOTOR_DETECT_RESISTANCE_SAMPLE_S (1.0f)
#define MOTOR_DETECT_RESISTANCE_MIN_SAMPLES (100U)
#define MOTOR_DETECT_RESISTANCE_MIN_OHM (0.005f)
#define MOTOR_DETECT_RESISTANCE_MAX_OHM (10.0f)
#define MOTOR_DETECT_RESISTANCE_FIT_POINTS (3U)
#define MOTOR_DETECT_RESISTANCE_FIT_MIN_R2 (0.95f)
#define MOTOR_DETECT_RESISTANCE_MIN_TRACK_RATIO (0.90f)
#define MOTOR_DETECT_RESISTANCE_MIN_SPAN_RATIO (0.50f)
/* 诊断门槛，不代表测量精度：窗口漂移3%，相邻斜率差10%。 */
#define MOTOR_DETECT_RESISTANCE_MAX_DRIFT_RATIO (0.03f)
#define MOTOR_DETECT_RESISTANCE_MAX_SLOPE_RATIO (0.10f)
/* L脉冲路径仍使用线性SVM指令电压，不包含死区/管压降；R已改用PWM重构。 */
#define MOTOR_DETECT_APPLIED_VOLTAGE_SCALE (1.0f)
#define MOTOR_DETECT_INDUCTANCE_ALIGN_V (3.0f)
#define MOTOR_DETECT_INDUCTANCE_ZERO_V (2.0f)
#define MOTOR_DETECT_INDUCTANCE_ALIGN_S (0.8f)
#define MOTOR_DETECT_INDUCTANCE_ZERO_S (0.2f)
#define MOTOR_DETECT_INDUCTANCE_PULSE_V (0.5f)
#define MOTOR_DETECT_INDUCTANCE_HALF_CYCLES (8U)
#define MOTOR_DETECT_INDUCTANCE_TARGET_PULSES_PER_DIRECTION (24U)
#define MOTOR_DETECT_INDUCTANCE_MAX_PULSES_PER_DIRECTION (72U)
#define MOTOR_DETECT_INDUCTANCE_MIN_SLOPE (1.0f)
#define MOTOR_DETECT_INDUCTANCE_MIN_H (5.0e-6f)
#define MOTOR_DETECT_INDUCTANCE_MAX_H (0.05f)
#define MOTOR_DETECT_INDUCTANCE_STABILITY_SCALE (0.9f)
#define MOTOR_DETECT_PI (3.14159265358f)
#define MOTOR_DETECT_TIMEOUT_RESISTANCE_MS (4000U)
#define MOTOR_DETECT_TIMEOUT_INDUCTANCE_MS (5000U)
#define MOTOR_DETECT_FLUX_ALIGN_S (1.0f)
#define MOTOR_DETECT_FLUX_RAMP_S (8.0f)
#define MOTOR_DETECT_FLUX_TRANSITION_S (1.0f)
#define MOTOR_DETECT_FLUX_VOLTAGE_LIMIT_V (8.0f)
#define MOTOR_DETECT_FLUX_MIN_RUN_DUTY (0.03f)
#define MOTOR_DETECT_FLUX_DUTY_COLLAPSE_RATIO (0.70f)
#define MOTOR_DETECT_FLUX_COAST_SETTLE_S (0.01f)
#define MOTOR_DETECT_FLUX_COAST_SAMPLE_S (0.50f)
#define MOTOR_DETECT_FLUX_COAST_MIN_ERPM (100.0f)
#define MOTOR_DETECT_FLUX_MIN_TARGET_SPEED_RATIO (0.35f)
#define MOTOR_DETECT_FLUX_MIN_BEMF_V (0.08f)
#define MOTOR_DETECT_FLUX_MIN_SAMPLES (300U)
#define MOTOR_DETECT_FLUX_MIN_WB (1.0e-4f)
#define MOTOR_DETECT_FLUX_MAX_WB (0.2f)
#define MOTOR_DETECT_MIN_ELECTRICAL_RPM (200.0f)
#define MOTOR_DETECT_MAX_ELECTRICAL_RPM (3000.0f)
#define MOTOR_DETECT_VESC_CURRENT_TC_S (0.0015f)
#define MOTOR_DETECT_TIMEOUT_FLUX_MS (18000U)
#define MOTOR_DETECT_PWM_COOLDOWN_MS (50U)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
 * 校准任务会改 PWM 输出。置位后 DMA 中断仍采样换算，但不跑 FOC、不跑软件过流
 * （零偏未定时电流不可信）；ADC_Value[] 由 DMA 硬件持续刷新。
 * 电流零点改由 ISR 按 TIM1 方向分沿累加：Hall FOC 只用下降沿，任务 1 ms 抽
 * ADC_Value 会混入上升沿，导致约 2~3 计数共模偏差。
 */
static volatile bool s_detect_active = false;
static volatile bool s_offset_isr_enable = false;
static volatile uint32_t s_offset_down_n = 0U;
static volatile uint32_t s_offset_up_n = 0U;
static motor_detect_offset_t s_offset_down;
static motor_detect_offset_t s_offset_up;
typedef enum {
    MOTOR_DETECT_STAGE_IDLE = 0,
    MOTOR_DETECT_STAGE_RESISTANCE,
    MOTOR_DETECT_STAGE_INDUCTANCE,
    MOTOR_DETECT_STAGE_FLUX,
} motor_detect_stage_t;

typedef struct {
    volatile motor_detect_stage_t stage;
    volatile bool done;
    volatile bool success;
    volatile motor_detect_error_t error;
    volatile int fault;
    float elapsed;
    float current_a;
    float vd;
    float vq;
    float accum_a;
    float accum_b;
    float resistance_voltage_avg;
    float resistance_current_avg;
    /* R诊断：均值/波动与模长统计并列，不改变控制和拟合输入。 */
    float r_id_sum;
    float r_iq_sum;
    float r_id_square_sum;
    float r_iq_square_sum;
    float r_v_alpha_sum;
    float r_v_beta_sum;
    float r_current_sum;
    float r_phase_sum[3];
    uint32_t r_phase_negative[3];
    uint32_t r_phase_near_zero[3];
    float r_bus_sum;
    float r_window_i[2];
    float r_window_v[2];
    uint32_t r_window_n[2];
    float measured_resistance;
    float max_phase_current;
    float overcurrent_elapsed;
    uint32_t samples;
    uint32_t pulse_position;
    uint32_t pulse_count;
    uint32_t valid_pulses;
    float pulse_start_current;
    float injection_theta;
    float theta;
    float electrical_rpm_target;
    float electrical_rpm_now;
    float flux_duty_still;
    float flux_duty_now;
    float flux_duty_max;
    float flux_coast_elapsed;
    float flux_bemf_angle_last;
    float flux_angle_sum;
    /* scan按配对时间积分：前两项单位分别为电气rad、V*s。 */
    float flux_encoder_omega_sum, flux_encoder_voltage_sum;
    float flux_pair_bemf_angle, flux_pair_time; /* rad、s，与编码器使用相同样本区间 */
    uint32_t flux_encoder_count, flux_encoder_attempts;
    float inductance_sum[MOTOR_DETECT_INDUCTANCE_DIRECTIONS];
    uint32_t inductance_valid[MOTOR_DETECT_INDUCTANCE_DIRECTIONS];
    bool pulse_started;
    bool rotation_started;
    bool flux_coast_started;
    bool flux_bemf_angle_valid;
} motor_detect_control_t;

static motor_detect_control_t s_detect_control;
static motor_detect_result_t s_detect_result = {.version = MOTOR_DETECT_RESULT_VERSION};
static volatile bool s_detect_abort_requested;
static bool s_flux_scan;
static float s_flux_reference_dt;
/* scan专用滑行轨迹，ISR只存储，关桥后在任务中打印。 */
static struct {
    float time, alpha, beta, erpm, age;
    uint16_t count;
} s_flux_trace[64];
static uint32_t s_flux_trace_n;
static float s_flux_raw_angle;
static bool s_flux_raw_valid;
static float s_flux_bemf_unwrapped, s_flux_raw_bemf;
static uint32_t s_flux_raw_stamp;

static void Detect_PrintFluxTrace(void)
{
    uint32_t i;
    for (i = 0U; i < s_flux_trace_n; i++) {
        PRINTF("Flux trace: t=%.6f ab=%.6f/%.6f enc=%.1f age=%.6f count=%u\r\n",
               (double)s_flux_trace[i].time, (double)s_flux_trace[i].alpha,
               (double)s_flux_trace[i].beta, (double)s_flux_trace[i].erpm,
               (double)s_flux_trace[i].age, (unsigned)s_flux_trace[i].count);
    }
}
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 校准任务侧查询是否因保护锁存应中止。
 * @return {bool} true 应停采并 PwmStop
 * @author: lkc
 */
static bool Detect_ShouldAbort(void)
{
    return Motor_Protect_IsFaultLatched();
}

/**
 * @description: 将当前 RAM 中的配置/校准写入 Flash（须已停机且未占用功率桥）
 * @return {bool} true=保存成功
 * @author: lkc
 */
static bool Detect_SaveOffsetToFlash(void)
{
    if (!FlashStorage_Save(&motor)) {
        PRINTF("Detect offset: flash save failed\r\n");
        return false;
    }
    PRINTF("Detect offset: saved to flash\r\n");
    return true;
}

/**
 * @description: 打印某一沿的电流零点均值（ADC 计数）。
 * @param {const char *} tag 沿名称
 * @param {const motor_detect_offset_t *} st 累加状态
 * @return {*}
 */
static void Detect_PrintEdgeMeans(const char *tag, const motor_detect_offset_t *st)
{
    float n;

    if (tag == NULL || st == NULL || st->count == 0U) {
        PRINTF("Detect offset: %s n=0\r\n", (tag != NULL) ? tag : "?");
        return;
    }
    n = (float)st->count;
    PRINTF("Detect offset: %s n=%lu CURR %.2f %.2f %.2f\r\n", tag, (unsigned long)st->count,
           (double)(st->i_sum[0] / n), (double)(st->i_sum[1] / n), (double)(st->i_sum[2] / n));
    PRINTF("Detect offset: %s p2p %.1f %.1f %.1f sum_span %.1f (%.3f A)\r\n", tag,
           (double)(st->i_max[0] - st->i_min[0]), (double)(st->i_max[1] - st->i_min[1]),
           (double)(st->i_max[2] - st->i_min[2]), (double)(st->sum_max - st->sum_min),
           (double)((st->sum_max - st->sum_min) * Hal_Motor_CurrentScale()));
}

static float Detect_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static bool Detect_FitResistance(const float *current, const float *voltage, uint32_t count, float *resistance,
                                 float *voltage_offset, float *r_squared)
{
    float sum_i = 0.0f;
    float sum_v = 0.0f;
    float sum_ii = 0.0f;
    float sum_iv = 0.0f;
    float ss_tot = 0.0f;
    float ss_res = 0.0f;
    float denominator;
    float mean_v;
    uint32_t i;

    if (current == NULL || voltage == NULL || resistance == NULL || voltage_offset == NULL || r_squared == NULL ||
        count < 2U) {
        return false;
    }
    for (i = 0U; i < count; i++) {
        if (!isfinite(current[i]) || !isfinite(voltage[i])) return false;
        sum_i += current[i];
        sum_v += voltage[i];
        sum_ii += current[i] * current[i];
        sum_iv += current[i] * voltage[i];
    }
    denominator = (float)count * sum_ii - sum_i * sum_i;
    if (fabsf(denominator) < 1.0e-6f) return false;
    *resistance = ((float)count * sum_iv - sum_i * sum_v) / denominator;
    *voltage_offset = (sum_v - *resistance * sum_i) / (float)count;
    mean_v = sum_v / (float)count;
    for (i = 0U; i < count; i++) {
        float error = voltage[i] - (*resistance * current[i] + *voltage_offset);
        float deviation = voltage[i] - mean_v;
        ss_res += error * error;
        ss_tot += deviation * deviation;
    }
    *r_squared = ss_tot > 1.0e-9f ? 1.0f - ss_res / ss_tot : 0.0f;
    return isfinite(*resistance) && isfinite(*voltage_offset) && isfinite(*r_squared);
}

static float Detect_PhaseCurrentPeak(const MOTOR_PARAM_S *motor_param)
{
    float peak = fabsf(motor_param->ia);
    if (fabsf(motor_param->ib) > peak) {
        peak = fabsf(motor_param->ib);
    }
    if (fabsf(motor_param->ic) > peak) {
        peak = fabsf(motor_param->ic);
    }
    return peak;
}

static bool Detect_CurrentTooHigh(const MOTOR_PARAM_S *motor_param, float dt)
{
    float peak = Detect_PhaseCurrentPeak(motor_param);
    if (peak > s_detect_control.max_phase_current) {
        s_detect_control.max_phase_current = peak;
    }
    if (peak >= MOTOR_DETECT_CURRENT_HARD_LIMIT_A) return true;
    if (peak > MOTOR_DETECT_CURRENT_LIMIT_A) {
        s_detect_control.overcurrent_elapsed += dt;
    } else {
        s_detect_control.overcurrent_elapsed = 0.0f;
    }
    return s_detect_control.overcurrent_elapsed >= MOTOR_DETECT_CURRENT_TRIP_S;
}

static void Detect_ControlReset(MOTOR_PARAM_S *motor_param)
{
    motor_param->ol_vd_int = 0.0f;
    motor_param->ol_vq_int = 0.0f;
    motor_param->Id = 0.0f;
    motor_param->Iq = 0.0f;
}

static void Detect_RunCurrentDq(MOTOR_PARAM_S *motor_param, float theta, float id_target, float iq_target,
                                float voltage_limit, float dt, float *vd, float *vq)
{
    motor_dq_voltage_t voltage;
    float safe_limit = motor_param->v_bus * 0.5f;

    if (voltage_limit < safe_limit) {
        safe_limit = voltage_limit;
    }
    Motor_Current_FeedbackUpdateDq(motor_param, theta);
    voltage = Motor_Current_RunPi(motor_param, id_target, iq_target, dt, safe_limit);
    Motor_OutputVoltageLinear(motor_param, voltage.vd, voltage.vq, theta, motor_param->v_bus);
    if (vd != NULL) {
        *vd = voltage.vd;
    }
    if (vq != NULL) {
        *vq = voltage.vq;
    }
}

static void Detect_RunCurrent(MOTOR_PARAM_S *motor_param, float id_target, float voltage_limit, float dt, float *vd,
                              float *vq)
{
    Detect_RunCurrentDq(motor_param, 0.0f, id_target, 0.0f, voltage_limit, dt, vd, vq);
}

static void Detect_RunVoltage(MOTOR_PARAM_S *motor_param, float vd, float theta)
{
    Motor_Current_FeedbackUpdateDq(motor_param, theta);
    Motor_OutputVoltageLinear(motor_param, vd, 0.0f, theta, motor_param->v_bus);
}

static void Detect_FinishFromIsr(bool success, motor_detect_error_t error)
{
    Motor_OutputSafe(&motor);
    Hal_Motor_PwmStop();
    s_detect_control.success = success;
    s_detect_control.error = error;
    s_detect_control.done = true;
    s_detect_control.stage = MOTOR_DETECT_STAGE_IDLE;
}

static void Detect_RunResistanceStep(MOTOR_PARAM_S *motor_param, float dt)
{
    float total_time = MOTOR_DETECT_RESISTANCE_RAMP_S + MOTOR_DETECT_RESISTANCE_SETTLE_S +
                       MOTOR_DETECT_RESISTANCE_SAMPLE_S;
    float ramp;
    float id_target;

    s_detect_control.elapsed += dt;
    ramp = Detect_Clamp(s_detect_control.elapsed / MOTOR_DETECT_RESISTANCE_RAMP_S, 0.0f, 1.0f);
    id_target = s_detect_control.current_a * ramp;
    Detect_RunCurrent(motor_param, id_target, MOTOR_DETECT_RESISTANCE_VOLTAGE_V, dt, &s_detect_control.vd,
                      &s_detect_control.vq);

    if (Detect_CurrentTooHigh(motor_param, dt)) {
        Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
        return;
    }
    if (s_detect_control.elapsed > MOTOR_DETECT_RESISTANCE_RAMP_S + MOTOR_DETECT_RESISTANCE_SETTLE_S) {
        float current_mag = sqrtf(motor_param->Id * motor_param->Id + motor_param->Iq * motor_param->Iq);
        float v_alpha;
        float v_beta;
        float voltage_mag;
        /* 与无感共用PWM电压估计；有效死区未标定，暂禁用补偿。 */
        if (!Motor_OutputEstimateVoltage(motor_param, Hal_Motor_PwmPeriodCounts(), motor_param->v_bus,
                                         0.0f, &v_alpha, &v_beta)) {
            Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_RESISTANCE);
            return;
        }
        voltage_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
        if (!isfinite(current_mag) || !isfinite(voltage_mag)) {
            Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_RESISTANCE);
            return;
        }
        if (current_mag > s_detect_control.current_a * 0.25f) {
            uint32_t half = s_detect_control.elapsed < total_time -
                            MOTOR_DETECT_RESISTANCE_SAMPLE_S * 0.5f ? 0U : 1U;
            s_detect_control.r_window_i[half] += motor_param->Id;
            s_detect_control.r_window_v[half] += v_alpha;
            s_detect_control.r_window_n[half]++;
            s_detect_control.accum_a += voltage_mag;
            s_detect_control.accum_b += current_mag;
            s_detect_control.r_id_sum += motor_param->Id;
            s_detect_control.r_iq_sum += motor_param->Iq;
            s_detect_control.r_id_square_sum += motor_param->Id * motor_param->Id;
            s_detect_control.r_iq_square_sum += motor_param->Iq * motor_param->Iq;
            s_detect_control.r_v_alpha_sum += v_alpha;
            s_detect_control.r_v_beta_sum += v_beta;
            s_detect_control.r_current_sum += motor_param->ia + motor_param->ib + motor_param->ic;
            {
                float phase[3] = {motor_param->ia, motor_param->ib, motor_param->ic};
                uint32_t channel;
                for (channel = 0U; channel < 3U; channel++) {
                    s_detect_control.r_phase_sum[channel] += phase[channel];
                    if (phase[channel] < 0.0f) {
                        s_detect_control.r_phase_negative[channel]++;
                    }
                    /* 仅诊断阈值，不用于滤除样本、控制或质量验收。 */
                    if (fabsf(phase[channel]) < 0.1f) {
                        s_detect_control.r_phase_near_zero[channel]++;
                    }
                }
                s_detect_control.r_bus_sum += motor_param->v_bus;
            }
            s_detect_control.samples++;
        }
    }
    if (s_detect_control.elapsed >= total_time) {
        if (s_detect_control.samples < MOTOR_DETECT_RESISTANCE_MIN_SAMPLES || s_detect_control.accum_b <= 0.0f) {
            Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_RESISTANCE);
            return;
        }
        s_detect_control.resistance_voltage_avg = s_detect_control.accum_a / (float)s_detect_control.samples;
        s_detect_control.resistance_current_avg = s_detect_control.accum_b / (float)s_detect_control.samples;
        Detect_FinishFromIsr(true, MOTOR_DETECT_ERROR_NONE);
    }
}

static void Detect_RunInductanceStep(MOTOR_PARAM_S *motor_param, float dt)
{
    float pulse_time_start = MOTOR_DETECT_INDUCTANCE_ALIGN_S + MOTOR_DETECT_INDUCTANCE_ZERO_S;
    uint32_t direction;

    s_detect_control.elapsed += dt;
    if (s_detect_control.elapsed < MOTOR_DETECT_INDUCTANCE_ALIGN_S) {
        float ramp = Detect_Clamp(s_detect_control.elapsed / MOTOR_DETECT_INDUCTANCE_ALIGN_S, 0.0f, 1.0f);
        Detect_RunCurrent(motor_param, s_detect_control.current_a * ramp, MOTOR_DETECT_INDUCTANCE_ALIGN_V, dt, NULL,
                          NULL);
        if (Detect_CurrentTooHigh(motor_param, dt)) {
            Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
        }
        return;
    }
    if (s_detect_control.elapsed < pulse_time_start) {
        Detect_RunCurrent(motor_param, 0.0f, MOTOR_DETECT_INDUCTANCE_ZERO_V, dt, NULL, NULL);
        if (Detect_CurrentTooHigh(motor_param, dt)) {
            Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
        }
        return;
    }
    if (!s_detect_control.pulse_started) {
        Detect_ControlReset(motor_param);
        s_detect_control.pulse_started = true;
    }
    {
        float sign = (s_detect_control.pulse_count & 1U) ? -1.0f : 1.0f;
        float voltage = sign * MOTOR_DETECT_INDUCTANCE_PULSE_V;
        direction = s_detect_control.pulse_count / MOTOR_DETECT_INDUCTANCE_MAX_PULSES_PER_DIRECTION;
        if (direction >= MOTOR_DETECT_INDUCTANCE_DIRECTIONS) {
            direction = MOTOR_DETECT_INDUCTANCE_DIRECTIONS - 1U;
        }
        s_detect_control.injection_theta =
            (float)direction * (MOTOR_DETECT_PI / (float)MOTOR_DETECT_INDUCTANCE_DIRECTIONS);
        Detect_RunVoltage(motor_param, voltage, s_detect_control.injection_theta);
        if (s_detect_control.pulse_position == 0U) {
            s_detect_control.pulse_start_current = motor_param->Id;
        }
        if (s_detect_control.pulse_position >= MOTOR_DETECT_INDUCTANCE_HALF_CYCLES - 1U) {
            float current_end = motor_param->Id;
            float current_mid = 0.5f * (s_detect_control.pulse_start_current + current_end);
            float slope_dt = (float)(MOTOR_DETECT_INDUCTANCE_HALF_CYCLES - 1U) * dt;
            float slope = (current_end - s_detect_control.pulse_start_current) / slope_dt;
            float effective_voltage = voltage * MOTOR_DETECT_APPLIED_VOLTAGE_SCALE -
                                      s_detect_control.measured_resistance * current_mid;
            /* 恢复旧首尾两点法；指令电压与真实端电压的误差仍需独立核验。 */
            if (fabsf(slope) > MOTOR_DETECT_INDUCTANCE_MIN_SLOPE) {
                float inductance = effective_voltage / slope;
                if (inductance > MOTOR_DETECT_INDUCTANCE_MIN_H && inductance < MOTOR_DETECT_INDUCTANCE_MAX_H) {
                    s_detect_control.inductance_sum[direction] += inductance;
                    s_detect_control.inductance_valid[direction]++;
                    s_detect_control.valid_pulses++;
                }
            }
            s_detect_control.pulse_count++;
            s_detect_control.pulse_position = 0U;
        } else {
            s_detect_control.pulse_position++;
        }
    }
    if (Detect_CurrentTooHigh(motor_param, dt)) {
        Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
        return;
    }
    if (s_detect_control.pulse_count >=
        MOTOR_DETECT_INDUCTANCE_DIRECTIONS * MOTOR_DETECT_INDUCTANCE_MAX_PULSES_PER_DIRECTION) {
        float minimum = MOTOR_DETECT_INDUCTANCE_MAX_H;
        float maximum = 0.0f;
        uint32_t i;

        for (i = 0U; i < MOTOR_DETECT_INDUCTANCE_DIRECTIONS; i++) {
            float value;
            if (s_detect_control.inductance_valid[i] < MOTOR_DETECT_INDUCTANCE_TARGET_PULSES_PER_DIRECTION) {
                Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_INDUCTANCE);
                return;
            }
            value = MOTOR_DETECT_INDUCTANCE_STABILITY_SCALE * s_detect_control.inductance_sum[i] /
                    (float)s_detect_control.inductance_valid[i];
            s_detect_result.inductance_direction_h[i] = value;
            if (value < minimum) {
                minimum = value;
            }
            if (value > maximum) {
                maximum = value;
            }
        }
        s_detect_result.inductance_d_h = minimum;
        s_detect_result.inductance_q_h = maximum;
        s_detect_result.inductance_h = 0.5f * (minimum + maximum);
        s_detect_result.inductance_valid = true;
        Detect_FinishFromIsr(true, MOTOR_DETECT_ERROR_NONE);
    }
}

static void Detect_RunFluxStep(MOTOR_PARAM_S *motor_param, float dt)
{
    float rotation_time;

    s_detect_control.elapsed += dt;
    if (s_detect_control.elapsed < MOTOR_DETECT_FLUX_ALIGN_S) {
        float ramp = Detect_Clamp(s_detect_control.elapsed / MOTOR_DETECT_FLUX_ALIGN_S, 0.0f, 1.0f);
        Detect_RunCurrentDq(motor_param, 0.0f, s_detect_control.current_a * ramp, 0.0f,
                            MOTOR_DETECT_FLUX_VOLTAGE_LIMIT_V, dt, &s_detect_control.vd, &s_detect_control.vq);
        if (motor_param->v_bus > 1.0f && ramp > 0.9f) {
            s_detect_control.flux_duty_still =
                sqrtf(s_detect_control.vd * s_detect_control.vd + s_detect_control.vq * s_detect_control.vq) /
                motor_param->v_bus;
        }
        if (Detect_CurrentTooHigh(motor_param, dt)) {
            Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
        }
        return;
    }

    if (!s_detect_control.rotation_started) {
        Detect_ControlReset(motor_param);
        s_detect_control.rotation_started = true;
        s_detect_control.theta = 0.0f;
        s_detect_control.electrical_rpm_now = 0.0f;
    }
    rotation_time = s_detect_control.elapsed - MOTOR_DETECT_FLUX_ALIGN_S;

    if (s_detect_control.flux_coast_started) {
        float v_alpha;
        float v_beta;
        float voltage_mag;
        float angle;
        float delta;

        s_detect_control.flux_coast_elapsed += dt;
        clarke_transform(motor_param->va, motor_param->vb, motor_param->vc, &v_alpha, &v_beta);
        if (s_flux_scan && s_flux_trace_n < 64U &&
            s_detect_control.flux_coast_elapsed >= (float)s_flux_trace_n * 0.008f) {
            s_flux_trace[s_flux_trace_n].time = s_detect_control.flux_coast_elapsed;
            s_flux_trace[s_flux_trace_n].alpha = v_alpha;
            s_flux_trace[s_flux_trace_n].beta = v_beta;
            s_flux_trace[s_flux_trace_n].erpm = Motor_Encoder_GetMechRpm() * motor_param->ol_pole_pairs;
            s_flux_trace[s_flux_trace_n].age = Motor_Encoder_GetSampleAge();
            s_flux_trace[s_flux_trace_n].count = Motor_Encoder_GetCount();
            s_flux_trace_n++;
        }
        voltage_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
        angle = atan2f(v_beta, v_alpha);
        if (!s_detect_control.flux_bemf_angle_valid) {
            s_detect_control.flux_bemf_angle_last = angle;
            s_detect_control.flux_bemf_angle_valid = true;
            return;
        }
        delta = angle - s_detect_control.flux_bemf_angle_last;
        while (delta > MOTOR_DETECT_PI) {
            delta -= 2.0f * MOTOR_DETECT_PI;
        }
        while (delta < -MOTOR_DETECT_PI) {
            delta += 2.0f * MOTOR_DETECT_PI;
        }
        s_detect_control.flux_bemf_angle_last = angle;
        if (s_flux_scan) {
            s_flux_bemf_unwrapped += delta;
        }
        if (s_detect_control.flux_coast_elapsed > MOTOR_DETECT_FLUX_COAST_SETTLE_S &&
            voltage_mag >= MOTOR_DETECT_FLUX_MIN_BEMF_V) {
            s_detect_control.flux_angle_sum += delta;
            s_detect_control.accum_a += voltage_mag;
            s_detect_control.accum_b += dt;
            s_detect_control.samples++;
            /* 约1kHz抽样；仅新鲜有效测速与同拍电压配对，不改变控制反馈。 */
            if (s_flux_scan && (s_detect_control.samples % 16U) == 0U &&
                voltage_mag >= 2.0f * MOTOR_DETECT_FLUX_MIN_BEMF_V) {
                float age = Motor_Encoder_GetSampleAge();
                uint32_t stamp = Motor_Encoder_GetSampleTimeUs();
                float time = (float)(uint32_t)(stamp - s_flux_raw_stamp) * 1.0e-6f;
                float mech = Motor_Encoder_GetMechAngle();
                float interval = time;
                float change = mech - s_flux_raw_angle;
                float erpm = 0.0f;
                while (change > MOTOR_DETECT_PI) change -= 2.0f * MOTOR_DETECT_PI;
                while (change < -MOTOR_DETECT_PI) change += 2.0f * MOTOR_DETECT_PI;
                /* 快速滑行不能用PLL平滑测速；相邻新鲜原始角度按实际时间差配对。 */
                if (s_flux_raw_valid && interval > 0.0005f && interval <= 0.003f) {
                    erpm = change / interval * motor_param->ol_pole_pairs *
                           (60.0f / (2.0f * MOTOR_DETECT_PI));
                }
                s_detect_control.flux_encoder_attempts++;
                if (Motor_Encoder_IsSpeedValid() && !Motor_Encoder_IsSignalFault() &&
                    age <= 0.005f && isfinite(erpm) && erpm > 100.0f &&
                    voltage_mag >= 2.0f * MOTOR_DETECT_FLUX_MIN_BEMF_V) {
                    /* 同一时间区间累计角位移与电压，不能比较门槛裁剪后的两种均值。 */
                    s_detect_control.flux_encoder_omega_sum += change * motor_param->ol_pole_pairs;
                    s_detect_control.flux_encoder_voltage_sum += voltage_mag * interval;
                    s_detect_control.flux_pair_bemf_angle += s_flux_bemf_unwrapped - s_flux_raw_bemf;
                    s_detect_control.flux_pair_time += interval;
                    s_detect_control.flux_encoder_count++;
                }
                s_flux_raw_valid = Motor_Encoder_IsSpeedValid() && !Motor_Encoder_IsSignalFault() &&
                                   age <= 0.005f && isfinite(mech);
                s_flux_raw_angle = mech;
                s_flux_raw_stamp = stamp;
                s_flux_raw_bemf = s_flux_bemf_unwrapped;
            }
        }
        if (s_detect_control.flux_coast_elapsed >= MOTOR_DETECT_FLUX_COAST_SAMPLE_S) {
            float omega = 0.0f;
            float voltage_average = 0.0f;
            float linkage = 0.0f;
            if (s_detect_control.samples > 0U && s_detect_control.accum_b > 0.0f) {
                omega = s_detect_control.flux_angle_sum / s_detect_control.accum_b;
                voltage_average = s_detect_control.accum_a / (float)s_detect_control.samples;
            }
            if (s_flux_scan && s_detect_control.flux_pair_time > 0.0f) {
                omega = s_detect_control.flux_pair_bemf_angle / s_detect_control.flux_pair_time;
                voltage_average = s_detect_control.flux_encoder_voltage_sum / s_detect_control.flux_pair_time;
            }
            s_detect_control.electrical_rpm_now = fabsf(omega) * (60.0f / (2.0f * MOTOR_DETECT_PI));
            s_detect_control.resistance_voltage_avg = voltage_average;
            if (fabsf(omega) > 1.0e-6f) {
                linkage = voltage_average / fabsf(omega);
            }
            s_detect_result.flux_linkage_wb = linkage;
            if (s_detect_control.samples < MOTOR_DETECT_FLUX_MIN_SAMPLES ||
                s_detect_control.electrical_rpm_now < MOTOR_DETECT_FLUX_COAST_MIN_ERPM ||
                s_detect_control.electrical_rpm_now <
                    s_detect_control.electrical_rpm_target * MOTOR_DETECT_FLUX_MIN_TARGET_SPEED_RATIO ||
                voltage_average < MOTOR_DETECT_FLUX_MIN_BEMF_V || linkage < MOTOR_DETECT_FLUX_MIN_WB ||
                linkage > MOTOR_DETECT_FLUX_MAX_WB) {
                Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_FLUX);
                return;
            }
            s_detect_result.flux_valid = true;
            Detect_FinishFromIsr(true, MOTOR_DETECT_ERROR_NONE);
        }
        return;
    }

    {
        float transition = Detect_Clamp(rotation_time / MOTOR_DETECT_FLUX_TRANSITION_S, 0.0f, 1.0f);
        float speed_ramp = Detect_Clamp(rotation_time / MOTOR_DETECT_FLUX_RAMP_S, 0.0f, 1.0f);
        float id_target = s_detect_control.current_a * (1.0f - transition);
        float iq_target = s_detect_control.current_a * transition;

        s_detect_control.electrical_rpm_now = s_detect_control.electrical_rpm_target * speed_ramp;
        s_detect_control.theta += s_detect_control.electrical_rpm_now * (2.0f * MOTOR_DETECT_PI / 60.0f) * dt;
        while (s_detect_control.theta >= 2.0f * MOTOR_DETECT_PI) {
            s_detect_control.theta -= 2.0f * MOTOR_DETECT_PI;
        }
        Detect_RunCurrentDq(motor_param, s_detect_control.theta, id_target, iq_target,
                            MOTOR_DETECT_FLUX_VOLTAGE_LIMIT_V, dt, &s_detect_control.vd, &s_detect_control.vq);
        if (motor_param->v_bus > 1.0f) {
            s_detect_control.flux_duty_now =
                sqrtf(s_detect_control.vd * s_detect_control.vd + s_detect_control.vq * s_detect_control.vq) /
                motor_param->v_bus;
            if (s_detect_control.flux_duty_now > s_detect_control.flux_duty_max) {
                s_detect_control.flux_duty_max = s_detect_control.flux_duty_now;
            }
        }
    }
    if (Detect_CurrentTooHigh(motor_param, dt)) {
        Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_CURRENT);
        return;
    }
    if (rotation_time >= MOTOR_DETECT_FLUX_RAMP_S) {
        float minimum_run_duty = MOTOR_DETECT_FLUX_MIN_RUN_DUTY;
        bool rotation_invalid;
        if (s_flux_scan) {
            float erpm = Motor_Encoder_GetMechRpm() * motor_param->ol_pole_pairs;
            /* scan有独立参考：实际速度验收代替受加速瞬态影响的峰值占空比比值。
             * 原单点检测仍保留原占空比检查，滑行BEMF质量检查也保留。 */
            rotation_invalid = !Motor_Encoder_IsSpeedValid() || Motor_Encoder_IsSignalFault() ||
                               Motor_Encoder_GetSampleAge() > 0.005f || !isfinite(erpm) ||
                               erpm < 0.8f * s_detect_control.electrical_rpm_target ||
                               erpm > 1.2f * s_detect_control.electrical_rpm_target;
        } else {
            rotation_invalid = s_detect_control.flux_duty_max > minimum_run_duty &&
                               s_detect_control.flux_duty_now <
                                   s_detect_control.flux_duty_max * MOTOR_DETECT_FLUX_DUTY_COLLAPSE_RATIO;
        }
        if (s_detect_control.flux_duty_now < minimum_run_duty ||
            rotation_invalid) {
            Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_FLUX);
            return;
        }
        Hal_Motor_PwmStop();
        s_detect_control.flux_coast_started = true;
        s_detect_control.flux_coast_elapsed = 0.0f;
        s_detect_control.flux_bemf_angle_valid = false;
        s_detect_control.flux_angle_sum = 0.0f;
        s_detect_control.accum_a = 0.0f;
        s_detect_control.accum_b = 0.0f;
        s_detect_control.samples = 0U;
    }
}

/**
 * @description: 等待 ISR 在升降沿各采满 samples_per_phase；零点采用下降沿。
 * @return {bool} false 表示保护锁存或超时
 */
static bool Detect_SampleIsrBlocking(void)
{
    uint32_t waited_ms = 0U;
    uint32_t need = s_offset_down.samples_per_phase;

    s_offset_down_n = 0U;
    s_offset_up_n = 0U;
    s_offset_isr_enable = true;
    while ((s_offset_down_n < need) || (s_offset_up_n < need)) {
        if (Detect_ShouldAbort()) {
            s_offset_isr_enable = false;
            return false;
        }
        osDelay(pdMS_TO_TICKS(1));
        waited_ms++;
        if (waited_ms >= DETECT_OFFSET_ISR_TIMEOUT_MS) {
            s_offset_isr_enable = false;
            PRINTF("Detect offset: ISR sample timeout down=%lu up=%lu need=%lu\r\n",
                   (unsigned long)s_offset_down_n, (unsigned long)s_offset_up_n, (unsigned long)need);
            return false;
        }
    }
    s_offset_isr_enable = false;
    osDelay(pdMS_TO_TICKS(1));
    return !Detect_ShouldAbort();
}

/**
 * @description: 校准三相电流零点和驱动时相电压偏移。
 * @detail: 三相 50% 中心对齐；电流零点由 ISR 按 TIM1 沿分别累加，写入下降沿均值
 *          （与 Hall FOC 顶部零矢量采样窗一致）。cal_undriven=true 时再采未驱动相电压。
 *          累加/结算走 Motor_Detect_Offset*；本函数负责停机、PWM、osDelay 与打印。
 *          persist=true 时成功后写入 Flash；中止路径不保存。
 * @param {bool} cal_undriven true 时额外校准未驱动相电压偏移
 * @return {uint32_t} 1 成功
 * @author: lkc
 */
static uint32_t Detect_OffsetRun(bool cal_undriven, bool persist)
{
    /*
     * 先占住互斥，再停机关通道。之后 DMA 中断只采样换算（不跑软件过流），不跑 FOC。
     */
    if (Motor_Protect_IsFaultLatched()) {
        PRINTF("Detect offset: blocked, fault=%s — run: olfault clear\r\n",
               Motor_Protect_GetFaultName(Motor_Protect_GetFault()));
        return 0U;
    }
    Motor_Protect_ClearFault();

    s_offset_isr_enable = false;
    s_detect_active = true;
    (void)Motor_RequestStop(&motor);
    Motor_Sensorless_Reset();
    Hal_Motor_PwmStop();
    /* 清残留 dq/相电流，避免上一拍故障快照量影响后续判断 */
    motor.ia = 0.0f;
    motor.ib = 0.0f;
    motor.ic = 0.0f;
    motor.Id = 0.0f;
    motor.Iq = 0.0f;
    PRINTF("Detect offset: begin\r\n");

    Motor_Detect_OffsetReset(&s_offset_down, cal_undriven, MOTOR_DETECT_OFFSET_DEFAULT_SAMPLES,
                             MOTOR_DETECT_OFFSET_DEFAULT_ADC_SCALE);
    Motor_Detect_OffsetReset(&s_offset_up, false, MOTOR_DETECT_OFFSET_DEFAULT_SAMPLES,
                             MOTOR_DETECT_OFFSET_DEFAULT_ADC_SCALE);
    s_offset_down_n = 0U;
    s_offset_up_n = 0U;

    osDelay(pdMS_TO_TICKS(1000));
    if (Detect_ShouldAbort()) {
        goto abort_detect;
    }

    Hal_Motor_PwmSetSafe();
    Hal_Motor_PwmStart();
    osDelay(pdMS_TO_TICKS(1));
    PRINTF("Detect offset: all phases 50%, ISR edge sample\r\n");
    if (!Detect_SampleIsrBlocking()) {
        goto abort_detect;
    }

    Hal_Motor_PwmStop();
    Detect_PrintEdgeMeans("down", &s_offset_down);
    Detect_PrintEdgeMeans("up", &s_offset_up);
    Motor_Detect_OffsetFinalizeDriven(&s_offset_down);
    Motor_Detect_OffsetApply(&s_offset_down, &motor, false);

    PRINTF("ADC_RAW [%d] [%d] [%d]\r\n", (int)Hal_Motor_CurrentAdc(0U), (int)Hal_Motor_CurrentAdc(1U),
           (int)Hal_Motor_CurrentAdc(2U));
    PRINTF("Current Offset [%f] [%f] [%f]\r\n", motor.currentOffset[0], motor.currentOffset[1], motor.currentOffset[2]);
    PRINTF("Volt Offset [%f] [%f] [%f]\r\n", motor.voltOffset[0], motor.voltOffset[1], motor.voltOffset[2]);

    if (s_offset_down.offset_warn) {
        PRINTF("WARNING: current offset > %.0f%% ADC range — check hardware\r\n",
               (double)MOTOR_DETECT_OFFSET_WARN_PERCENT);
    }

    if (cal_undriven) {
        uint32_t i;

        PRINTF("Detect offset: undriven voltage\r\n");
        osDelay(pdMS_TO_TICKS(10));

        for (i = 0U; i < s_offset_down.samples_per_phase; i++) {
            if (Detect_ShouldAbort()) {
                goto abort_detect;
            }
            (void)Motor_Detect_OffsetFeedUndriven(&s_offset_down, Hal_Motor_SenseVolts(0U), Hal_Motor_SenseVolts(1U),
                                                  Hal_Motor_SenseVolts(2U));
            osDelay(pdMS_TO_TICKS(1));
        }
        Hal_Motor_PwmStop();
        Motor_Detect_OffsetFinalizeUndriven(&s_offset_down);
        Motor_Detect_OffsetApply(&s_offset_down, &motor, true);
    }

    Hal_Motor_PwmStop();
    s_offset_isr_enable = false;
    s_detect_active = false;
    PRINTF("Detect offset: done\r\n");
    if (persist) {
        (void)Detect_SaveOffsetToFlash();
    }
    return 1U;

abort_detect:
    s_offset_isr_enable = false;
    Hal_Motor_PwmStop();
    s_detect_active = false;
    {
        float ipeak = fabsf(motor.ia);
        if (fabsf(motor.ib) > ipeak) {
            ipeak = fabsf(motor.ib);
        }
        if (fabsf(motor.ic) > ipeak) {
            ipeak = fabsf(motor.ic);
        }
        PRINTF("Detect offset: aborted (latched=%d fault=%d %s Ipeak=%.3fA)\r\n",
               Motor_Protect_IsFaultLatched() ? 1 : 0, (int)Motor_Protect_GetFault(),
               Motor_Protect_GetFaultName(Motor_Protect_GetFault()), (double)ipeak);
    }
    return 0U;
}

uint32_t DETECT_Offset(bool cal_undriven)
{
    return Detect_OffsetRun(cal_undriven, true);
}

/**
 * @description: 校准三相电流零点，只更新 RAM，不写 Flash。
 * @detail: 采样、关桥和中止条件与 DETECT_Offset 相同。供停机诊断调用，阻塞在应用任务中。
 * @param {bool} cal_undriven true 时额外校准未驱动相电压偏移
 * @return {uint32_t} 1 成功，0 被中止
 * @author: lkc
 */
uint32_t DETECT_OffsetRam(bool cal_undriven)
{
    return Detect_OffsetRun(cal_undriven, false);
}

/**
 * @description: 查询偏移校准是否占用功率桥。
 * @return {bool} true 时 ISR 只采样+保护，不跑 FOC
 * @author: lkc
 */
bool Motor_Detect_IsActive(void)
{
    return s_detect_active;
}

/**
 * @description: DMA 完成中断喂入一拍电流/电压，仅零偏 ISR 采集窗口内有效。
 * @param {bool} pwm_down true=TIM1 向下计数
 * @return {*}
 */
void Motor_Detect_IsrFeed(bool pwm_down)
{
    motor_detect_offset_t *st;
    volatile uint32_t *count_vis;

    if (s_detect_control.stage != MOTOR_DETECT_STAGE_IDLE) {
        float dt;

        /* R/L 检测与正常有感 FOC 一样只使用 PWM 向下计数侧。 */
        if (!pwm_down) return;
        dt = Motor_Timing_GetControlDt(&motor);
        if (s_detect_abort_requested) {
            Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_USER_ABORT);
            return;
        }
        if (motor.v_bus < MOTOR_DETECT_MIN_BUS_VOLTAGE_V) {
            Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_FAULT);
            return;
        }
        if (Motor_Protect_IsFaultLatched()) {
            s_detect_control.fault = (int)Motor_Protect_GetFault();
            Detect_FinishFromIsr(false, MOTOR_DETECT_ERROR_FAULT);
            return;
        }
        if (s_detect_control.stage == MOTOR_DETECT_STAGE_RESISTANCE) {
            Detect_RunResistanceStep(&motor, dt);
        } else if (s_detect_control.stage == MOTOR_DETECT_STAGE_INDUCTANCE) {
            Detect_RunInductanceStep(&motor, dt);
        } else if (s_detect_control.stage == MOTOR_DETECT_STAGE_FLUX) {
            /* 检测占用ISR时正常传感器更新被绕过，仅scan补充1kHz参考测速。 */
            if (s_flux_scan) {
                s_flux_reference_dt += dt;
                if (s_flux_reference_dt >= 0.001f) {
                    Motor_Encoder_UpdateReference(s_flux_reference_dt);
                    s_flux_reference_dt = 0.0f;
                }
            }
            Detect_RunFluxStep(&motor, dt);
        }
        return;
    }

    if (!s_offset_isr_enable) return;
    if (pwm_down) {
        st = &s_offset_down;
        count_vis = &s_offset_down_n;
    } else {
        st = &s_offset_up;
        count_vis = &s_offset_up_n;
    }
    if (st->count >= st->samples_per_phase) return;
    (void)Motor_Detect_OffsetFeedDrivenTriple(st, (float)Hal_Motor_CurrentAdc(0U), (float)Hal_Motor_CurrentAdc(1U),
                                              (float)Hal_Motor_CurrentAdc(2U), Hal_Motor_SenseVolts(0U),
                                              Hal_Motor_SenseVolts(1U), Hal_Motor_SenseVolts(2U));
    *count_vis = st->count;
}

static bool Detect_RunStage(motor_detect_stage_t stage, float current_a, uint32_t timeout_ms)
{
    uint32_t waited_ms = 0U;
    uint32_t primask;

    s_detect_control.done = false;
    s_flux_reference_dt = 0.0f;
    s_flux_trace_n = 0U;
    s_flux_raw_valid = false;
    s_flux_bemf_unwrapped = s_flux_raw_bemf = 0.0f;
    s_flux_raw_stamp = 0U;
    s_detect_control.flux_pair_bemf_angle = s_detect_control.flux_pair_time = 0.0f;
    s_detect_control.success = false;
    s_detect_control.error = MOTOR_DETECT_ERROR_NONE;
    s_detect_control.fault = 0;
    s_detect_control.elapsed = 0.0f;
    s_detect_control.current_a = current_a;
    s_detect_control.vd = 0.0f;
    s_detect_control.vq = 0.0f;
    s_detect_control.accum_a = 0.0f;
    s_detect_control.accum_b = 0.0f;
    s_detect_control.resistance_voltage_avg = 0.0f;
    s_detect_control.resistance_current_avg = 0.0f;
    s_detect_control.r_id_sum = 0.0f;
    s_detect_control.r_iq_sum = 0.0f;
    s_detect_control.r_id_square_sum = 0.0f;
    s_detect_control.r_iq_square_sum = 0.0f;
    s_detect_control.r_v_alpha_sum = 0.0f;
    s_detect_control.r_v_beta_sum = 0.0f;
    s_detect_control.r_current_sum = 0.0f;
    memset(s_detect_control.r_phase_sum, 0, sizeof(s_detect_control.r_phase_sum));
    memset(s_detect_control.r_phase_negative, 0, sizeof(s_detect_control.r_phase_negative));
    memset(s_detect_control.r_phase_near_zero, 0, sizeof(s_detect_control.r_phase_near_zero));
    s_detect_control.r_bus_sum = 0.0f;
    memset(s_detect_control.r_window_i, 0, sizeof(s_detect_control.r_window_i));
    memset(s_detect_control.r_window_v, 0, sizeof(s_detect_control.r_window_v));
    memset(s_detect_control.r_window_n, 0, sizeof(s_detect_control.r_window_n));
    s_detect_control.max_phase_current = 0.0f;
    s_detect_control.overcurrent_elapsed = 0.0f;
    s_detect_control.samples = 0U;
    s_detect_control.pulse_position = 0U;
    s_detect_control.pulse_count = 0U;
    s_detect_control.valid_pulses = 0U;
    s_detect_control.pulse_start_current = 0.0f;
    s_detect_control.injection_theta = 0.0f;
    s_detect_control.theta = 0.0f;
    s_detect_control.electrical_rpm_now = 0.0f;
    s_detect_control.flux_duty_still = 0.0f;
    s_detect_control.flux_duty_now = 0.0f;
    s_detect_control.flux_duty_max = 0.0f;
    s_detect_control.flux_coast_elapsed = 0.0f;
    s_detect_control.flux_bemf_angle_last = 0.0f;
    s_detect_control.flux_angle_sum = 0.0f;
    s_detect_control.flux_encoder_omega_sum = s_detect_control.flux_encoder_voltage_sum = 0.0f;
    s_detect_control.flux_encoder_count = s_detect_control.flux_encoder_attempts = 0U;
    memset(s_detect_control.inductance_sum, 0, sizeof(s_detect_control.inductance_sum));
    memset(s_detect_control.inductance_valid, 0, sizeof(s_detect_control.inductance_valid));
    s_detect_control.pulse_started = false;
    s_detect_control.rotation_started = false;
    s_detect_control.flux_coast_started = false;
    s_detect_control.flux_bemf_angle_valid = false;
    Detect_ControlReset(&motor);

    primask = Motor_Crit_Enter();
    s_detect_control.stage = stage;
    s_detect_active = true;
    Motor_Crit_Exit(primask);
    Motor_OutputSafe(&motor);
    Hal_Motor_PwmStart();

    while (!s_detect_control.done && waited_ms < timeout_ms) {
        osDelay(pdMS_TO_TICKS(1));
        waited_ms++;
    }
    if (!s_detect_control.done) {
        primask = Motor_Crit_Enter();
        s_detect_control.stage = MOTOR_DETECT_STAGE_IDLE;
        Motor_Crit_Exit(primask);
        Hal_Motor_PwmStop();
        s_detect_control.error = MOTOR_DETECT_ERROR_TIMEOUT;
        s_detect_control.success = false;
    }
    Hal_Motor_PwmStop();
    osDelay(pdMS_TO_TICKS(MOTOR_DETECT_PWM_COOLDOWN_MS));
    if (!s_detect_control.success) {
        s_detect_result.error = s_detect_control.error;
        s_detect_result.fault = s_detect_control.fault;
    }
    return s_detect_control.success;
}

static bool Detect_MotorRlInternal(float current_a, motor_detect_result_t *result, bool retain_active)
{
    float fit_current[MOTOR_DETECT_RESISTANCE_FIT_POINTS];
    float fit_voltage[MOTOR_DETECT_RESISTANCE_FIT_POINTS];
    float signed_current[MOTOR_DETECT_RESISTANCE_FIT_POINTS];
    float signed_voltage[MOTOR_DETECT_RESISTANCE_FIT_POINTS];
    float resistance = 0.0f;
    float voltage_offset = 0.0f;
    float r_squared = 0.0f;
    float fit_current_min = MOTOR_DETECT_MAX_CURRENT_A;
    float fit_current_max = 0.0f;
    float fit_target_min;
    float fit_target_max;
    bool fit_tracking_valid = true;
    uint32_t point;

    if (s_detect_active || Motor_Bldc_IsSelected()) {
        s_detect_result.error = MOTOR_DETECT_ERROR_BUSY;
        goto detect_done;
    }
    if (!isfinite(current_a)) {
        s_detect_result.error = MOTOR_DETECT_ERROR_INVALID_INPUT;
        goto detect_done;
    }
    if (motor.ol_enable) {
        s_detect_result.error = MOTOR_DETECT_ERROR_MOTOR_RUNNING;
        goto detect_done;
    }
    if (Motor_Protect_IsFaultLatched()) {
        s_detect_result.error = MOTOR_DETECT_ERROR_FAULT;
        s_detect_result.fault = (int)Motor_Protect_GetFault();
        goto detect_done;
    }

    memset(&s_detect_result, 0, sizeof(s_detect_result));
    s_detect_result.version = MOTOR_DETECT_RESULT_VERSION;
    current_a = Detect_Clamp(fabsf(current_a), MOTOR_DETECT_MIN_CURRENT_A, MOTOR_DETECT_MAX_CURRENT_A);
    fit_target_min = Detect_Clamp(current_a, MOTOR_DETECT_MIN_CURRENT_A, 1.0f);
    fit_target_max = 2.0f * fit_target_min;
    s_detect_result.test_current_a = current_a;
    (void)Motor_RequestStop(&motor);
    Motor_Sensorless_Reset();
    Hal_Motor_PwmStop();
    s_detect_active = true;

    PRINTF("Detect motor: fitting phase resistance at %.1f..%.1f A\r\n", (double)fit_target_min,
           (double)fit_target_max);
    for (point = 0U; point < MOTOR_DETECT_RESISTANCE_FIT_POINTS; point++) {
        float ratio = (float)point / (float)(MOTOR_DETECT_RESISTANCE_FIT_POINTS - 1U);
        float fit_target = fit_target_min + ratio * (fit_target_max - fit_target_min);
        if (!Detect_RunStage(MOTOR_DETECT_STAGE_RESISTANCE, fit_target, MOTOR_DETECT_TIMEOUT_RESISTANCE_MS)) {
            PRINTF("  R fit aborted: peak=%.3f A\r\n", (double)s_detect_control.max_phase_current);
            goto detect_finish;
        }
        fit_current[point] = s_detect_control.resistance_current_avg;
        fit_voltage[point] = s_detect_control.resistance_voltage_avg;
        signed_current[point] = s_detect_control.r_id_sum / (float)s_detect_control.samples;
        signed_voltage[point] = s_detect_control.r_v_alpha_sum / (float)s_detect_control.samples;
        {
            float i0, i1, v0, v1, i_drift, v_drift;
            if (s_detect_control.r_window_n[0] < MOTOR_DETECT_RESISTANCE_MIN_SAMPLES ||
                s_detect_control.r_window_n[1] < MOTOR_DETECT_RESISTANCE_MIN_SAMPLES) {
                s_detect_result.error = MOTOR_DETECT_ERROR_RESISTANCE;
                goto detect_finish;
            }
            i0 = s_detect_control.r_window_i[0] / (float)s_detect_control.r_window_n[0];
            i1 = s_detect_control.r_window_i[1] / (float)s_detect_control.r_window_n[1];
            v0 = s_detect_control.r_window_v[0] / (float)s_detect_control.r_window_n[0];
            v1 = s_detect_control.r_window_v[1] / (float)s_detect_control.r_window_n[1];
            i_drift = fabsf(i1 - i0) / fmaxf(fabsf(signed_current[point]), 0.01f);
            v_drift = fabsf(v1 - v0) / fmaxf(fabsf(signed_voltage[point]), 0.01f);
            PRINTF("  R stability: Id drift=%.2f%% Vd drift=%.2f%%\r\n",
                   (double)(i_drift * 100.0f), (double)(v_drift * 100.0f));
            {
                float count = (float)s_detect_control.samples;
                PRINTF("  R windows: I=%.5f/%.5f A V=%.5f/%.5f V down-only bus=%.3f V\r\n",
                       (double)i0, (double)i1, (double)v0, (double)v1,
                       (double)(s_detect_control.r_bus_sum / count));
                PRINTF("  R phases: mean=%.4f/%.4f/%.4f A neg=%.1f/%.1f/%.1f%% near0=%.1f/%.1f/%.1f%%\r\n",
                       (double)(s_detect_control.r_phase_sum[0] / count),
                       (double)(s_detect_control.r_phase_sum[1] / count),
                       (double)(s_detect_control.r_phase_sum[2] / count),
                       (double)(100.0f * s_detect_control.r_phase_negative[0] / count),
                       (double)(100.0f * s_detect_control.r_phase_negative[1] / count),
                       (double)(100.0f * s_detect_control.r_phase_negative[2] / count),
                       (double)(100.0f * s_detect_control.r_phase_near_zero[0] / count),
                       (double)(100.0f * s_detect_control.r_phase_near_zero[1] / count),
                       (double)(100.0f * s_detect_control.r_phase_near_zero[2] / count));
            }
            if (!isfinite(i_drift) || !isfinite(v_drift) ||
                i_drift > MOTOR_DETECT_RESISTANCE_MAX_DRIFT_RATIO ||
                v_drift > MOTOR_DETECT_RESISTANCE_MAX_DRIFT_RATIO) {
                PRINTF("  R rejected: sample window is not steady\r\n");
                s_detect_result.error = MOTOR_DETECT_ERROR_RESISTANCE;
                goto detect_finish;
            }
        }
        if (fit_current[point] < fit_current_min) {
            fit_current_min = fit_current[point];
        }
        if (fit_current[point] > fit_current_max) {
            fit_current_max = fit_current[point];
        }
        if (fit_current[point] < fit_target * MOTOR_DETECT_RESISTANCE_MIN_TRACK_RATIO) {
            fit_tracking_valid = false;
        }
        PRINTF("  R fit %lu/%lu: target=%.3f A I=%.4f A V=%.4f V V/I=%.4f ohm\r\n",
               (unsigned long)(point + 1U), (unsigned long)MOTOR_DETECT_RESISTANCE_FIT_POINTS, (double)fit_target,
               (double)fit_current[point], (double)fit_voltage[point],
               (double)(fit_voltage[point] / fit_current[point]));
        {
            float count = (float)s_detect_control.samples;
            float id_mean = s_detect_control.r_id_sum / count;
            float iq_mean = s_detect_control.r_iq_sum / count;
            float id_var = s_detect_control.r_id_square_sum / count - id_mean * id_mean;
            float iq_var = s_detect_control.r_iq_square_sum / count - iq_mean * iq_mean;
            /* 成功阶段保证样本数非零。打印在任务侧，ISR仅累加。 */
            PRINTF("  R diag: Id=%.4f Iq=%.4f std=%.4f/%.4f A Vab=%.4f/%.4f V Isum=%.4f A n=%lu\r\n",
                   (double)id_mean, (double)iq_mean, (double)sqrtf(fmaxf(id_var, 0.0f)),
                   (double)sqrtf(fmaxf(iq_var, 0.0f)), (double)(s_detect_control.r_v_alpha_sum / count),
                   (double)(s_detect_control.r_v_beta_sum / count),
                   (double)(s_detect_control.r_current_sum / count), (unsigned long)s_detect_control.samples);
        }
    }
    if (!fit_tracking_valid ||
        fit_current_max - fit_current_min <
            (fit_target_max - fit_target_min) * MOTOR_DETECT_RESISTANCE_MIN_SPAN_RATIO) {
        s_detect_result.error = MOTOR_DETECT_ERROR_RESISTANCE;
        PRINTF("Detect motor: resistance fit invalid current tracking span=%.4f A\r\n",
               (double)(fit_current_max - fit_current_min));
        goto detect_finish;
    }
    if (!Detect_FitResistance(fit_current, fit_voltage, MOTOR_DETECT_RESISTANCE_FIT_POINTS, &resistance,
                              &voltage_offset, &r_squared) ||
        resistance < MOTOR_DETECT_RESISTANCE_MIN_OHM || resistance > MOTOR_DETECT_RESISTANCE_MAX_OHM ||
        r_squared < MOTOR_DETECT_RESISTANCE_FIT_MIN_R2) {
        s_detect_result.error = MOTOR_DETECT_ERROR_RESISTANCE;
        PRINTF("Detect motor: resistance fit invalid R=%.6f R2=%.4f\r\n", (double)resistance,
               (double)r_squared);
        goto detect_finish;
    }
    {
        float signed_r, signed_offset, signed_r2;
        float di0 = fit_current[1] - fit_current[0];
        float di1 = fit_current[2] - fit_current[1];
        float slope0, slope1, mismatch;
        if (Detect_FitResistance(signed_current, signed_voltage, MOTOR_DETECT_RESISTANCE_FIT_POINTS,
                                 &signed_r, &signed_offset, &signed_r2)) {
            PRINTF("  R signed Vd/Id: R=%.6f ohm offset=%.4f V R2=%.4f\r\n",
                   (double)signed_r, (double)signed_offset, (double)signed_r2);
        }
        if (di0 <= 0.0f || di1 <= 0.0f) {
            s_detect_result.error = MOTOR_DETECT_ERROR_RESISTANCE;
            goto detect_finish;
        }
        slope0 = (fit_voltage[1] - fit_voltage[0]) / di0;
        slope1 = (fit_voltage[2] - fit_voltage[1]) / di1;
        mismatch = fabsf(slope1 - slope0) / fmaxf(fabsf(resistance), MOTOR_DETECT_RESISTANCE_MIN_OHM);
        PRINTF("  R slopes: %.6f / %.6f ohm mismatch=%.2f%% (limit %.1f%%)\r\n",
               (double)slope0, (double)slope1, (double)(mismatch * 100.0f),
               (double)(MOTOR_DETECT_RESISTANCE_MAX_SLOPE_RATIO * 100.0f));
        if (!isfinite(mismatch) || slope0 <= 0.0f || slope1 <= 0.0f ||
            mismatch > MOTOR_DETECT_RESISTANCE_MAX_SLOPE_RATIO) {
            PRINTF("  R rejected: inconsistent slopes; fitted R=%.6f is diagnostic only\r\n",
                   (double)resistance);
            s_detect_result.error = MOTOR_DETECT_ERROR_RESISTANCE;
            goto detect_finish;
        }
    }
    s_detect_result.resistance_ohm = resistance;
    s_detect_result.inverter_voltage_offset_v = voltage_offset;
    s_detect_result.resistance_fit_r2 = r_squared;
    s_detect_result.resistance_valid = true;
    s_detect_control.measured_resistance = resistance;
    PRINTF("Detect motor: R=%.6f ohm offset=%.4f V R2=%.4f\r\n", (double)resistance,
           (double)voltage_offset, (double)r_squared);

    PRINTF("Detect motor: measuring dq phase inductance\r\n");
    if (!Detect_RunStage(MOTOR_DETECT_STAGE_INDUCTANCE, current_a, MOTOR_DETECT_TIMEOUT_INDUCTANCE_MS)) {
        goto detect_finish;
    }
    PRINTF("Detect motor: L=%.3f uH\r\n", (double)(s_detect_result.inductance_h * 1000000.0f));
    s_detect_result.error = MOTOR_DETECT_ERROR_NONE;

detect_finish:
    Hal_Motor_PwmStop();
    Detect_ControlReset(&motor);
    s_detect_control.stage = MOTOR_DETECT_STAGE_IDLE;
    s_detect_active = retain_active && s_detect_result.error == MOTOR_DETECT_ERROR_NONE;
detect_done:
    if (result != NULL) {
        *result = s_detect_result;
    }
    return s_detect_result.resistance_valid && s_detect_result.inductance_valid &&
           s_detect_result.error == MOTOR_DETECT_ERROR_NONE;
}

bool DETECT_MotorRl(float current_a, motor_detect_result_t *result)
{
    return Detect_MotorRlInternal(current_a, result, false);
}

static bool Detect_MotorParametersInternal(float current_a, float electrical_rpm, float flux_current_a,
                                           bool scan, motor_detect_result_t *result)
{
    float kp_old;
    float ki_old;
    float voltage_limit_old;
    float flux_current;
    bool success;
    unsigned point;
    unsigned points = scan ? 3U : 1U;
    float max_erpm = electrical_rpm;

    if (scan && (max_erpm < 900.0f || max_erpm > 3000.0f ||
                 !Motor_Encoder_IsCompareEnabled() || !Motor_Encoder_IsSpeedValid())) {
        if (result != NULL) { memset(result, 0, sizeof(*result)); result->version = MOTOR_DETECT_RESULT_VERSION; result->error = MOTOR_DETECT_ERROR_INVALID_INPUT; }
        return false;
    }
    if (!isfinite(current_a) || !isfinite(electrical_rpm) || !isfinite(flux_current_a)) {
        if (result != NULL) {
            memset(result, 0, sizeof(*result));
            result->version = MOTOR_DETECT_RESULT_VERSION;
            result->error = MOTOR_DETECT_ERROR_INVALID_INPUT;
        }
        return false;
    }
    /* R/L 到磁链之间继续占用功率桥，失败不能解除其他检测的占用。 */
    success = Detect_MotorRlInternal(current_a, result, true);
    if (!success) return false;
    electrical_rpm = Detect_Clamp(fabsf(electrical_rpm), MOTOR_DETECT_MIN_ELECTRICAL_RPM,
                                  MOTOR_DETECT_MAX_ELECTRICAL_RPM);
    flux_current = Detect_Clamp(fabsf(flux_current_a), MOTOR_DETECT_MIN_CURRENT_A, MOTOR_DETECT_MAX_CURRENT_A);
    s_flux_scan = scan;
    for (point = 0U; point < points; ++point) {
        if (scan) electrical_rpm = max_erpm * (0.5f + 0.25f * (float)point);
        s_detect_result.test_electrical_rpm = electrical_rpm;
        s_detect_result.test_flux_current_a = flux_current;
        s_detect_control.electrical_rpm_target = electrical_rpm;
        s_detect_control.measured_resistance = s_detect_result.resistance_ohm;

        kp_old = motor.ol_current_kp;
        ki_old = motor.ol_current_ki;
        voltage_limit_old = motor.ol_vq_limit;
        motor.ol_current_kp = s_detect_result.inductance_h / MOTOR_DETECT_VESC_CURRENT_TC_S;
        motor.ol_current_ki = s_detect_result.resistance_ohm / MOTOR_DETECT_VESC_CURRENT_TC_S;
        motor.ol_vq_limit = MOTOR_DETECT_FLUX_VOLTAGE_LIMIT_V;

        PRINTF("Detect motor: accelerating to %.0f eRPM for coast BEMF (%.2f A)\r\n", (double)electrical_rpm,
               (double)flux_current);
        success = Detect_RunStage(MOTOR_DETECT_STAGE_FLUX, flux_current, MOTOR_DETECT_TIMEOUT_FLUX_MS);
        motor.ol_current_kp = kp_old;
        motor.ol_current_ki = ki_old;
        motor.ol_vq_limit = voltage_limit_old;
        if (scan) {
            Detect_PrintFluxTrace();
        }
        if (!success) {
            if (scan) {
                PRINTF("Flux scan rejected reference: encoder=%.1f eRPM coverage=%.1f%% pairs=%lu/%lu\r\n",
                       (double)(s_detect_control.flux_pair_time > 0.0f ?
                           s_detect_control.flux_encoder_omega_sum / s_detect_control.flux_pair_time *
                               (60.0f / (2.0f * MOTOR_DETECT_PI)) : 0.0f),
                       (double)(s_detect_control.flux_encoder_attempts ?
                           100.0f * s_detect_control.flux_encoder_count / s_detect_control.flux_encoder_attempts : 0.0f),
                       (unsigned long)s_detect_control.flux_encoder_count,
                       (unsigned long)s_detect_control.flux_encoder_attempts);
            }
            PRINTF("Detect motor: flux invalid duty still=%.4f now=%.4f max=%.4f coast=%.1f eRPM Bemf=%.4f V n=%lu\r\n",
                   (double)s_detect_control.flux_duty_still, (double)s_detect_control.flux_duty_now,
                   (double)s_detect_control.flux_duty_max, (double)s_detect_control.electrical_rpm_now,
                   (double)s_detect_control.resistance_voltage_avg, (unsigned long)s_detect_control.samples);
            goto parameters_done;
        }
        PRINTF("Detect motor: flux=%.4f mWb, coast Bemf=%.4f V at %.1f eRPM\r\n",
               (double)(s_detect_result.flux_linkage_wb * 1000.0f),
               (double)s_detect_control.resistance_voltage_avg, (double)s_detect_control.electrical_rpm_now);
        if (scan) {
            float coverage = s_detect_control.flux_encoder_attempts ?
                (float)s_detect_control.flux_encoder_count / s_detect_control.flux_encoder_attempts : 0.0f;
            float enc_omega = s_detect_control.flux_pair_time > 0.0f ?
                s_detect_control.flux_encoder_omega_sum / s_detect_control.flux_pair_time : 0.0f;
            float enc_v = s_detect_control.flux_pair_time > 0.0f ?
                s_detect_control.flux_encoder_voltage_sum / s_detect_control.flux_pair_time : 0.0f;
            float enc_erpm = enc_omega * (60.0f / (2.0f * MOTOR_DETECT_PI));
            s_detect_result.flux_scan_erpm[point] = s_detect_control.electrical_rpm_now;
            s_detect_result.flux_scan_voltage[point] = s_detect_control.resistance_voltage_avg;
            s_detect_result.flux_scan_encoder_erpm[point] = enc_erpm;
            s_detect_result.flux_scan_encoder_voltage[point] = enc_v;
            s_detect_result.flux_scan_coverage[point] = coverage;
            PRINTF("Flux scan %u/3: BEMF=%.1f encoder=%.1f eRPM E=%.4fV coverage=%.1f%%\r\n",
                   point + 1U, (double)s_detect_control.electrical_rpm_now, (double)enc_erpm,
                   (double)enc_v, (double)(coverage * 100.0f));
            if (coverage < 0.8f || s_detect_control.flux_encoder_count < 30U ||
                fabsf(enc_erpm - s_detect_control.electrical_rpm_now) >
                    0.2f * s_detect_control.electrical_rpm_now) {
                s_detect_result.error = MOTOR_DETECT_ERROR_FLUX;
                goto parameters_done;
            }
        }
    }
    if (scan) {
        float omega[3];
        for (point = 0U; point < 3U; ++point)
            omega[point] = s_detect_result.flux_scan_encoder_erpm[point] * (2.0f * MOTOR_DETECT_PI / 60.0f);
        s_detect_result.flux_scan_valid = fabsf(omega[2] - omega[0]) >= 50.0f &&
            Detect_FitResistance(omega, s_detect_result.flux_scan_encoder_voltage, 3U,
                &s_detect_result.flux_scan_fit_wb, &s_detect_result.flux_scan_offset_v,
                &s_detect_result.flux_scan_r2) &&
            s_detect_result.flux_scan_fit_wb >= MOTOR_DETECT_FLUX_MIN_WB &&
            s_detect_result.flux_scan_fit_wb <= MOTOR_DETECT_FLUX_MAX_WB && s_detect_result.flux_scan_r2 >= 0.95f;
        if (!s_detect_result.flux_scan_valid) {
            s_detect_result.error = MOTOR_DETECT_ERROR_FLUX;
            goto parameters_done;
        }
    }
    s_detect_result.error = MOTOR_DETECT_ERROR_NONE;

parameters_done:
    s_flux_scan = false;
    s_detect_active = false;
    if (result != NULL) {
        *result = s_detect_result;
    }
    return s_detect_result.resistance_valid && s_detect_result.inductance_valid && s_detect_result.flux_valid &&
           s_detect_result.error == MOTOR_DETECT_ERROR_NONE;
}

bool DETECT_MotorParameters(float current_a, float electrical_rpm, float flux_current_a,
                            motor_detect_result_t *result)
{
    return Detect_MotorParametersInternal(current_a, electrical_rpm, flux_current_a, false, result);
}

bool DETECT_MotorFluxScan(float current_a, float max_erpm, float flux_current_a, motor_detect_result_t *result)
{
    return Detect_MotorParametersInternal(current_a, max_erpm, flux_current_a, true, result);
}

void DETECT_GetResult(motor_detect_result_t *result)
{
    if (result != NULL) {
        uint32_t primask = Motor_Crit_Enter();
        *result = s_detect_result;
        Motor_Crit_Exit(primask);
    }
}

void DETECT_Abort(void)
{
    s_detect_abort_requested = true;
}

void DETECT_ClearAbortRequest(void)
{
    if (!s_detect_active) {
        s_detect_abort_requested = false;
    }
}

const char *DETECT_GetErrorName(motor_detect_error_t error)
{
    switch (error) {
    case MOTOR_DETECT_ERROR_NONE: return "NONE";
    case MOTOR_DETECT_ERROR_BUSY: return "BUSY";
    case MOTOR_DETECT_ERROR_MOTOR_RUNNING: return "MOTOR_RUNNING";
    case MOTOR_DETECT_ERROR_FAULT: return "FAULT";
    case MOTOR_DETECT_ERROR_TIMEOUT: return "TIMEOUT";
    case MOTOR_DETECT_ERROR_CURRENT: return "CURRENT_TOO_HIGH";
    case MOTOR_DETECT_ERROR_RESISTANCE: return "RESISTANCE_INVALID";
    case MOTOR_DETECT_ERROR_INDUCTANCE: return "INDUCTANCE_INVALID";
    case MOTOR_DETECT_ERROR_FLUX: return "FLUX_INVALID";
    case MOTOR_DETECT_ERROR_USER_ABORT: return "USER_ABORT";
    case MOTOR_DETECT_ERROR_INVALID_INPUT: return "INVALID_INPUT";
    default: return "UNKNOWN";
    }
}

/**
 * @description: 开环转一圈，标定霍尔状态与电角度对应表。
 * @return {bool} true 表示 6 个有效扇区标定成功
 * @author: lkc
 */
bool DETECT_Hall(void)
{
    uint8_t hall_tab[MOTOR_HALL_TABLE_SIZE];
    bool res;
    int i;

    /* I/F 扫角电流；PI 限流，避免开环电压过流 */
    Motor_Hall_Detect(1.2f, hall_tab, &res);

    if (res) {
        INFO("DETECT HALL Success.\r\n");
        for (i = 0; i < (int)MOTOR_HALL_TABLE_SIZE; i++) {
            PRINTF("%d ", (int)motor.hall_table[i]);
        }
        PRINTF("\r\n");
        for (i = 0; i < 6; i++) {
            PRINTF("%d ", motor.hallABC[i]);
        }
        PRINTF("\r\n");
    } else {
        PRINTF("DETECT HALL Failed.\r\n");
    }

    return res;
}

/**
 * @description: 上电偏移校准：已有 Flash 电流零点则跳过，否则检测并写入 Flash。
 * @detail: 校准期间 ISR 仍采样+软件保护，但不跑 FOC；失败时 fault 保持锁存。
 * 开机偏移：Flash 已有有效电流零点则直接使用；否则校准约 5 秒并自动存 Flash。
 * 会撤销运行命令，结束后保持停机，需要转动时由 olspeed/olstart 明确启动。
 * @return {void} 无
 * @author: lkc
 */
void Detect_Init(void)
{
    if (Motor_Detect_OffsetIsStored(&motor)) {
        PRINTF("Detect_Init: skip, using stored offset\r\n");
        PRINTF("Current Offset [%f] [%f] [%f]\r\n", motor.currentOffset[0], motor.currentOffset[1],
               motor.currentOffset[2]);
        PRINTF("Volt Offset [%f] [%f] [%f]\r\n", motor.voltOffset[0], motor.voltOffset[1], motor.voltOffset[2]);
        PRINTF("Undriven Offset [%f] [%f] [%f]\r\n", motor.voltundrivenOffset[0], motor.voltundrivenOffset[1],
               motor.voltundrivenOffset[2]);
        return;
    }

    if (DETECT_Offset(true) == 0U) {
        PRINTF("Detect_Init: offset aborted\r\n");
    }
}

#ifdef __cplusplus
}
#endif
