/*
 * @Description: 平台无关的配置与校准快照；保持旧控制字段兼容。
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_settings.h"
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <float.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* 当前线格式要求 IEEE754 单精度；不支持的平台应在编译时失败。 */
typedef char
    motor_float_size_check[(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128) ? 1 : -1];
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static bool settings_valid(const motor_settings_t *s);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 校验完整 settings 快照：所有配置为有限值且取值合法。
 * @detail: 任一字段非法即整体拒绝；含校准数组范围与霍尔表编码合法性检查。
 * @param {const motor_settings_t *} s 待校验快照，不可为空
 * @return {bool} 合法返回 true
 * @author: lkc
 */
static bool settings_valid(const motor_settings_t *s)
{
    unsigned i;
    const motor_config_t *c = &s->config;
    if (!isfinite(c->ol_max_rpm)) return false;
    if (!isfinite(c->ol_v_boost)) return false;
    if (!isfinite(c->ol_kv_f)) return false;
    if (!isfinite(c->ol_ramp_rpm_s)) return false;
    if (!isfinite(c->ol_vq_limit)) return false;
    if (!isfinite(c->ol_foc_freq)) return false;
    if (!isfinite(c->ol_pole_pairs)) return false;
    if (!isfinite(c->ol_min_rpm)) return false;
    if (!isfinite(c->ol_current_kp)) return false;
    if (!isfinite(c->ol_current_ki)) return false;
    if (!isfinite(c->ol_start_phase_boost)) return false;
    if (!isfinite(c->ol_align_current)) return false;
    if (!isfinite(c->ol_align_time)) return false;
    if (!isfinite(c->ol_align_angle)) return false;
    if (!isfinite(c->hfi_start_voltage)) return false;
    if (!isfinite(c->hfi_start_min_confidence)) return false;
    if (!isfinite(c->speed_kp)) return false;
    if (!isfinite(c->speed_ki)) return false;
    if (!isfinite(c->speed_iq_limit)) return false;
    if (!isfinite(c->speed_loop_hz)) return false;
    if (!isfinite(c->sensorless_motor_r)) return false;
    if (!isfinite(c->sensorless_motor_l)) return false;
    if (!isfinite(c->sensorless_flux_linkage)) return false;
    if (!isfinite(c->sensorless_observer_gain)) return false;
    if (!isfinite(c->sensorless_pll_kp)) return false;
    if (!isfinite(c->sensorless_pll_ki)) return false;
    if (!isfinite(c->sensorless_transition_rpm)) return false;
    if (!isfinite(c->sensorless_fallback_rpm)) return false;
    if (!isfinite(c->sensorless_openloop_max_rpm)) return false;
    if (!isfinite(c->sensorless_lock_time)) return false;
    if (!isfinite(c->sensorless_loss_time)) return false;
    if (!isfinite(c->sensorless_blend_time)) return false;
    if (!isfinite(c->sensorless_phase_tolerance)) return false;
    if (!isfinite(c->sensorless_phase_offset)) return false;
    if (!isfinite(c->sensorless_speed_filter)) return false;
    if (c->ol_v_boost < 0.0f) return false;
    if (c->ol_kv_f < 0.0f) return false;
    if (c->ol_ramp_rpm_s < 0.0f) return false;
    if (c->ol_vq_limit < 0.0f) return false;
    if (c->ol_min_rpm < 0.0f) return false;
    if (c->ol_current_kp < 0.0f) return false;
    if (c->ol_current_ki < 0.0f) return false;
    if (c->ol_start_phase_boost < 0.0f) return false;
    if (c->ol_align_current < 0.0f) return false;
    if (c->ol_align_time < 0.0f) return false;
    if (c->hfi_start_voltage < 0.0f) return false;
    if (c->hfi_start_min_confidence < 0.0f) return false;
    if (c->speed_kp < 0.0f) return false;
    if (c->speed_ki < 0.0f) return false;
    if (c->speed_iq_limit < 0.0f) return false;
    if (c->sensorless_motor_r < 0.0f) return false;
    if (c->sensorless_motor_l < 0.0f) return false;
    if (c->sensorless_flux_linkage < 0.0f) return false;
    if (c->sensorless_observer_gain < 0.0f) return false;
    if (c->sensorless_pll_kp < 0.0f) return false;
    if (c->sensorless_pll_ki < 0.0f) return false;
    if (c->sensorless_transition_rpm < 0.0f) return false;
    if (c->sensorless_fallback_rpm < 0.0f) return false;
    if (c->sensorless_openloop_max_rpm < 0.0f) return false;
    if (c->sensorless_lock_time < 0.0f) return false;
    if (c->sensorless_loss_time < 0.0f) return false;
    if (c->sensorless_blend_time < 0.0f) return false;
    if (c->sensorless_phase_tolerance < 0.0f) return false;
    if (c->sensorless_speed_filter < 0.0f) return false;
    if (c->ol_pole_pairs < 1.0f || c->ol_pole_pairs > 255.0f || floorf(c->ol_pole_pairs) != c->ol_pole_pairs ||
        c->ol_max_rpm <= 0.0f || c->ol_foc_freq < 1.0f || c->speed_loop_hz < 1.0f ||
        c->speed_loop_hz > c->ol_foc_freq || c->ol_min_rpm > c->ol_max_rpm ||
        c->sensorless_fallback_rpm > c->sensorless_transition_rpm || c->sensorless_openloop_max_rpm > c->ol_max_rpm ||
        c->sensorless_speed_filter > 1.0f || c->sensorless_phase_tolerance > 1.5f ||
        c->hfi_start_min_confidence > 1.0f || c->hall_extra_samples < 0 || c->hall_extra_samples > 32 ||
        (c->hfi_start_enable && c->hfi_start_cycles == 0U) ||
        (c->sensorless_enable && c->sensorless_flux_linkage < 1.0e-6f)) {
        return false;
    }
    for (i = 0; i < 3; ++i) {
        if (!isfinite(s->calibration.current_offset[i]) || s->calibration.current_offset[i] < 0.0f ||
            !isfinite(s->calibration.voltage_offset[i]) || !isfinite(s->calibration.undriven_offset[i])) {
            return false;
        }
    }
    for (i = 0; i < MOTOR_HALL_TABLE_SIZE; ++i) {
        if (s->calibration.hall_table[i] > 200U && s->calibration.hall_table[i] != 255U) return false;
    }
    if (!isfinite(s->calibration.encoder_offset_rad) || s->calibration.encoder_offset_rad < 0.0f ||
        s->calibration.encoder_offset_rad >= 6.28318530718f) {
        return false;
    }
    return true;
}

/**
 * @description: 从旧电机对象提取配置/校准；不提取命令与实时状态。
 * @detail: 先在临时快照上校验；失败不修改输出。
 * @param {const MOTOR_PARAM_S *} motor 已初始化的旧电机对象，不可为空
 * @param {motor_settings_t *} settings 输出快照，不可为空；失败不修改
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_Settings_Capture(const MOTOR_PARAM_S *motor, motor_settings_t *settings)
{
    motor_settings_t candidate;
    if (motor == NULL || settings == NULL) return false;
    memset(&candidate, 0, sizeof(candidate));
    candidate.config.ol_max_rpm = motor->ol_max_rpm;
    candidate.config.ol_v_boost = motor->ol_v_boost;
    candidate.config.ol_kv_f = motor->ol_kv_f;
    candidate.config.ol_ramp_rpm_s = motor->ol_ramp_rpm_s;
    candidate.config.ol_vq_limit = motor->ol_vq_limit;
    candidate.config.ol_foc_freq = motor->ol_foc_freq;
    candidate.config.ol_pole_pairs = motor->ol_pole_pairs;
    candidate.config.ol_min_rpm = motor->ol_min_rpm;
    candidate.config.ol_current_kp = motor->ol_current_kp;
    candidate.config.ol_current_ki = motor->ol_current_ki;
    candidate.config.ol_start_phase_boost = motor->ol_start_phase_boost;
    candidate.config.ol_align_enable = motor->ol_align_enable;
    candidate.config.ol_align_current = motor->ol_align_current;
    candidate.config.ol_align_time = motor->ol_align_time;
    candidate.config.ol_align_angle = motor->ol_align_angle;
    candidate.config.hfi_start_enable = motor->hfi_start_enable;
    candidate.config.hfi_start_voltage = motor->hfi_start_voltage;
    candidate.config.hfi_start_cycles = motor->hfi_start_cycles;
    candidate.config.hfi_start_min_confidence = motor->hfi_start_min_confidence;
    candidate.config.speed_enable = motor->speed_enable;
    candidate.config.speed_allow_braking = motor->speed_allow_braking;
    candidate.config.speed_kp = motor->speed_kp;
    candidate.config.speed_ki = motor->speed_ki;
    candidate.config.speed_iq_limit = motor->speed_iq_limit;
    candidate.config.speed_loop_hz = motor->speed_loop_hz;
    candidate.config.sensorless_enable = motor->sensorless_enable;
    candidate.config.sensorless_takeover = motor->sensorless_takeover;
    candidate.config.sensorless_motor_r = motor->sensorless_motor_r;
    candidate.config.sensorless_motor_l = motor->sensorless_motor_l;
    candidate.config.sensorless_flux_linkage = motor->sensorless_flux_linkage;
    candidate.config.sensorless_observer_gain = motor->sensorless_observer_gain;
    candidate.config.sensorless_pll_kp = motor->sensorless_pll_kp;
    candidate.config.sensorless_pll_ki = motor->sensorless_pll_ki;
    candidate.config.sensorless_transition_rpm = motor->sensorless_transition_rpm;
    candidate.config.sensorless_fallback_rpm = motor->sensorless_fallback_rpm;
    candidate.config.sensorless_openloop_max_rpm = motor->sensorless_openloop_max_rpm;
    candidate.config.sensorless_lock_time = motor->sensorless_lock_time;
    candidate.config.sensorless_loss_time = motor->sensorless_loss_time;
    candidate.config.sensorless_blend_time = motor->sensorless_blend_time;
    candidate.config.sensorless_phase_tolerance = motor->sensorless_phase_tolerance;
    candidate.config.sensorless_phase_offset = motor->sensorless_phase_offset;
    candidate.config.sensorless_speed_filter = motor->sensorless_speed_filter;
    candidate.config.hall_extra_samples = motor->hall_extra_samples;
    memcpy(candidate.calibration.current_offset, motor->currentOffset, sizeof(candidate.calibration.current_offset));
    memcpy(candidate.calibration.voltage_offset, motor->voltOffset, sizeof(candidate.calibration.voltage_offset));
    memcpy(candidate.calibration.undriven_offset, motor->voltundrivenOffset,
           sizeof(candidate.calibration.undriven_offset));
    memcpy(candidate.calibration.hall_table, motor->hall_table, sizeof(candidate.calibration.hall_table));
    candidate.calibration.encoder_offset_rad = motor->encoder_offset_rad;
    candidate.calibration.encoder_inverted = motor->encoder_inverted;
    candidate.calibration.encoder_calibrated = motor->encoder_calibrated;
    if (!settings_valid(&candidate)) return false;
    *settings = candidate;
    return true;
}

/**
 * @description: 将 config 全部字段写回电机对象（仅配置域）。
 * @param {MOTOR_PARAM_S *} motor 目标电机对象，不可为空
 * @param {const motor_config_t *} config 来源配置，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void settings_write_config(MOTOR_PARAM_S *motor, const motor_config_t *config)
{
    motor->ol_max_rpm = config->ol_max_rpm;
    motor->ol_v_boost = config->ol_v_boost;
    motor->ol_kv_f = config->ol_kv_f;
    motor->ol_ramp_rpm_s = config->ol_ramp_rpm_s;
    motor->ol_vq_limit = config->ol_vq_limit;
    motor->ol_foc_freq = config->ol_foc_freq;
    motor->ol_pole_pairs = config->ol_pole_pairs;
    motor->ol_min_rpm = config->ol_min_rpm;
    motor->ol_current_kp = config->ol_current_kp;
    motor->ol_current_ki = config->ol_current_ki;
    motor->ol_start_phase_boost = config->ol_start_phase_boost;
    motor->ol_align_enable = config->ol_align_enable;
    motor->ol_align_current = config->ol_align_current;
    motor->ol_align_time = config->ol_align_time;
    motor->ol_align_angle = config->ol_align_angle;
    motor->hfi_start_enable = config->hfi_start_enable;
    motor->hfi_start_voltage = config->hfi_start_voltage;
    motor->hfi_start_cycles = config->hfi_start_cycles;
    motor->hfi_start_min_confidence = config->hfi_start_min_confidence;
    motor->speed_enable = config->speed_enable;
    motor->speed_allow_braking = config->speed_allow_braking;
    motor->speed_kp = config->speed_kp;
    motor->speed_ki = config->speed_ki;
    motor->speed_iq_limit = config->speed_iq_limit;
    motor->speed_loop_hz = config->speed_loop_hz;
    motor->sensorless_enable = config->sensorless_enable;
    motor->sensorless_takeover = config->sensorless_takeover;
    motor->sensorless_motor_r = config->sensorless_motor_r;
    motor->sensorless_motor_l = config->sensorless_motor_l;
    motor->sensorless_flux_linkage = config->sensorless_flux_linkage;
    motor->sensorless_observer_gain = config->sensorless_observer_gain;
    motor->sensorless_pll_kp = config->sensorless_pll_kp;
    motor->sensorless_pll_ki = config->sensorless_pll_ki;
    motor->sensorless_transition_rpm = config->sensorless_transition_rpm;
    motor->sensorless_fallback_rpm = config->sensorless_fallback_rpm;
    motor->sensorless_openloop_max_rpm = config->sensorless_openloop_max_rpm;
    motor->sensorless_lock_time = config->sensorless_lock_time;
    motor->sensorless_loss_time = config->sensorless_loss_time;
    motor->sensorless_blend_time = config->sensorless_blend_time;
    motor->sensorless_phase_tolerance = config->sensorless_phase_tolerance;
    motor->sensorless_phase_offset = config->sensorless_phase_offset;
    motor->sensorless_speed_filter = config->sensorless_speed_filter;
    motor->hall_extra_samples = config->hall_extra_samples;
}

/**
 * @description: 停机时应用合法配置/校准；失败不修改对象。
 * @detail: 电机使能中或快照非法时拒绝；不写命令与实时状态。
 * @param {MOTOR_PARAM_S *} motor 已停机的目标电机对象，不可为空
 * @param {const motor_settings_t *} settings 已校验的配置与校准，不可为空
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_Settings_Apply(MOTOR_PARAM_S *motor, const motor_settings_t *settings)
{
    if (motor == NULL || settings == NULL || motor->ol_enable || !settings_valid(settings)) return false;
    settings_write_config(motor, &settings->config);
    memcpy(motor->currentOffset, settings->calibration.current_offset, sizeof(motor->currentOffset));
    memcpy(motor->voltOffset, settings->calibration.voltage_offset, sizeof(motor->voltOffset));
    memcpy(motor->voltundrivenOffset, settings->calibration.undriven_offset, sizeof(motor->voltundrivenOffset));
    memcpy(motor->hall_table, settings->calibration.hall_table, sizeof(motor->hall_table));
    motor->encoder_offset_rad = settings->calibration.encoder_offset_rad;
    motor->encoder_inverted = settings->calibration.encoder_inverted;
    motor->encoder_calibrated = settings->calibration.encoder_calibrated;
    return true;
}

/**
 * @description: 判断配置变更是否必须停机后才能应用。
 * @detail: 停机字段含极对数、控制/速度环频率、对齐/HFI、霍尔滤波次数与无感模型参数。
 * @param {const motor_config_t *} from 当前配置，不可为空
 * @param {const motor_config_t *} to 目标配置，不可为空
 * @return {bool} true=必须停机；空指针视为需要停机
 * @author: lkc
 */
bool Motor_Settings_ConfigRequiresStop(const motor_config_t *from, const motor_config_t *to)
{
    if (from == NULL || to == NULL) return true;

    if (from->ol_pole_pairs != to->ol_pole_pairs || from->ol_foc_freq != to->ol_foc_freq ||
        from->speed_loop_hz != to->speed_loop_hz || from->hall_extra_samples != to->hall_extra_samples) {
        return true;
    }
    if (from->ol_align_enable != to->ol_align_enable || from->ol_align_current != to->ol_align_current ||
        from->ol_align_time != to->ol_align_time || from->ol_align_angle != to->ol_align_angle) {
        return true;
    }
    if (from->hfi_start_enable != to->hfi_start_enable || from->hfi_start_voltage != to->hfi_start_voltage ||
        from->hfi_start_cycles != to->hfi_start_cycles ||
        from->hfi_start_min_confidence != to->hfi_start_min_confidence) {
        return true;
    }
    if (from->sensorless_motor_r != to->sensorless_motor_r || from->sensorless_motor_l != to->sensorless_motor_l ||
        from->sensorless_flux_linkage != to->sensorless_flux_linkage ||
        from->sensorless_observer_gain != to->sensorless_observer_gain ||
        from->sensorless_pll_kp != to->sensorless_pll_kp || from->sensorless_pll_ki != to->sensorless_pll_ki ||
        from->sensorless_transition_rpm != to->sensorless_transition_rpm ||
        from->sensorless_fallback_rpm != to->sensorless_fallback_rpm ||
        from->sensorless_lock_time != to->sensorless_lock_time ||
        from->sensorless_loss_time != to->sensorless_loss_time ||
        from->sensorless_phase_offset != to->sensorless_phase_offset ||
        from->sensorless_speed_filter != to->sensorless_speed_filter) {
        return true;
    }
    return false;
}

/**
 * @description: 应用配置（不改校准、不改命令目标）。
 * @detail: 先拼完整 settings 校验；运行中且改动停机字段时拒绝。
 * @param {MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {const motor_config_t *} config 目标配置，不可为空
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_Settings_ApplyConfig(MOTOR_PARAM_S *motor, const motor_config_t *config)
{
    motor_settings_t candidate;

    if (motor == NULL || config == NULL) return false;
    if (!Motor_Settings_Capture(motor, &candidate)) return false;
    if (motor->ol_enable && Motor_Settings_ConfigRequiresStop(&candidate.config, config)) return false;
    candidate.config = *config;
    if (!settings_valid(&candidate)) return false;
    settings_write_config(motor, config);
    return true;
}

/**
 * @description: 按小端序写入 4 字节并前进指针。
 * @param {uint8_t **} p 读写位置指针，不可为空
 * @param {uint32_t} value 待写入值
 * @return {void} 无
 * @author: lkc
 */
static void put_word(uint8_t **p, uint32_t value)
{
    unsigned i;
    for (i = 0; i < 4; ++i) {
        *(*p)++ = (uint8_t)(value >> (8U * i));
    }
}

/**
 * @description: 按小端序读取 4 字节并前进指针。
 * @param {const uint8_t **} p 读写位置指针，不可为空
 * @return {uint32_t} 读取值
 * @author: lkc
 */
static uint32_t get_word(const uint8_t **p)
{
    uint32_t value = 0;
    unsigned i;
    for (i = 0; i < 4; ++i) {
        value |= (uint32_t)*(*p)++ << (8U * i);
    }
    return value;
}

/**
 * @description: 以 IEEE754 位模式写入 float（小端 4 字节）。
 * @param {uint8_t **} p 读写位置指针，不可为空
 * @param {float} value 待写入浮点值
 * @return {void} 无
 * @author: lkc
 */
static void put_float(uint8_t **p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, 4);
    put_word(p, bits);
}

/**
 * @description: 按小端序读取 4 字节并解释为 IEEE754 float。
 * @param {const uint8_t **} p 读写位置指针，不可为空
 * @return {float} 读取的浮点值
 * @author: lkc
 */
static float get_float(const uint8_t **p)
{
    uint32_t bits = get_word(p);
    float value;
    memcpy(&value, &bits, 4);
    return value;
}

/**
 * @description: 编码为固定小端格式；size 必须匹配，非法输入返回 false 且不写输出。
 * @detail: 布尔按 32 位字编码、浮点按 IEEE754；霍尔表原样拷贝尾部。
 * @param {const motor_settings_t *} settings 输入配置/校准快照
 * @param {uint8_t *} data 输出固定编码缓冲区，不可为空
 * @param {uint32_t} size 缓冲区长度 (byte)，须等于 MOTOR_SETTINGS_WIRE_SIZE
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_Settings_Encode(const motor_settings_t *settings, uint8_t *data, uint32_t size)
{
    uint8_t *p = data;
    unsigned i;
    if (settings == NULL || data == NULL || size != MOTOR_SETTINGS_WIRE_SIZE || !settings_valid(settings)) return false;
    put_float(&p, settings->config.ol_max_rpm);
    put_float(&p, settings->config.ol_v_boost);
    put_float(&p, settings->config.ol_kv_f);
    put_float(&p, settings->config.ol_ramp_rpm_s);
    put_float(&p, settings->config.ol_vq_limit);
    put_float(&p, settings->config.ol_foc_freq);
    put_float(&p, settings->config.ol_pole_pairs);
    put_float(&p, settings->config.ol_min_rpm);
    put_float(&p, settings->config.ol_current_kp);
    put_float(&p, settings->config.ol_current_ki);
    put_float(&p, settings->config.ol_start_phase_boost);
    put_word(&p, settings->config.ol_align_enable);
    put_float(&p, settings->config.ol_align_current);
    put_float(&p, settings->config.ol_align_time);
    put_float(&p, settings->config.ol_align_angle);
    put_word(&p, settings->config.hfi_start_enable);
    put_float(&p, settings->config.hfi_start_voltage);
    put_word(&p, settings->config.hfi_start_cycles);
    put_float(&p, settings->config.hfi_start_min_confidence);
    put_word(&p, settings->config.speed_enable);
    put_word(&p, settings->config.speed_allow_braking);
    put_float(&p, settings->config.speed_kp);
    put_float(&p, settings->config.speed_ki);
    put_float(&p, settings->config.speed_iq_limit);
    put_float(&p, settings->config.speed_loop_hz);
    put_word(&p, settings->config.sensorless_enable);
    put_word(&p, settings->config.sensorless_takeover);
    put_float(&p, settings->config.sensorless_motor_r);
    put_float(&p, settings->config.sensorless_motor_l);
    put_float(&p, settings->config.sensorless_flux_linkage);
    put_float(&p, settings->config.sensorless_observer_gain);
    put_float(&p, settings->config.sensorless_pll_kp);
    put_float(&p, settings->config.sensorless_pll_ki);
    put_float(&p, settings->config.sensorless_transition_rpm);
    put_float(&p, settings->config.sensorless_fallback_rpm);
    put_float(&p, settings->config.sensorless_openloop_max_rpm);
    put_float(&p, settings->config.sensorless_lock_time);
    put_float(&p, settings->config.sensorless_loss_time);
    put_float(&p, settings->config.sensorless_blend_time);
    put_float(&p, settings->config.sensorless_phase_tolerance);
    put_float(&p, settings->config.sensorless_phase_offset);
    put_float(&p, settings->config.sensorless_speed_filter);
    put_word(&p, settings->config.hall_extra_samples);
    for (i = 0; i < 3; ++i) {
        put_float(&p, settings->calibration.current_offset[i]);
    }
    for (i = 0; i < 3; ++i) {
        put_float(&p, settings->calibration.voltage_offset[i]);
    }
    for (i = 0; i < 3; ++i) {
        put_float(&p, settings->calibration.undriven_offset[i]);
    }
    memcpy(p, settings->calibration.hall_table, MOTOR_HALL_TABLE_SIZE);
    p += MOTOR_HALL_TABLE_SIZE;
    put_float(&p, settings->calibration.encoder_offset_rad);
    put_word(&p, settings->calibration.encoder_inverted);
    put_word(&p, settings->calibration.encoder_calibrated);
    return true;
}

/**
 * @description: 解码并校验；失败不修改输出，不依赖结构体布局。
 * @detail: 布尔字按 0/1、cycles/滤波次数按上限校验；结尾整体走 settings_valid。
 * @param {const uint8_t *} data 输入固定编码缓冲区，不可为空
 * @param {uint32_t} size 缓冲区长度 (byte)，须等于 MOTOR_SETTINGS_WIRE_SIZE
 * @param {motor_settings_t *} settings 输出快照，失败不修改
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_Settings_Decode(const uint8_t *data, uint32_t size, motor_settings_t *settings)
{
    const uint8_t *p = data;
    motor_settings_t candidate;
    uint32_t word;
    unsigned i;
    if (data == NULL || settings == NULL || (size != MOTOR_SETTINGS_WIRE_SIZE && size != MOTOR_SETTINGS_WIRE_SIZE_V2)) return false;
    memset(&candidate, 0, sizeof(candidate));
    candidate.config.ol_max_rpm = get_float(&p);
    candidate.config.ol_v_boost = get_float(&p);
    candidate.config.ol_kv_f = get_float(&p);
    candidate.config.ol_ramp_rpm_s = get_float(&p);
    candidate.config.ol_vq_limit = get_float(&p);
    candidate.config.ol_foc_freq = get_float(&p);
    candidate.config.ol_pole_pairs = get_float(&p);
    candidate.config.ol_min_rpm = get_float(&p);
    candidate.config.ol_current_kp = get_float(&p);
    candidate.config.ol_current_ki = get_float(&p);
    candidate.config.ol_start_phase_boost = get_float(&p);
    word = get_word(&p);
    if (word > 1U) return false;
    candidate.config.ol_align_enable = (bool)word;
    candidate.config.ol_align_current = get_float(&p);
    candidate.config.ol_align_time = get_float(&p);
    candidate.config.ol_align_angle = get_float(&p);
    word = get_word(&p);
    if (word > 1U) return false;
    candidate.config.hfi_start_enable = (bool)word;
    candidate.config.hfi_start_voltage = get_float(&p);
    word = get_word(&p);
    if (word > 65535U) return false;
    candidate.config.hfi_start_cycles = (uint16_t)word;
    candidate.config.hfi_start_min_confidence = get_float(&p);
    word = get_word(&p);
    if (word > 1U) return false;
    candidate.config.speed_enable = (bool)word;
    word = get_word(&p);
    if (word > 1U) return false;
    candidate.config.speed_allow_braking = (bool)word;
    candidate.config.speed_kp = get_float(&p);
    candidate.config.speed_ki = get_float(&p);
    candidate.config.speed_iq_limit = get_float(&p);
    candidate.config.speed_loop_hz = get_float(&p);
    word = get_word(&p);
    if (word > 1U) return false;
    candidate.config.sensorless_enable = (bool)word;
    word = get_word(&p);
    if (word > 1U) return false;
    candidate.config.sensorless_takeover = (bool)word;
    candidate.config.sensorless_motor_r = get_float(&p);
    candidate.config.sensorless_motor_l = get_float(&p);
    candidate.config.sensorless_flux_linkage = get_float(&p);
    candidate.config.sensorless_observer_gain = get_float(&p);
    candidate.config.sensorless_pll_kp = get_float(&p);
    candidate.config.sensorless_pll_ki = get_float(&p);
    candidate.config.sensorless_transition_rpm = get_float(&p);
    candidate.config.sensorless_fallback_rpm = get_float(&p);
    candidate.config.sensorless_openloop_max_rpm = get_float(&p);
    candidate.config.sensorless_lock_time = get_float(&p);
    candidate.config.sensorless_loss_time = get_float(&p);
    candidate.config.sensorless_blend_time = get_float(&p);
    candidate.config.sensorless_phase_tolerance = get_float(&p);
    candidate.config.sensorless_phase_offset = get_float(&p);
    candidate.config.sensorless_speed_filter = get_float(&p);
    word = get_word(&p);
    if (word > 32U) return false;
    candidate.config.hall_extra_samples = (int)word;
    for (i = 0; i < 3; ++i) {
        candidate.calibration.current_offset[i] = get_float(&p);
    }
    for (i = 0; i < 3; ++i) {
        candidate.calibration.voltage_offset[i] = get_float(&p);
    }
    for (i = 0; i < 3; ++i) {
        candidate.calibration.undriven_offset[i] = get_float(&p);
    }
    memcpy(candidate.calibration.hall_table, p, MOTOR_HALL_TABLE_SIZE);
    p += MOTOR_HALL_TABLE_SIZE;
    if (size == MOTOR_SETTINGS_WIRE_SIZE) {
        candidate.calibration.encoder_offset_rad = get_float(&p);
        word = get_word(&p);
        if (word > 1U) return false;
        candidate.calibration.encoder_inverted = (bool)word;
        word = get_word(&p);
        if (word > 1U) return false;
        candidate.calibration.encoder_calibrated = (bool)word;
    }
    if (!settings_valid(&candidate)) return false;
    *settings = candidate;
    return true;
}

#ifdef __cplusplus
}
#endif
