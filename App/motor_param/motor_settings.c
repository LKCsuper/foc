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

/* 单字段校验：FLOAT 须有限且按需非负；其余类型的范围由解码与跨字段约束负责。 */
static bool settings_field_valid_FLOAT(float value, bool nonneg)
{
    return isfinite(value) && !(nonneg && value < 0.0f);
}
static bool settings_field_valid_BOOL(bool value, bool nonneg)
{
    (void)value;
    (void)nonneg;
    return true;
}
static bool settings_field_valid_U16(uint16_t value, bool nonneg)
{
    (void)value;
    (void)nonneg;
    return true;
}
static bool settings_field_valid_INT(int value, bool nonneg)
{
    (void)value;
    (void)nonneg;
    return true;
}

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
#define MOTOR_CONFIG_FIELD(name, kind, nonneg, stop, word_max)                                                         \
    if (!settings_field_valid_##kind(c->name, (nonneg) != 0)) return false;
#include "motor_config_fields.inc"
#undef MOTOR_CONFIG_FIELD
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
#define MOTOR_CONFIG_FIELD(name, kind, nonneg, stop, word_max) candidate.config.name = motor->name;
#include "motor_config_fields.inc"
#undef MOTOR_CONFIG_FIELD
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
#define MOTOR_CONFIG_FIELD(name, kind, nonneg, stop, word_max) motor->name = config->name;
#include "motor_config_fields.inc"
#undef MOTOR_CONFIG_FIELD
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
#define MOTOR_CONFIG_FIELD(name, kind, nonneg, stop, word_max)                                                         \
    if ((stop) != 0 && from->name != to->name) return true;
#include "motor_config_fields.inc"
#undef MOTOR_CONFIG_FIELD
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

/* 按字段类型编码：FLOAT 写位模式，其余写 32 位字（与旧版逐字段 put_word 完全一致）。 */
static void settings_put_FLOAT(uint8_t **p, float value) { put_float(p, value); }
static void settings_put_BOOL(uint8_t **p, bool value) { put_word(p, value); }
static void settings_put_U16(uint8_t **p, uint16_t value) { put_word(p, value); }
static void settings_put_INT(uint8_t **p, int value) { put_word(p, (uint32_t)value); }

/* 按字段类型解码：非 FLOAT 字值超过 word_max 时拒绝。 */
static bool settings_get_FLOAT(const uint8_t **p, uint32_t word_max, float *out)
{
    (void)word_max;
    *out = get_float(p);
    return true;
}
static bool settings_get_BOOL(const uint8_t **p, uint32_t word_max, bool *out)
{
    uint32_t word = get_word(p);
    if (word > word_max) return false;
    *out = (bool)word;
    return true;
}
static bool settings_get_U16(const uint8_t **p, uint32_t word_max, uint16_t *out)
{
    uint32_t word = get_word(p);
    if (word > word_max) return false;
    *out = (uint16_t)word;
    return true;
}
static bool settings_get_INT(const uint8_t **p, uint32_t word_max, int *out)
{
    uint32_t word = get_word(p);
    if (word > word_max) return false;
    *out = (int)word;
    return true;
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
#define MOTOR_CONFIG_FIELD(name, kind, nonneg, stop, word_max) settings_put_##kind(&p, settings->config.name);
#include "motor_config_fields.inc"
#undef MOTOR_CONFIG_FIELD
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
#define MOTOR_CONFIG_FIELD(name, kind, nonneg, stop, word_max)                                                         \
    if (!settings_get_##kind(&p, word_max, &candidate.config.name)) return false;
#include "motor_config_fields.inc"
#undef MOTOR_CONFIG_FIELD
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
