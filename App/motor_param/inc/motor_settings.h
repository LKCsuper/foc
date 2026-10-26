/*
 * @Description: 配置/校准快照与固定格式编码，不保存命令或实时状态。
 * @Author: lkc
 */
#ifndef MOTOR_SETTINGS_H
#define MOTOR_SETTINGS_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* 字段类型映射，供 motor_config_fields.inc 展开 */
#define MOTOR_CONFIG_TYPE_FLOAT float
#define MOTOR_CONFIG_TYPE_BOOL  bool
#define MOTOR_CONFIG_TYPE_U16   uint16_t
#define MOTOR_CONFIG_TYPE_INT   int

/**
 * @description: 可持久化的运行配置；字段、单位与约束见 motor_config_fields.inc。
 */
typedef struct {
#define MOTOR_CONFIG_FIELD(name, kind, nonneg, stop, word_max) MOTOR_CONFIG_TYPE_##kind name;
#include "motor_config_fields.inc"
#undef MOTOR_CONFIG_FIELD
} motor_config_t;

typedef struct {
    float current_offset[3];                   /* ADC count；0 表示尚未校准，上电检测后写入 Flash */
    float voltage_offset[3];                   /* 采样点 V */
    float undriven_offset[3];                  /* 采样点 V */
    uint8_t hall_table[MOTOR_HALL_TABLE_SIZE]; /* 0..200，255 未标定；保留旧表字节编码 */
    float encoder_offset_rad;
    bool encoder_inverted;
    bool encoder_calibrated;
} motor_calibration_t;

typedef struct {
    motor_config_t config;
    motor_calibration_t calibration;
} motor_settings_t;

/* v2 固定线格式：配置字、9 个 IEEE754 binary32、8 个霍尔字节。
 * 顺序固定，修改字段/顺序/编码必须更新 Flash 格式版本。禁止直接写 sizeof(struct)。 */
#define MOTOR_SETTINGS_WIRE_SIZE_V2 (216U)
#define MOTOR_SETTINGS_WIRE_SIZE    (228U)

/**
 * @description: 从旧电机对象提取配置/校准；调用者保证快照一致，不提取命令和积分。
 * @detail: 指针不可为空；配置单位沿用 motor_param.h，长度单位 byte。非 ISR 接口。
 * @param {const MOTOR_PARAM_S *} motor 已初始化的旧电机对象，不可为空
 * @param {motor_settings_t *} settings 输出快照，不可为空；失败不修改
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_Settings_Capture(const MOTOR_PARAM_S *motor, motor_settings_t *settings);

/**
 * @description: 停机时应用合法配置/校准；失败不修改对象，命令和实时状态均不改。
 * @detail: 指针不可为空；配置单位沿用 motor_param.h，长度单位 byte。非 ISR 接口。
 * @param {MOTOR_PARAM_S *} motor 已停机的目标电机对象，不可为空
 * @param {const motor_settings_t *} settings 已校验的配置与校准，不可为空
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_Settings_Apply(MOTOR_PARAM_S *motor, const motor_settings_t *settings);

/**
 * @description: 判断配置变更是否必须停机后才能应用。
 * @detail: 停机字段：极对数、控制/速度环频率、对齐/HFI、霍尔滤波次数、
 *          无感电机模型与观测器结构参数。下列允许在线修改：
 *          V/F 斜率与补偿、斜坡、Vq/电流环增益、max/min RPM、速度 PI 增益、
 *          制动开关、I/F 高速护栏与相位混合时间，以及命令型开关
 *          speed_enable / sensorless_enable / takeover。
 * @param {const motor_config_t *} from 当前配置，不可为空
 * @param {const motor_config_t *} to 目标配置，不可为空
 * @return {bool} true=必须停机；空指针视为需要停机
 * @author: lkc
 */
bool Motor_Settings_ConfigRequiresStop(const motor_config_t *from, const motor_config_t *to);

/**
 * @description: 应用配置（不改校准、不改命令目标电流/转速 enable）。
 * @detail: 先与当前校准拼成完整 settings 校验；仅停机字段变化且电机在跑时失败。
 *          成功时只写配置相关字段。非 ISR 接口。
 * @param {MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {const motor_config_t *} config 目标配置，不可为空
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_Settings_ApplyConfig(MOTOR_PARAM_S *motor, const motor_config_t *config);

/**
 * @description: 编码为固定小端格式；size 必须匹配，非法输入返回 false 且不写输出。
 * @detail: 指针不可为空；配置单位沿用 motor_param.h，长度单位 byte。非 ISR 接口。
 * @param {const motor_settings_t *} settings 输入配置/校准快照
 * @param {uint8_t *} data 输出固定编码缓冲区，不可为空
 * @param {uint32_t} size 缓冲区长度 (byte)，须等于 MOTOR_SETTINGS_WIRE_SIZE
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_Settings_Encode(const motor_settings_t *settings, uint8_t *data, uint32_t size);

/**
 * @description: 解码并校验；失败不修改输出，不依赖 MCU/RTOS 或 C 结构体填充。
 * @detail: 指针不可为空；配置单位沿用 motor_param.h，长度单位 byte。非 ISR 接口。
 * @param {motor_settings_t *} settings 输出快照，失败不修改
 * @param {const uint8_t *} data 输入固定编码缓冲区，不可为空
 * @param {uint32_t} size 缓冲区长度 (byte)，须等于 MOTOR_SETTINGS_WIRE_SIZE
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_Settings_Decode(const uint8_t *data, uint32_t size, motor_settings_t *settings);

#ifdef __cplusplus
}
#endif
#endif
