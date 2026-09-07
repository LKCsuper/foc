/*
 * @Description: 配置/校准快照与固定格式编码，不保存命令或实时状态。
 * @Author: lkc
 */
#ifndef MOTOR_SETTINGS_H
#define MOTOR_SETTINGS_H
#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ol_max_rpm;           /* max target speed RPM */
    float ol_v_boost;           /* V/F 启动补偿 V */
    float ol_kv_f;              /* V/F 斜率 V/Hz */
    float ol_ramp_rpm_s;        /* 软启动速率 RPM/s */
    float ol_vq_limit;          /* Vq 上限 V */
    float ol_foc_freq;          /* ADC-DMA 基准频率 Hz */
    float ol_pole_pairs;        /* 电机极对数 */
    float ol_min_rpm;           /* 旧启动策略兼容参数；平滑启动不作转速地板 */
    float ol_current_kp;        /* 开环电流环 Kp */
    float ol_current_ki;        /* 开环电流环 Ki */
    float ol_start_phase_boost; /* 兼容旧启动方式的一拍相位补偿 rad */
    bool ol_align_enable;       /* true=每次 I/F 启动前执行预对齐 */
    float ol_align_current;     /* 对齐 D 轴电流 Id (A) */
    float ol_align_time;        /* 固定磁场保持时间 (s) */
    float ol_align_angle;       /* 固定对齐电角度 (rad) */
    bool hfi_start_enable;      /* true=I/F 启动前先扫描转子凸极轴 */
    float hfi_start_voltage;    /* 双极性注入电压幅值 V */
    uint16_t hfi_start_cycles;
    float hfi_start_min_confidence;
    bool speed_enable;
    bool speed_allow_braking;
    float speed_kp;
    float speed_ki;
    float speed_iq_limit;
    float speed_loop_hz;
    bool sensorless_enable;
    bool sensorless_takeover;
    float sensorless_motor_r;
    float sensorless_motor_l;
    float sensorless_flux_linkage;
    float sensorless_observer_gain;
    float sensorless_pll_kp;
    float sensorless_pll_ki;
    float sensorless_transition_rpm;
    float sensorless_fallback_rpm;
    float sensorless_openloop_max_rpm;
    float sensorless_lock_time;
    float sensorless_loss_time;
    float sensorless_blend_time;
    float sensorless_phase_tolerance;
    float sensorless_phase_offset;
    float sensorless_speed_filter;
    int hall_extra_samples; /* 霍尔 GPIO 额外滤波次数 */
} motor_config_t;

typedef struct {
    float current_offset[3];                   /* ADC count；0 表示尚未校准，上电检测后写入 Flash */
    float voltage_offset[3];                   /* 采样点 V */
    float undriven_offset[3];                  /* 采样点 V */
    uint8_t hall_table[MOTOR_HALL_TABLE_SIZE]; /* 0..200，255 未标定；保留旧表字节编码 */
} motor_calibration_t;

typedef struct {
    motor_config_t config;
    motor_calibration_t calibration;
} motor_settings_t;

/* v2 固定线格式：配置字、9 个 IEEE754 binary32、8 个霍尔字节。
 * 顺序固定，修改字段/顺序/编码必须更新 Flash 格式版本。禁止直接写 sizeof(struct)。 */
#define MOTOR_SETTINGS_WIRE_SIZE (216U)

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
