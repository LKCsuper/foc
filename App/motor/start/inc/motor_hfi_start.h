/*
 * @Description: 零速 HFI 初始位置估计状态机
 * @Author: lkc
 */
#ifndef MOTOR_HFI_START_H
#define MOTOR_HFI_START_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "common_assert.h"
#include "motor_config.h"
#include <stddef.h>
#include <stdint.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_HFI_START_AXIS_COUNT (12U)
/* Private typedef -----------------------------------------------------------*/

/* HFI 配置：探测电压、每轴周期数、置信度门限与母线电压比例。 */
typedef struct {
    bool enabled;                /* true=启用 HFI 初始位置估计 */
    float voltage;               /* 探测电压 (V) */
    uint16_t cycles_per_axis;    /* 每个探测轴的正负交替周期数 */
    float min_confidence;        /* 成功判定的最小置信度 (0..1) */
    float max_bus_voltage_ratio; /* 探测电压相对母线电压的上限比例 */
} motor_hfi_start_config_t;

/* HFI 每拍输出。 */
typedef struct {
    bool apply_voltage;  /* true=本拍施加探测电压 */
    float vd;            /* D 轴探测电压 (V) */
    float theta;         /* 探测电角 (rad) */
} motor_hfi_start_output_t;

/* HFI 结果码。 */
typedef enum {
    MOTOR_HFI_START_SKIPPED = 0,  /* 未启用/参数无效/已尝试过 */
    MOTOR_HFI_START_RUNNING,      /* 扫描中 */
    MOTOR_HFI_START_FINISHED      /* 扫描完成 */
} motor_hfi_start_result_t;

/* HFI 状态机状态：扫描进度与各轴电流响应累计。 */
typedef struct {
    bool running;        /* true=扫描进行中 */
    bool attempted;      /* true=已执行过 */
    bool succeeded;      /* true=估计成功 */
    bool sample_valid;   /* true=已有上一拍电流样本 */
    uint16_t axis;       /* 当前探测轴 */
    uint16_t transitions; /* 当前轴已完成的极性切换次数 */
    int8_t polarity;     /* 当前极性 (+1/-1) */
    float previous_current;                                   /* 上一拍投影电流 (A) */
    float response[MOTOR_HFI_START_AXIS_COUNT];               /* 各轴总响应 */
    float positive_response[MOTOR_HFI_START_AXIS_COUNT];      /* 各轴正极性响应 */
    float negative_response[MOTOR_HFI_START_AXIS_COUNT];      /* 各轴负极性响应 */
    float angle;        /* 估计转子角 (rad) */
    float confidence;   /* 估计置信度 (0..1) */
} motor_hfi_start_t;

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 复位 HFI 初始位置估计状态机，将全部状态清零。
 * @param {motor_hfi_start_t} *state  HFI 状态机实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 热路径共享实现；与NormalizeConfig保持同一套规则，不缓存配置。
 * @param {motor_hfi_start_config_t *} config 配置，不可为空
 * @return {void} 无
 */
static inline void MotorHfiStart_NormalizeConfigInline(motor_hfi_start_config_t *config)
{
    COMMON_ASSERT(config != NULL);
    if (config->voltage < 0.0f) config->voltage = 0.0f;
    if (config->cycles_per_axis < 2U) config->cycles_per_axis = 2U;
    if (config->cycles_per_axis > 32U) config->cycles_per_axis = 32U;
    if (config->min_confidence < 0.0f) config->min_confidence = 0.0f;
    if (config->min_confidence > 1.0f) config->min_confidence = 1.0f;
    if (config->max_bus_voltage_ratio <= 0.0f || config->max_bus_voltage_ratio > 1.0f)
        config->max_bus_voltage_ratio = 0.20f;
}

#if MOTOR_CFG_SENSORLESS
/**
 * @description: 复位 HFI 初始位置估计状态机，将全部状态清零。
 * @param {motor_hfi_start_t} *state  HFI 状态机实例，可为 NULL
 * @return {void} 无
 */
void MotorHfiStart_Reset(motor_hfi_start_t *state);

/**
 * @description: 校验并规整 HFI 配置参数，使取值合法。
 * @param {motor_hfi_start_config_t} *config  HFI 配置结构体，被原地规整
 * @return {void} 无
 * @author: lkc
 */
void MotorHfiStart_NormalizeConfig(motor_hfi_start_config_t *config);

/**
 * @description: HFI 零速初始位置估计单步，每 FOC 周期调用一次。
 * @param {motor_hfi_start_t} *state                HFI 状态机实例
 * @param {const motor_hfi_start_config_t} *config  HFI 配置
 * @param {float} i_alpha                           定子 α 轴电流采样 (A)
 * @param {float} i_beta                            定子 β 轴电流采样 (A)
 * @param {float} v_bus                             母线电压 (V)
 * @param {motor_hfi_start_output_t} *output        本拍探测电压输出
 * @return {motor_hfi_start_result_t} SKIPPED/RUNNING/FINISHED
 * @author: lkc
 */
motor_hfi_start_result_t MotorHfiStart_Step(
    motor_hfi_start_t *state,
    const motor_hfi_start_config_t *config,
    float i_alpha,
    float i_beta,
    float v_bus,
    motor_hfi_start_output_t *output);
#else
/**
 * @description: 模块关闭时的兼容空实现；不执行对应功能。
 * @param {motor_hfi_start_t *} state 兼容参数，本实现不使用
 * @return {void} 无
 */
static inline void MotorHfiStart_Reset(motor_hfi_start_t *state) { (void)state; }
/**
 * @description: 校验并规整 HFI 配置参数，使取值合法。
 * @param {motor_hfi_start_config_t} *config  HFI 配置结构体，被原地规整
 * @return {void} 无
 */
static inline void MotorHfiStart_NormalizeConfig(motor_hfi_start_config_t *config)
{
    MotorHfiStart_NormalizeConfigInline(config);
}
/**
 * @description: HFI 零速初始位置估计单步，每 FOC 周期调用一次。
 * @param {motor_hfi_start_t} *state                HFI 状态机实例，保存扫描进度与累计响应
 * @param {const motor_hfi_start_config_t} *config  HFI 配置（电压、每轴周期数、置信度门限、母线电压比例）
 * @param {float} i_alpha                           定子 α 轴电流采样 (A)
 * @param {float} i_beta                            定子 β 轴电流采样 (A)
 * @param {float} v_bus                             母线电压 (V)，限制探测电压上限
 * @param {motor_hfi_start_output_t} *output        本拍探测电压输出（apply_voltage/vd/theta）
 * @return {motor_hfi_start_result_t} 当前值 MOTOR_HFI_START_SKIPPED
 */
static inline motor_hfi_start_result_t MotorHfiStart_Step(motor_hfi_start_t *state,
                                                          const motor_hfi_start_config_t *config, float i_alpha,
                                                          float i_beta, float v_bus, motor_hfi_start_output_t *output)
{
    (void)state;
    (void)config;
    (void)i_alpha;
    (void)i_beta;
    (void)v_bus;
    if (output != NULL) {
        output->apply_voltage = false;
        output->vd = 0.0f;
        output->theta = 0.0f;
    }
    return MOTOR_HFI_START_SKIPPED;
}
#endif

#ifdef __cplusplus
}
#endif

#endif
