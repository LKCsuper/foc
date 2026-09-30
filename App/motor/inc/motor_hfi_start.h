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
#include <stdint.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_HFI_START_AXIS_COUNT (12U)
/* Private typedef -----------------------------------------------------------*/

typedef struct {
    bool enabled;
    float voltage;
    uint16_t cycles_per_axis;
    float min_confidence;
    float max_bus_voltage_ratio;
} motor_hfi_start_config_t;

typedef struct {
    bool apply_voltage;
    float vd;
    float theta;
} motor_hfi_start_output_t;

typedef enum {
    MOTOR_HFI_START_SKIPPED = 0,
    MOTOR_HFI_START_RUNNING,
    MOTOR_HFI_START_FINISHED
} motor_hfi_start_result_t;

typedef struct {
    bool running;
    bool attempted;
    bool succeeded;
    bool sample_valid;
    uint16_t axis;
    uint16_t transitions;
    int8_t polarity;
    float previous_current;
    float response[MOTOR_HFI_START_AXIS_COUNT];
    float positive_response[MOTOR_HFI_START_AXIS_COUNT];
    float negative_response[MOTOR_HFI_START_AXIS_COUNT];
    float angle;
    float confidence;
} motor_hfi_start_t;

/**
 * @description: 复位 HFI 初始位置估计状态机，将全部状态清零。
 * @param {motor_hfi_start_t} *state  HFI 状态机实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void MotorHfiStart_Reset(motor_hfi_start_t *state);

/**
 * @description: 校验并规整 HFI 配置参数，使取值合法。
 * @param {motor_hfi_start_config_t} *config  HFI 配置结构体，被原地规整
 * @return {*}
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

#ifdef __cplusplus
}
#endif

#endif
