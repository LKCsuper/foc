/*
 * @Description: HFI 凸极扫描零速初始位置估计
 * @Author: lkc
 */
#include "motor_hfi_start.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define MOTOR_HFI_START_PI                   (3.14159265358979323846f)
#define MOTOR_HFI_START_TWO_PI               (2.0f * MOTOR_HFI_START_PI)
#define MOTOR_HFI_START_MIN_CURRENT_RESPONSE (1.0e-5f)

/**
 * @description: 将角度循环归一化到 [0, 2π) 区间。
 * @param {float} angle  待归一化的角度 (rad)
 * @return {float} 归一化后的角度 (rad)
 * @author: lkc
 */
static float MotorHfiStart_NormalizeAngle(float angle)
{
    while (angle >= MOTOR_HFI_START_TWO_PI) angle -= MOTOR_HFI_START_TWO_PI;
    while (angle < 0.0f) angle += MOTOR_HFI_START_TWO_PI;
    return angle;
}

/**
 * @description: 复位 HFI 初始位置估计状态机，将全部状态清零。
 * @param {motor_hfi_start_t} *state  HFI 状态机实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void MotorHfiStart_Reset(motor_hfi_start_t *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

/**
 * @description: 校验并规整 HFI 配置参数，使取值合法。
 * @detail: 电压非负、每轴周期数与置信度限幅；max_bus_voltage_ratio 非法时回退为默认值 0.20。
 * @param {motor_hfi_start_config_t} *config  HFI 配置结构体，被原地规整
 * @return {*}
 * @author: lkc
 */
void MotorHfiStart_NormalizeConfig(motor_hfi_start_config_t *config)
{
    if (config == NULL) return;
    if (config->voltage < 0.0f) config->voltage = 0.0f;
    if (config->cycles_per_axis < 2U) config->cycles_per_axis = 2U;
    if (config->cycles_per_axis > 32U) config->cycles_per_axis = 32U;
    if (config->min_confidence < 0.0f) config->min_confidence = 0.0f;
    if (config->min_confidence > 1.0f) config->min_confidence = 1.0f;
    if (config->max_bus_voltage_ratio <= 0.0f ||
        config->max_bus_voltage_ratio > 1.0f) {
        config->max_bus_voltage_ratio = 0.20f;
    }
}

/**
 * @description: 全部轴扫描完成后收尾，估计转子初始角度与可信度。
 * @detail: 取电流响应最大的轴估计转子角，依据正/负极性响应差判断磁极方向
 *          （必要时补 π），计算置信度并置 succeeded/attempted 标志。
 * @param {motor_hfi_start_t} *state            HFI 状态机实例，保存各轴响应并写回结果
 * @param {const motor_hfi_start_config_t} *config   HFI 配置，min_confidence 作为成功判定门限
 * @return {*}
 * @author: lkc
 */
static void MotorHfiStart_Finish(motor_hfi_start_t *state,
                                 const motor_hfi_start_config_t *config)
{
    uint16_t i;
    uint16_t best = 0U;
    float maximum = state->response[0];
    float minimum = maximum;

    for (i = 1U; i < MOTOR_HFI_START_AXIS_COUNT; i++) {
        if (state->response[i] > maximum) {
            maximum = state->response[i];
            best = i;
        }
        if (state->response[i] < minimum) minimum = state->response[i];
    }

    state->confidence = maximum > MOTOR_HFI_START_MIN_CURRENT_RESPONSE
        ? (maximum - minimum) / maximum : 0.0f;
    state->angle = MOTOR_HFI_START_PI * (float)best /
                   (float)MOTOR_HFI_START_AXIS_COUNT;
    if (state->negative_response[best] > state->positive_response[best]) {
        state->angle += MOTOR_HFI_START_PI;
    }
    state->angle = MotorHfiStart_NormalizeAngle(state->angle);
    state->succeeded = state->confidence >= config->min_confidence;
    state->running = false;
    state->attempted = true;
}

/**
 * @description: HFI 零速初始位置估计单步，每 FOC 周期调用一次。
 * @detail: 在试探角上交替施加正/负 d 轴电压并累加电流增量响应，
 *          扫完 12 个轴后结束，由状态机给出转子初始角度估计。
 * @param {motor_hfi_start_t} *state                HFI 状态机实例，保存扫描进度与累计响应
 * @param {const motor_hfi_start_config_t} *config  HFI 配置（电压、每轴周期数、置信度门限、母线电压比例）
 * @param {float} i_alpha                           定子 α 轴电流采样 (A)
 * @param {float} i_beta                            定子 β 轴电流采样 (A)
 * @param {float} v_bus                             母线电压 (V)，限制探测电压上限
 * @param {motor_hfi_start_output_t} *output        本拍探测电压输出（apply_voltage/vd/theta）
 * @return {motor_hfi_start_result_t} SKIPPED=未启用/参数无效/已尝试过；RUNNING=扫描中；FINISHED=扫描完成
 * @author: lkc
 */
motor_hfi_start_result_t MotorHfiStart_Step(
    motor_hfi_start_t *state,
    const motor_hfi_start_config_t *config,
    float i_alpha,
    float i_beta,
    float v_bus,
    motor_hfi_start_output_t *output)
{
    float theta;
    float current;
    float delta;
    float voltage;

    if (output != NULL) memset(output, 0, sizeof(*output));
    if (state == NULL || config == NULL || output == NULL ||
        !config->enabled || config->voltage <= 0.0f || state->attempted) {
        return MOTOR_HFI_START_SKIPPED;
    }

    if (!state->running) {
        MotorHfiStart_Reset(state);
        state->running = true;
        state->polarity = 1;
    }

    theta = MOTOR_HFI_START_PI * (float)state->axis /
            (float)MOTOR_HFI_START_AXIS_COUNT;
    current = i_alpha * cosf(theta) + i_beta * sinf(theta);

    if (state->sample_valid) {
        delta = fabsf(current - state->previous_current);
        state->response[state->axis] += delta;
        if (state->polarity > 0) state->positive_response[state->axis] += delta;
        else state->negative_response[state->axis] += delta;
        state->transitions++;
    }

    state->previous_current = current;
    state->sample_valid = true;
    state->polarity = -state->polarity;

    if (state->transitions >= (uint16_t)(config->cycles_per_axis * 2U)) {
        state->axis++;
        state->transitions = 0U;
        state->sample_valid = false;
        state->polarity = 1;
        if (state->axis >= MOTOR_HFI_START_AXIS_COUNT) {
            MotorHfiStart_Finish(state, config);
            return MOTOR_HFI_START_FINISHED;
        }
        theta = MOTOR_HFI_START_PI * (float)state->axis /
                (float)MOTOR_HFI_START_AXIS_COUNT;
    }

    voltage = config->voltage;
    if (voltage > v_bus * config->max_bus_voltage_ratio) {
        voltage = v_bus * config->max_bus_voltage_ratio;
    }
    output->apply_voltage = true;
    output->vd = (float)state->polarity * voltage;
    output->theta = theta;
    return MOTOR_HFI_START_RUNNING;
}
