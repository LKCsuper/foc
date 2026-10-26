/*
 * @Description: I/F 开环启动：相位补偿、Iq 缩放与角度积分转速
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_if_start.h"
#include "common_assert.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#if defined(__CC_ARM)
/* 位于 ADC 中断控制路径，与其他实时模块一致局部 O2；不启用快速浮点，公式不变。 */
#pragma O2
#endif
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static float MotorIfStart_Sign(float value);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 符号函数，负数返回 -1.0，否则返回 +1.0。
 * @param {float} value  待取符号的数值
 * @return {float} 数值的符号：负数 -1.0，其余 +1.0
 * @author: lkc
 */
static float MotorIfStart_Sign(float value)
{
    return value < 0.0f ? -1.0f : 1.0f;
}

/**
 * @description: 复位 I/F 启动状态机并恢复默认运行值。
 * @detail: 清零全部状态后，置首周期标志并将 Iq 缩放设为 1。
 * @param {motor_if_start_t} *state  I/F 启动状态机实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void MotorIfStart_Reset(motor_if_start_t *state)
{
    COMMON_ASSERT(state != NULL);
    memset(state, 0, sizeof(*state));
    state->first_cycle = true;
    state->iq_scale = 1.0f;
}

/**
 * @description: 规整 I/F 启动配置，使取值合法。
 * @detail: min_rpm 限制为非负，min_iq_scale 限定在 [0, 1]。
 * @param {motor_if_start_config_t} *config  I/F 启动配置结构体，被原地规整
 * @return {*}
 * @author: lkc
 */
void MotorIfStart_NormalizeConfig(motor_if_start_config_t *config)
{
    COMMON_ASSERT(config != NULL);
    if (config->min_rpm < 0.0f) config->min_rpm = 0.0f;
    if (config->min_iq_scale < 0.0f) config->min_iq_scale = 0.0f;
    if (config->min_iq_scale > 1.0f) config->min_iq_scale = 1.0f;
}

/**
 * @description: 置为“转子已对齐/位置已知”的中性状态。
 * @detail: 停止角度推进（0 RPM）、清除相位补偿、恢复满 Iq，
 *          供预对齐或 HFI 测出位置后的正常 I/F 起步使用。
 * @param {motor_if_start_t} *state  I/F 启动状态机实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void MotorIfStart_SetNeutral(motor_if_start_t *state)
{
    COMMON_ASSERT(state != NULL);
    state->angle_rpm = 0.0f;
    state->iq_scale = 1.0f;
    state->phase_boost = 0.0f;
}

/**
 * @description: V/F 起步，直接设定角度积分转速。
 * @detail: V/F 不需要 I/F 的相位补偿与电流缩放，故同时复位两者。
 * @param {motor_if_start_t} *state  I/F 启动状态机实例，可为 NULL
 * @param {float} rpm                角度积分转速 (RPM)
 * @return {*}
 * @author: lkc
 */
void MotorIfStart_SetVfRpm(motor_if_start_t *state, float rpm)
{
    COMMON_ASSERT(state != NULL);
    state->angle_rpm = rpm;
    state->iq_scale = 1.0f;
    state->phase_boost = 0.0f;
}

/**
 * @description: 每 FOC 周期更新一次 I/F 启动派生量。
 * @detail: 首周期且转子位置未知时注入一次带方向的启动相位补偿；
 *          转子位置未知且转速低于 min_rpm 时按转速比例减小 Iq 缩放（带下限）；
 *          转子位置已知用实际转速积分角度，未达最低速时跳升至最低速。
 * @param {motor_if_start_t} *state                I/F 启动状态机实例，派生量写回本结构体
 * @param {const motor_if_start_config_t} *config  I/F 启动配置（min_rpm/phase_boost/min_iq_scale）
 * @param {const motor_if_start_input_t} *input    本拍输入（目标/当前转速、转子位置是否已知）
 * @return {*}
 * @author: lkc
 */
void MotorIfStart_Step(motor_if_start_t *state,
                       const motor_if_start_config_t *config,
                       const motor_if_start_input_t *input)
{
    float scale;

    COMMON_ASSERT(state != NULL && config != NULL && input != NULL);
    state->phase_boost = 0.0f;

    if (state->first_cycle && input->target_rpm != 0.0f) {
        if (!input->rotor_position_known) {
            state->phase_boost = MotorIfStart_Sign(input->target_rpm) *
                                 config->phase_boost;
        }
        state->first_cycle = false;
    }

    state->iq_scale = 1.0f;
    if (!input->rotor_position_known && config->min_rpm > 0.0f &&
        fabsf(input->current_rpm) < config->min_rpm) {
        scale = fabsf(input->current_rpm) / config->min_rpm;
        if (scale < config->min_iq_scale) scale = config->min_iq_scale;
        state->iq_scale = scale;
    }

    if (input->rotor_position_known) {
        state->angle_rpm = input->current_rpm;
    } else if (fabsf(input->target_rpm) >= config->min_rpm &&
               fabsf(input->current_rpm) < config->min_rpm) {
        state->angle_rpm = MotorIfStart_Sign(input->target_rpm) *
                           config->min_rpm;
    } else {
        state->angle_rpm = input->current_rpm;
    }
}

#ifdef __cplusplus
}
#endif
