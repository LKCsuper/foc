/*
 * @Description: I/F 启动前转子 D 轴预对齐状态机
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_alignment.h"
#include "common_assert.h"
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
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 复位对齐状态机，恢复到“已停止”的初始状态
 * @detail: 清零全部状态字段，并将 stopped 置位
 * @param {motor_alignment_t} *state   对齐状态机；为 NULL 时直接返回
 * @return {void} 无
 * @author: lkc
 */
void MotorAlignment_Reset(motor_alignment_t *state)
{
    COMMON_ASSERT(state != NULL);
    memset(state, 0, sizeof(*state));
    state->stopped = true;
}

/**
 * @description: 归一化对齐配置，将非法负值钳位为 0
 * @detail: 电流与持续时间为负时置 0；为 0 时调用侧视为关闭预对齐
 * @param {motor_alignment_config_t} *config   对齐配置；为 NULL 时直接返回
 * @return {void} 无
 * @author: lkc
 */
void MotorAlignment_NormalizeConfig(motor_alignment_config_t *config)
{
    MotorAlignment_NormalizeConfigInline(config);
}

/**
 * @description: 将状态机标记为“对齐完成”
 * @detail: 供其他定位流程（如 HFI 识别成功）代替预对齐完成后调用
 * @param {motor_alignment_t} *state   对齐状态机；为 NULL 时直接返回
 * @return {void} 无
 * @author: lkc
 */
void MotorAlignment_MarkComplete(motor_alignment_t *state)
{
    COMMON_ASSERT(state != NULL);
    state->stopped = false;
    state->running = false;
    state->complete = true;
}

/**
 * @description: 将状态机标记为“跳过对齐”
 * @detail: 供独立对齐状态机的调用者记录本次未执行保持阶段；主控制启动状态机
 *          会先完成定角电流爬升，再决定是否进入保持阶段。
 * @param {motor_alignment_t} *state   对齐状态机；为 NULL 时直接返回
 * @return {void} 无
 * @author: lkc
 */
void MotorAlignment_MarkSkipped(motor_alignment_t *state)
{
    COMMON_ASSERT(state != NULL);
    state->stopped = false;
    state->running = false;
    state->complete = false;
}

/**
 * @description: 推进对齐状态机一个控制周期
 * @detail: 停止状态下满足“使能、电流>0、时长>0”时启动；运行中输出定位角度
 *          与 D 轴电流给定 (Iq=0)；时长累计到配置值后停止保持，并区分返回
 *          SKIPPED / STARTED / RUNNING / FINISHED
 * @param {motor_alignment_t} *state         对齐状态机
 * @param {motor_alignment_config_t} *config 对齐配置
 * @param {float} dt                         本拍时间步长 (s)
 * @param {motor_alignment_output_t} *output 输出：保持位置、角度、id/iq 给定
 * @return {motor_alignment_result_t} SKIPPED/STARTED/RUNNING/FINISHED
 * @author: lkc
 */
motor_alignment_result_t MotorAlignment_Step(
    motor_alignment_t *state,
    const motor_alignment_config_t *config,
    float dt,
    motor_alignment_output_t *output)
{
    bool started = false;

    if (output != NULL) memset(output, 0, sizeof(*output));
    if (state == NULL || config == NULL || output == NULL) return MOTOR_ALIGNMENT_SKIPPED;

    if (state->stopped && config->enabled && config->current > 0.0f &&
        config->duration > 0.0f) {
        state->stopped = false;
        state->running = true;
        state->complete = false;
        state->elapsed = 0.0f;
        started = true;
    }
    if (!state->running) return MOTOR_ALIGNMENT_SKIPPED;

    output->hold_position = true;
    output->angle = config->angle;
    output->id_target = config->current;
    output->iq_target = 0.0f;

    if (dt > 0.0f) state->elapsed += dt;
    if (state->elapsed >= config->duration) {
        state->elapsed = config->duration;
        state->running = false;
        state->complete = true;
        output->hold_position = false;
        return MOTOR_ALIGNMENT_FINISHED;
    }
    return started ? MOTOR_ALIGNMENT_STARTED : MOTOR_ALIGNMENT_RUNNING;
}

#ifdef __cplusplus
}
#endif
