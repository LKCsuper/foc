/*
 * @Description: FOC 控制环分频计数
 * @Author: lkc
 *
 * 学习要点：ADC-DMA 回调频率常高于电流环（如 30kHz vs 15kHz）。
 * 本计数器只决定这一回调是否跑完整 FOC。采样和软件保护由调用方放在完整拍，
 * 跳过拍不再做这两项。FOC 的 dt 须乘以 divider，否则斜坡/PI/观测器时间尺度会错。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_foc_divider.h"
#include "common_assert.h"
#include <stddef.h>
#if defined(__CC_ARM)
/* 每个 ADC 回调都会调用，局部 O2；不改浮点模式。 */
#pragma O2
#endif
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 初始化分频状态
 * @param {motor_foc_divider_t} *state  分频状态；NULL 直接返回
 * @param {int} divider                 分频数；<=0 时置为 1
 * @return {*}
 * @author: lkc
 */
void Motor_FocDivider_Init(motor_foc_divider_t *state, int divider)
{
    COMMON_ASSERT(state != NULL);
    state->skip = 0;
    state->divider = (divider > 0) ? divider : 1;
}

/**
 * @description: 每个 ADC 回调推进一拍，判断本拍是否应跑完整 FOC。
 * @detail: divider=2 时模式为 false,true,false,true…；NULL 恒 true（无分频兜底）。
 *          返回 false 时调用方不得补跑采样或保护。
 * @param {motor_foc_divider_t *} state  分频状态
 * @return {bool} true=本拍执行采样、保护、电流环和 SVPWM
 * @author: lkc
 */
bool Motor_FocDivider_ShouldRun(motor_foc_divider_t *state)
{
    int divider;

    if (state == NULL) return true;

    divider = state->divider;
    if (divider <= 0) {
        divider = 1;
        state->divider = 1;
    }

    if (++state->skip >= divider) {
        state->skip = 0;
        return true;
    }
    return false;
}

/**
 * @description: 按计数方向与半周期内位置判断本回调是否跑完整 FOC。
 * @detail: 软件计数分频在回调被推迟或 DMA 完成标志合并时会改变奇偶，使完整拍跳到另一个 PWM 半周期；
 *          本函数只看定时器方向和位置，不累计历史，因此不会出现这种跳变。
 * @param {motor_foc_phase_gate_t *} gate 门控状态，不可为空
 * @param {const motor_foc_phase_gate_config_t *} config 门控参数，不可为空
 * @param {bool} counting_down 定时器是否处于向下计数
 * @param {uint32_t} count 定时器当前计数
 * @param {uint32_t} period 自动重装值
 * @param {uint32_t} now_us 当前微秒时间戳，允许回绕
 * @return {bool} true=本拍执行完整 FOC
 * @author: lkc
 */
bool Motor_FocDivider_PhaseGate(motor_foc_phase_gate_t *gate, const motor_foc_phase_gate_config_t *config,
                                bool counting_down, uint32_t count, uint32_t period, uint32_t now_us)
{
    uint32_t since_top;

    COMMON_ASSERT(gate != NULL);
    COMMON_ASSERT(config != NULL);
    if (!counting_down || count > period) return false;
    /* 中心对齐向下计数：顶部更新时 CNT=ARR，之后递减，ARR-CNT 即顶部更新后经过的计数。 */
    since_top = period - count;
    if (since_top < config->min_counts || since_top > config->max_counts) return false;
    if (gate->seeded && (uint32_t)(now_us - gate->last_us) < config->repeat_us) return false;
    gate->last_us = now_us;
    gate->seeded = true;
    return true;
}

bool Motor_FocDivider_ControlExpired(motor_foc_phase_gate_t *clock, bool running, bool execute,
                                     uint32_t now_us, uint32_t max_gap_us)
{
    COMMON_ASSERT(clock != NULL);
    if (!running) {
        clock->seeded = false;
        return false;
    }
    if (!clock->seeded) {
        clock->last_us = now_us;
        clock->seeded = true;
    }
    if (max_gap_us == 0U || (uint32_t)(now_us - clock->last_us) > max_gap_us) return true;
    if (execute) clock->last_us = now_us;
    return false;
}

#ifdef __cplusplus
}
#endif
