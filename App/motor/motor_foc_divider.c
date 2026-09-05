/*
 * @Description: FOC 控制环分频计数
 * @Author: lkc
 *
 * 学习要点：ADC-DMA 回调频率常高于电流环（如 30kHz vs 15kHz）。
 * 采样与软件保护每拍都跑；完整 FOC（电流环/观测器/SVPWM）按 divider 分频。
 * FOC 的 dt 须乘以 divider，否则斜坡/PI/观测器时间尺度会错。
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "motor_foc_divider.h"

#include <stddef.h>

/**
 * @description: 初始化分频状态。
 * @param {motor_foc_divider_t *} state  分频状态；NULL 直接返回
 * @param {int} divider  分频数；<=0 时置为 1
 * @return {*}
 * @author: lkc
 */
void Motor_FocDivider_Init(motor_foc_divider_t *state, int divider)
{
    if (state == NULL) {
        return;
    }
    state->skip = 0;
    state->divider = (divider > 0) ? divider : 1;
}

/**
 * @description: 每个 ADC 回调推进一拍，判断本拍是否应跑完整 FOC。
 * @detail: divider=2 时模式为 false,true,false,true…；NULL 恒 true（无分频兜底）。
 * @param {motor_foc_divider_t *} state  分频状态
 * @return {bool} true=本拍执行电流环/观测器/SVPWM
 * @author: lkc
 */
bool Motor_FocDivider_ShouldRun(motor_foc_divider_t *state)
{
    int divider;

    if (state == NULL) {
        return true;
    }

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

#ifdef __cplusplus
}
#endif
