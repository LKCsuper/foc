/*
 * @Description: FOC 控制环分频计数（ADC 回调 → 是否跑完整 FOC）
 * @Author: lkc
 *
 * 学习要点：采样/软件保护每拍；电流环与 SVPWM 按 divider 分频；FOC 的 dt 须×divider。
 */
#ifndef _MOTOR_FOC_DIVIDER_H
#define _MOTOR_FOC_DIVIDER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @description: FOC 控制环分频状态（不持久化）。
 * @detail: skip 为累计拍数；满 divider 清零并返回“本拍跑 FOC”。
 */
typedef struct {
    int skip;     /* 距下次 FOC 的累计拍数 */
    int divider;  /* 分频数，<=0 时按 1 处理 */
} motor_foc_divider_t;

/**
 * @description: 初始化分频状态。
 * @param {motor_foc_divider_t *} state  分频状态；NULL 直接返回
 * @param {int} divider  分频数；<=0 时置为 1
 * @return {*}
 * @author: lkc
 */
void Motor_FocDivider_Init(motor_foc_divider_t *state, int divider);

/**
 * @description: 每个 ADC 回调推进一拍，判断本拍是否应跑完整 FOC。
 * @detail: 采样/保护不经此门；仅电流环、观测器、SVPWM 使用返回值。
 * @param {motor_foc_divider_t *} state  分频状态；NULL 时恒返回 true
 * @return {bool} true=本拍执行电流环/观测器/SVPWM
 * @author: lkc
 */
bool Motor_FocDivider_ShouldRun(motor_foc_divider_t *state);

#ifdef __cplusplus
}
#endif

#endif
