/*
 * @Description: FOC 控制环分频计数（ADC 回调 → 是否跑完整 FOC）
 * @Author: lkc
 *
 * 学习要点：分频只决定是否跑完整 FOC；采样和保护在完整拍，dt 须×divider。
 */
#ifndef MOTOR_FOC_DIVIDER_H
#define MOTOR_FOC_DIVIDER_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: FOC 控制环分频状态（不持久化）
 * @detail: skip 为累计拍数；满 divider 清零并返回“本拍跑 FOC”
 */
typedef struct {
    int skip;    /* 距下次 FOC 的累计拍数 */
    int divider; /* 分频数，<=0 时按 1 处理 */
} motor_foc_divider_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化分频状态
 * @param {motor_foc_divider_t} *state  分频状态；NULL 直接返回
 * @param {int} divider                 分频数；<=0 时置为 1
 * @return {*}
 * @author: lkc
 */
void Motor_FocDivider_Init(motor_foc_divider_t *state, int divider);

/**
 * @description: 每个 ADC 回调推进一拍，判断本拍是否应跑完整 FOC
 * @detail: 返回 true 时调用方才采样、保护和跑电流环；返回 false 的拍不再补做这些
 * @param {motor_foc_divider_t} *state  分频状态；NULL 时恒返回 true
 * @return {bool} true=本拍执行采样、保护、电流环和 SVPWM
 * @author: lkc
 */
bool Motor_FocDivider_ShouldRun(motor_foc_divider_t *state);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
