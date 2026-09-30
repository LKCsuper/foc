/*
 * @Description: 短临界区（屏蔽 IRQ）通用实现
 * @Author: lkc
 *
 * 学习要点：
 *   1) 主机编译时编译器守卫不成立，退化为空操作，测试无需桩。
 *   2) 进入返回 PRIMASK；退出仅在进入前 IRQ 开启时恢复，支持嵌套调用场景不误开中断。
 *   3) 本工程是 Keil ARMCC5，它不定义 __ARM_ARCH。守卫必须认 __CC_ARM，否则目标上临界区会被编译掉。
 */
#ifndef MOTOR_CRIT_H
#define MOTOR_CRIT_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* ARMCC5 认 __CC_ARM；ARMCC6 认 __ARMCC_VERSION；GCC-ARM 认 __arm__。三者都不成立时视为主机。 */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || (defined(__GNUC__) && defined(__arm__))
#include "stm32f4xx.h"
#define MOTOR_CRIT_HAS_IRQ_LOCK 1
#else
#define MOTOR_CRIT_HAS_IRQ_LOCK 0
#endif
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 进入短临界区（屏蔽 IRQ），返回进入前的 PRIMASK。
 * @detail: 目标上先读 PRIMASK 再关中断。返回值非 0 表示进入前中断已关，配对的 Exit 不会重新打开。
 * @return {uint32_t} 进入前 PRIMASK；非 ARM 平台返回 0
 * @author: lkc
 */
static inline uint32_t Motor_Crit_Enter(void)
{
#if MOTOR_CRIT_HAS_IRQ_LOCK
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
#else
    return 0U;
#endif
}

/**
 * @description: 退出短临界区；仅当进入前 IRQ 开启时恢复。
 * @param {uint32_t} primask 进入临界区时的 PRIMASK（0=原 IRQ 开启）
 * @return {void} 无
 * @author: lkc
 */
static inline void Motor_Crit_Exit(uint32_t primask)
{
#if MOTOR_CRIT_HAS_IRQ_LOCK
    if (primask == 0U) {
        __enable_irq();
    }
#else
    (void)primask;
#endif
}

#ifdef __cplusplus
}
#endif
#endif
