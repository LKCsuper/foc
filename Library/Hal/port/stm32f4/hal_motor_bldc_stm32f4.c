/*
 * @Description: BM75/STM32F4 六步换相功率桥实现
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#ifndef HAL_INTERNAL
#define HAL_INTERNAL
#endif
#include "includes.h"
#include "hal_motor_bldc.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint16_t s_logical_to_channel[3] = {
    TIM_Channel_1, /* 逻辑 A -> INHA */
    TIM_Channel_3, /* 逻辑 B -> INHC（BM75 软件 ACB 映射） */
    TIM_Channel_2, /* 逻辑 C -> INHB */
};
/* Private function prototypes -----------------------------------------------*/
static void Hal_Motor_BldcConfigurePhase(uint16_t channel, hal_motor_bldc_phase_mode_t mode);
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 配置一个 TIM1 半桥通道的六步输出状态。
 * @detail: PWM 使用互补 PWM1；LOW 强制主输出无效并使能互补输出；FLOAT 同时关闭上下管。
 * @param {uint16_t} channel TIM1 通道
 * @param {hal_motor_bldc_phase_mode_t} mode 目标桥臂状态
 * @return {void} 无
 * @author: lkc
 */
static void Hal_Motor_BldcConfigurePhase(uint16_t channel, hal_motor_bldc_phase_mode_t mode)
{
    if (mode == HAL_MOTOR_BLDC_PHASE_PWM) {
        TIM_SelectOCxM(TIM1, channel, TIM_OCMode_PWM1);
        TIM_CCxCmd(TIM1, channel, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, channel, TIM_CCxN_Enable);
    } else if (mode == HAL_MOTOR_BLDC_PHASE_LOW) {
        TIM_SelectOCxM(TIM1, channel, TIM_ForcedAction_InActive);
        TIM_CCxCmd(TIM1, channel, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, channel, TIM_CCxN_Enable);
    } else {
        TIM_SelectOCxM(TIM1, channel, TIM_ForcedAction_InActive);
        TIM_CCxCmd(TIM1, channel, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, channel, TIM_CCxN_Disable);
    }
}

/**
 * @description: 同步应用 BM75 三相六步桥臂命令。
 * @detail: 逻辑 A/B/C 在本移植层映射到 TIM1 CH1/CH3/CH2；通过 COM 事件同步模式更新。
 * @param {hal_motor_bldc_phase_mode_t} phase_a 逻辑 A 相状态
 * @param {hal_motor_bldc_phase_mode_t} phase_b 逻辑 B 相状态
 * @param {hal_motor_bldc_phase_mode_t} phase_c 逻辑 C 相状态
 * @param {float} duty PWM 相占空比 [0,1]
 * @return {void} 无
 * @author: lkc
 */
void Hal_Motor_BldcApply(hal_motor_bldc_phase_mode_t phase_a,
                         hal_motor_bldc_phase_mode_t phase_b,
                         hal_motor_bldc_phase_mode_t phase_c, float duty)
{
    hal_motor_bldc_phase_mode_t phase_mode[3];
    uint32_t period = (uint32_t)TIM1->ARR;
    uint32_t pulse;
    uint8_t phase;

    phase_mode[0] = phase_a;
    phase_mode[1] = phase_b;
    phase_mode[2] = phase_c;
    if (!(duty >= 0.0f)) {
        pulse = 0U;
    } else if (duty >= 1.0f) {
        pulse = period;
    } else {
        pulse = (uint32_t)(duty * (float)period);
    }

    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->CCR1 = pulse;
    TIM1->CCR2 = pulse;
    TIM1->CCR3 = pulse;
    for (phase = 0U; phase < 3U; phase++) {
        Hal_Motor_BldcConfigurePhase(s_logical_to_channel[phase], phase_mode[phase]);
    }
    TIM1->CR1 &= ~TIM_CR1_UDIS;
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}
#ifdef __cplusplus
}
#endif
