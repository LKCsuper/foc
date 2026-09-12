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
#define HAL_MOTOR_BLDC_FOC_SAMPLE_COUNT 250U
#define HAL_MOTOR_BLDC_MIN_SAMPLE_COUNT 32U
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint16_t s_logical_to_channel[3] = {
    TIM_Channel_1, /* 逻辑 A -> INHA */
    TIM_Channel_2, /* 逻辑 B -> INHB */
    TIM_Channel_3, /* 逻辑 C -> INHC */
};
/* Private function prototypes -----------------------------------------------*/
static void Hal_Motor_BldcConfigurePhase(uint16_t channel, hal_motor_bldc_phase_mode_t mode);
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 配置 BM75/HD75 的 BLDC 模拟采样路径。
 * @detail: 通用 PWM 启动会为 FOC 接通相电压滤波；六步反电动势检测需要立即断开滤波电容。
 * @return {void} 无
 * @author: lkc
 */
void Hal_Motor_BldcPrepare(void)
{
    PHASE_FILTER_OFF();
}

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
 * @detail: 六步使用原理图 INHA/B/C 与 TIM1 CH1/2/3 的直接映射；通过 COM 事件同步模式更新。
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

    /*
     * TIM2 与 TIM1 在 APB1/APB2 上分别以 84/168MHz 计数，并在 TIM1
     * 每次更新时复位。TIM2 CCR2=pulse/4 对应 TIM1 PWM 导通脉冲的中点。
     * 停机时恢复 FOC 原有固定采样点，避免 BLDC 配置泄漏到 FOC。
     */
    if (pulse == 0U) {
        TIM2->CCR2 = HAL_MOTOR_BLDC_FOC_SAMPLE_COUNT;
    } else {
        uint32_t sample_count = pulse / 4U;

        if (sample_count < HAL_MOTOR_BLDC_MIN_SAMPLE_COUNT) {
            sample_count = HAL_MOTOR_BLDC_MIN_SAMPLE_COUNT;
        }
        TIM2->CCR2 = sample_count;
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

/**
 * @description: 判断当前 ADC 帧是否位于 TIM1 向上计数侧。
 * @detail: TIM1 中心对齐运行，只接受底部更新后的向上计数侧；采样时刻由占空比动态放在导通脉冲中点。
 * @return {bool} 向上计数侧返回 true
 * @author: lkc
 */
bool Hal_Motor_BldcBemfSampleReady(void)
{
    return (TIM1->CR1 & TIM_CR1_DIR) == 0U;
}

/**
 * @description: 读取当前 BLDC 相电压 ADC 触发位置。
 * @return {uint32_t} TIM2 CCR2 计数值
 * @author: lkc
 */
uint32_t Hal_Motor_BldcGetSampleCount(void)
{
    return TIM2->CCR2;
}
#ifdef __cplusplus
}
#endif
