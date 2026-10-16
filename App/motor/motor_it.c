/*
 * @Description: FOC 中断集成：采样、保护、分频、控制调用与 PWM/耗时
 * @Author: lkc
 *
 * 学习要点：
 *   1) 本文件是平台胶水，不承载对齐/斜坡/电流 PI 算法细节。
 *   2) 算法在 motor_control / motor_current / 启动与无感模块。
 *   3) 分频前：Hal 采样帧 → Sense 换算 + 软件保护；分频后：Motor_Control_Step → 应用 PWM。
 *   4) 速度 PI 不在本中断；由 Motor_Task foc pid 写出 Iq，本拍只读。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"
#include "motor_param.h"
#include "motor_it.h"
#include "motor_fast_vf.h"
#include "motor_foc_isr.h"
#include "motor_detect.h"
#include "motor_detect_isr.h"
#include "motor_bldc.h"
#include "motor_timing.h"
#include "motor_sensorless.h"
#include "hal_motor.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void Motor_DmaInt_CaptureHalf(bool running)
{
    motor_pair_frame_t frame;
    uint32_t remaining;
    uint16_t cr1;
    unsigned index;
    if (running) {
        Motor_Timing_HandoffStop();
        return;
    }
    memset(&frame, 0, sizeof(frame));
    remaining = DMA2_Stream4->NDTR;
    cr1 = (uint16_t)TIM1->CR1;
    frame.entry_us = Hal_Motor_TimeUs();
    frame.callback_count = (uint16_t)TIM1->CNT;
    frame.callback_cr1 = cr1;
    frame.pwm_sequence = Hal_Motor_PwmDiagnosticSequence();
    frame.period = (uint16_t)TIM1->ARR;
    frame.ccr[0] = (uint16_t)TIM1->CCR1;
    frame.ccr[1] = (uint16_t)TIM1->CCR2;
    frame.ccr[2] = (uint16_t)TIM1->CCR3;
    for (index = 0U; index < 3U; index++)
        frame.adc[index] = Hal_Motor_CurrentAdc((uint8_t)index);
    frame.reconstructed = 0U;
    for (index = 0U; index < 3U; index++) {
        if (frame.adc[index] != Hal_Motor_CurrentAdc((uint8_t)index))
            frame.reconstructed |= 1U;
    }
    if (((uint16_t)TIM1->CR1 ^ cr1) & TIM_CR1_DIR)
        frame.reconstructed |= 2U;
    if (DMA2_Stream4->NDTR != remaining)
        frame.reconstructed |= 4U;
    if (Hal_Motor_PwmDiagnosticSequence() != frame.pwm_sequence)
        frame.reconstructed |= 8U;
    if (Motor_Timing_PairFrame(&frame)) {
        motor_sensorless_light_snapshot_t observer;
        Motor_Sensorless_GetLightSnapshot(&observer);
        Motor_Timing_PairPush(observer.v_alpha, observer.v_beta);
    }
}

/** 半周期诊断入口：模式与采集开关的判断集中在此。 */
static void Motor_DmaInt_DiagnosticStep(bool running)
{
    if (Motor_Timing_PairHalfMode() && Motor_Timing_PairActive()) {
        Motor_DmaInt_CaptureHalf(running);
    }
}

/* 返回 true 表示校准已占用本次中断，后续控制路径不得继续执行。 */
static bool Motor_DmaInt_HandleDetect(void)
{
    if (!Motor_Detect_IsActive()) return false;
    Motor_DetectIsr_Step(&motor, (TIM1->CR1 & TIM_CR1_DIR) != 0U);
    return true;
}

/* 返回 true 表示已交给 BLDC 处理，避免同一拍再执行 FOC。 */
static bool Motor_DmaInt_HandleBldc(void)
{
    if (!Motor_Bldc_IsSelected()) return false;
    Motor_Bldc_RunIsr(&motor);
    return true;
}

/**
 * @description: ADC-DMA 传输完成中断回调，FOC 控制链驱动入口
 * @detail: 按校准、BLDC 与正常 FOC 的顺序分派，本函数不承载控制算法。
 * @param {void *} p 回调用户参数，未使用
 * @param {uint32_t} flags DMA 事件标志，未使用
 * @return {void} 无
 */
void Motor_DmaInt(void *p, uint32_t flags)
{
    (void)p;
    (void)flags;

    if (Motor_DmaInt_HandleDetect()) return;
    if (Motor_DmaInt_HandleBldc()) return;

    Motor_DmaInt_DiagnosticStep(motor.ol_enable); // 诊断函数
    Motor_FocIsr_Step(&motor);                  // foc中断

    return;
}

#ifdef __cplusplus
}
#endif
