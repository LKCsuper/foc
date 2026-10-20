/*
 * @Description: FOC 中断集成：采样、保护、分频、控制调用与 PWM/耗时
 * @Author: lkc
 *
 * 学习要点：
 *   1) 本文件是平台胶水，不承载对齐/斜坡/电流 PI 算法细节。
 *   2) 算法在 motor_control / motor_current / 启动与无感模块。
 *   3) 分频没到的 ADC 回调直接返回；到点的那一拍才采样、保护和 Motor_Control_Step。
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
/**
 * @description: 停机时抓取半周期回调寄存器，供连续两帧配对。
 * @detail: running 为 true 时只停止采集并返回，不阻止后续控制。
 *          停机时记录 DMA 剩余计数、TIM1 计数、方向、比较值和三相电流 ADC。
 *          这些是回调时刻的寄存器，不是 ADC 触发锁存值。
 *          reconstructed 的 bit0～bit3 分别标记读取期间 ADC、计数方向、DMA 剩余计数、PWM 序号发生变化。
 *          PairFrame 返回 true 时，同拍推入观测器的 α/β 电压 (V)。
 * @param {bool} running 开环已使能时为 true
 * @return {void} 无
 */
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

/**
 * @description: 半周期诊断入口，集中判断模式与采集开关。
 * @detail: 仅在半周期配对模式且采集仍有效时抓取一帧。
 * @param {bool} running 开环已使能时为 true，传给抓取函数
 * @return {void} 无
 */
static void Motor_DmaInt_DiagnosticStep(bool running)
{
    if (Motor_Timing_PairHalfMode() && Motor_Timing_PairActive()) {
        Motor_DmaInt_CaptureHalf(running);
    }
}

/**
 * @description: 校准占用本次中断时执行检测步进。
 * @detail: 返回 true 表示本次中断已由校准占用，后续控制路径不得继续执行。
 * @return {bool} 校准进行中返回 true
 */
static bool Motor_DmaInt_HandleDetect(void)
{
    if (!Motor_Detect_IsActive()) return false;
    Motor_DetectIsr_Step(&motor, (TIM1->CR1 & TIM_CR1_DIR) != 0U);
    return true;
}

/**
 * @description: BLDC 模式占用本次中断时执行六步控制。
 * @detail: 返回 true 表示已交给 BLDC，同一拍不再执行 FOC。
 * @return {bool} 当前为 BLDC 模式返回 true
 */
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

    Motor_DmaInt_DiagnosticStep(motor.ol_enable);
    Motor_FocIsr_Step(&motor);

    return;
}

#ifdef __cplusplus
}
#endif
