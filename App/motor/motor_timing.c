/*
 * @Description: ADC/控制周期计算与 ADC-DMA 中断耗时统计
 * @Author: lkc
 */
#include <stddef.h>
#include "motor_timing.h"
#include "motor_it.h"
#include "hal_motor.h"

static volatile float s_runtime_average_us;
static volatile float s_runtime_maximum_us;

/**
 * @description: 更新本拍 ADC-DMA 中断耗时统计（平均与最大值）。
 * @detail: 超过 1000 us 的样本丢弃，避免计时回绕或异常拉高最大耗时。平均系数 0.02。
 * @param {uint32_t} start_time_us 本拍开始时的时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_RecordRuntime(uint32_t start_time_us)
{
    uint32_t elapsed_us = Hal_Motor_TimeUs() - start_time_us;
    float elapsed = (float)elapsed_us;
    if (elapsed_us > 1000U) {
        return;
    }
    if (s_runtime_average_us <= 0.0f) {
        s_runtime_average_us = elapsed;
    } else {
        s_runtime_average_us += 0.02f * (elapsed - s_runtime_average_us);
    }
    if (elapsed > s_runtime_maximum_us) {
        s_runtime_maximum_us = elapsed;
    }
}

/**
 * @description: 计算 ADC/保护用采样周期。
 * @detail: 优先用 Hal_Motor_AdcSampleRateHz()；无效时回退 motor->ol_foc_freq，并写回该字段。
 * @param {MOTOR_PARAM_S *} motor 电机对象，为空时返回 1 s
 * @return {float} 采样周期 (s)，即 1/ADC 触发频率
 * @author: lkc
 */
float Motor_Timing_GetAdcDt(MOTOR_PARAM_S *motor)
{
    float adc_hz;
    if (motor == NULL) {
        return 1.0f;
    }
    adc_hz = Hal_Motor_AdcSampleRateHz();
    if (adc_hz > 1.0f) {
        motor->ol_foc_freq = adc_hz;
    } else {
        adc_hz = motor->ol_foc_freq;
        if (adc_hz <= 1.0f) {
            adc_hz = 1.0f;
            motor->ol_foc_freq = adc_hz;
        }
    }
    return 1.0f / adc_hz;
}

/**
 * @description: 计算 FOC 控制环周期（已乘分频）。
 * @detail: dt = ADC 周期 × FOC_CONTROL_LOOP_FREQ_DIVIDER，供斜坡、PI 和观测器使用。
 * @param {MOTOR_PARAM_S *} motor 电机对象
 * @return {float} 控制周期 (s)
 * @author: lkc
 */
float Motor_Timing_GetControlDt(MOTOR_PARAM_S *motor)
{
    return Motor_Timing_GetAdcDt(motor) * (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
}

/**
 * @description: 返回 ADC-DMA 中断的平均执行时间。
 * @return {float} 平均耗时 (us)
 * @author: lkc
 */
float Motor_DmaInt_GetAverageRuntimeUs(void) { return s_runtime_average_us; }

/**
 * @description: 返回 ADC-DMA 中断的历史最长执行时间。
 * @return {float} 最大耗时 (us)
 * @author: lkc
 */
float Motor_DmaInt_GetMaximumRuntimeUs(void) { return s_runtime_maximum_us; }

/**
 * @description: 返回实际 ADC-DMA 触发频率。
 * @detail: HAL 频率无效时回退 motor.ol_foc_freq。
 * @return {float} ADC 采样频率 (Hz)
 * @author: lkc
 */
float Motor_DmaInt_GetAdcFrequencyHz(void)
{
    float adc_hz = Hal_Motor_AdcSampleRateHz();
    return (adc_hz > 0.0f) ? adc_hz : motor.ol_foc_freq;
}

/**
 * @description: 返回 FOC 控制环实际运行频率。
 * @detail: 控制频率 = ADC 频率 / FOC_CONTROL_LOOP_FREQ_DIVIDER。
 * @return {float} 控制环频率 (Hz)
 * @author: lkc
 */
float Motor_DmaInt_GetControlFrequencyHz(void)
{
    return Motor_DmaInt_GetAdcFrequencyHz() / (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
}
