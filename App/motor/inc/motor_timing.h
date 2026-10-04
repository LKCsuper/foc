/*
 * @Description: 电机控制时间基准与 ISR 耗时统计
 * @Author: lkc
 */
#ifndef MOTOR_TIMING_H
#define MOTOR_TIMING_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* ADC 每个中心对齐 PWM 周期采样两次，FOC 默认每个完整 PWM 周期运行一次。 */
#ifndef FOC_CONTROL_LOOP_FREQ_DIVIDER
#define FOC_CONTROL_LOOP_FREQ_DIVIDER 2
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 更新本拍 ADC-DMA 中断耗时统计（平均与最大值）。
 * @detail: 超过 1000 us 的样本丢弃，避免计时回绕或异常拉高最大耗时。在中断末尾调用，不阻塞。
 * @param {uint32_t} start_time_us 本拍开始时的时间戳 (us)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Timing_RecordRuntime(uint32_t start_time_us);

/**
 * @description: 计算 ADC/保护用采样周期。
 * @detail: 优先用 Hal_Motor_AdcSampleRateHz()；无效时回退 motor->ol_foc_freq，并写回该字段。
 * @param {MOTOR_PARAM_S *} motor 电机对象，为空时返回 1 s
 * @return {float} 采样周期 (s)，即 1/ADC 触发频率
 * @author: lkc
 */
float Motor_Timing_GetAdcDt(MOTOR_PARAM_S *motor);

/**
 * @description: 计算 FOC 控制环周期（已乘分频）。
 * @detail: dt = ADC 周期 × FOC_CONTROL_LOOP_FREQ_DIVIDER，供斜坡、PI 和观测器使用。
 * @param {MOTOR_PARAM_S *} motor 电机对象
 * @return {float} 控制周期 (s)
 * @author: lkc
 */
float Motor_Timing_GetControlDt(MOTOR_PARAM_S *motor);

/**
 * @description: 记录一次实际执行的 FOC 控制拍位于 PWM 向上或向下计数侧。
 * @detail: 在控制拍调用，只累加计数，不阻塞。
 * @param {bool} upward true=PWM 向上计数侧
 * @return {void} 无
 */
void Motor_Timing_RecordSampleDirection(bool upward);

/**
 * @description: 读取 FOC 控制拍落在 PWM 向上/向下计数侧的累计次数。
 * @detail: 输出指针可为空，空指针对应的一侧不写出。可在任务中调用。
 * @param {uint32_t *} up 向上计数侧累计次数，可为 NULL
 * @param {uint32_t *} down 向下计数侧累计次数，可为 NULL
 * @return {void} 无
 */
void Motor_DmaInt_GetSampleDirectionCounts(uint32_t *up, uint32_t *down);

/**
 * @description: 返回 ADC-DMA 中断的平均执行时间。
 * @return {float} 平均耗时 (us)
 */
float Motor_DmaInt_GetAverageRuntimeUs(void);

/**
 * @description: 返回 ADC-DMA 中断的历史最长执行时间。
 * @detail: 超过 1000 us 的样本不参与统计。
 * @return {float} 最大耗时 (us)
 */
float Motor_DmaInt_GetMaximumRuntimeUs(void);

/**
 * @description: 返回实际 ADC-DMA 触发频率。
 * @detail: HAL 频率无效时回退 motor.ol_foc_freq。
 * @return {float} ADC 采样频率 (Hz)
 */
float Motor_DmaInt_GetAdcFrequencyHz(void);

/**
 * @description: 返回 FOC 控制环实际运行频率。
 * @detail: 控制频率 = ADC 频率 / FOC_CONTROL_LOOP_FREQ_DIVIDER。
 * @return {float} 控制环频率 (Hz)
 */
float Motor_DmaInt_GetControlFrequencyHz(void);

/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif
