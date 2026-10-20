/*
 * @Description: BLDC 六步 ADC-DMA 中断处理
 * @Author: lkc
 */
#ifndef MOTOR_BLDC_ISR_H
#define MOTOR_BLDC_ISR_H
#include <stdbool.h>
#include "motor_bldc_sensorless.h"
#include "motor_param.h"

/**
 * @description: 执行一拍 BLDC 采样、保护和 BEMF 换相处理。
 * @detail: 在 ADC-DMA 中断调用，不阻塞。保护检查每一帧；BEMF 只消费 sample_upward 选定的 PWM 半周期。
 *          传给无感步进的时间是 2 倍 ADC 周期，对应一个完整 PWM 周期。
 *          FOC 运行量按 A-C-B 存放，这里恢复为原理图 SENS1/2/3 的 A-B-C。实例不可为空，违反约定触发 COMMON_ASSERT。
 * @param {motor_bldc_sensorless_t *} sensorless BLDC 无感实例，不可为空
 * @param {MOTOR_PARAM_S *} motor                电机采样与保护状态，不可为空
 * @param {bool} sample_upward                   true=使用 PWM 向上计数侧 BEMF 样本
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcIsr_Step(motor_bldc_sensorless_t *sensorless, MOTOR_PARAM_S *motor, bool sample_upward);
#endif
