/*
 * @Description: 正常 FOC 中断路径与运行状态
 * @Author: lkc
 */
#ifndef MOTOR_FOC_ISR_H
#define MOTOR_FOC_ISR_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "motor_param.h"
#include "motor_timing.h"
/** 读取默认实例完整诊断字段；不改控制/PWM。motor_param/sample 必须非空，否则断言停住。 */
void Motor_FocIsr_CaptureTraceSample(const MOTOR_PARAM_S *motor_param, float vd, float vq,
                                    motor_trace_sample_t *sample);
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 推进一次正常 FOC ADC 中断路径，并按约 100 Hz 发布状态快照。
 * @detail: 在 ADC-DMA 中断调用，正常路径不阻塞；motor_param 为空表示调用错误，触发断言停住。
 * @param {MOTOR_PARAM_S *} motor_param 电机采样与保护对象，不可为空
 * @return {void} 无
 */
void Motor_FocIsr_Step(MOTOR_PARAM_S *motor_param);

/**
 * @description: 查询本拍是否到达 FOC 控制分频点。
 * @detail: 与快速 V/F 共用同一分频器，保持原有分频相位。在 ADC 中断调用。
 * @return {bool} 本拍应执行控制返回 true
 */
bool Motor_FocIsr_ShouldRun(void);

/**
 * @description: 查询开环对齐是否仍在进行。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {bool} 对齐阶段未结束返回 true
 */
bool Motor_OpenLoop_IsAligning(void);

/**
 * @description: 读取当前对齐已用时间。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {float} 对齐已用时间 (s)
 */
float Motor_OpenLoop_GetAlignElapsed(void);

/**
 * @description: 查询 HFI 启动是否仍在运行。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {bool} HFI 启动进行中返回 true
 */
bool Motor_HfiStart_IsRunning(void);

/**
 * @description: 查询最近一次 HFI 启动是否成功。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {bool} 成功返回 true
 */
bool Motor_HfiStart_Succeeded(void);

/**
 * @description: 读取 HFI 启动给出的电角度。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {float} 电角度 (rad)
 */
float Motor_HfiStart_GetAngle(void);

/**
 * @description: 读取 HFI 启动置信度。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {float} 置信度，无量纲
 */
float Motor_HfiStart_GetConfidence(void);

/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
