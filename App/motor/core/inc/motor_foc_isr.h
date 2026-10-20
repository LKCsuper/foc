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
#include "motor_current.h"

/**
 * @description: 停机时读取或清零默认 ISR 的电流跟踪观察。
 * @detail: 运行中返回 false，且不写出 diag。diag 不可为空，违反约定触发 COMMON_ASSERT。
 *          clear 为真时在拷贝后清零。清零不影响故障或其他记录。不改控制。
 * @param {motor_current_tracking_diag_t *} diag 输出，不可为空
 * @param {bool} clear                           true=读出后清零
 * @return {bool} 已读出返回 true；运行中返回 false
 * @author: lkc
 */
bool Motor_FocIsr_ReadTracking(motor_current_tracking_diag_t *diag, bool clear);

/**
 * @description: 停机时打开或关闭电流跟踪观察。
 * @detail: 默认关闭。运行中返回 false，不改开关。打开时清历史；关闭时清当前累计，保留已记下的事件。
 *          不改电流或保护。
 * @param {bool} enabled true=打开
 * @return {bool} 已写入返回 true；运行中返回 false
 * @author: lkc
 */
bool Motor_FocIsr_EnableTracking(bool enabled);

/**
 * @description: 查询电流跟踪观察是否打开。
 * @detail: 只读开关，不改采集或控制。
 * @return {bool} true=已打开
 * @author: lkc
 */
bool Motor_FocIsr_TrackingIsEnabled(void);

/**
 * @description: 组装一条完整轨迹样本。
 * @detail: 只读现有诊断字段。motor_param 与 sample 不可为空，违反约定触发 COMMON_ASSERT。
 *          间隔统计字段写 0，由 TracePush 覆盖。不提交 PWM，不改控制。
 * @param {const MOTOR_PARAM_S *} motor_param 电机对象，不可为空
 * @param {float} vd                          本拍 d 轴电压 (V)
 * @param {float} vq                          本拍 q 轴电压 (V)
 * @param {motor_trace_sample_t *} sample     输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_FocIsr_CaptureTraceSample(const MOTOR_PARAM_S *motor_param, float vd, float vq,
                                    motor_trace_sample_t *sample);
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 推进一次正常 FOC ADC 中断路径，并按约 100 Hz 发布状态快照。
 * @detail: 在 ADC-DMA 中断调用，正常路径不阻塞。motor_param 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor_param 电机采样与保护对象，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_FocIsr_Step(MOTOR_PARAM_S *motor_param);

/**
 * @description: 查询本拍是否到达 FOC 控制分频点。
 * @detail: 与快速 V/F 共用同一分频器，保持原有分频相位。在 ADC 中断调用。
 * @return {bool} 本拍应执行控制返回 true
 * @author: lkc
 */
bool Motor_FocIsr_ShouldRun(void);

/**
 * @description: 查询开环对齐是否仍在进行。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {bool} 对齐阶段未结束返回 true
 * @author: lkc
 */
bool Motor_OpenLoop_IsAligning(void);

/**
 * @description: 读取当前对齐已用时间。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {float} 对齐已用时间 (s)
 * @author: lkc
 */
float Motor_OpenLoop_GetAlignElapsed(void);

/**
 * @description: 查询 HFI 启动是否仍在运行。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {bool} HFI 启动进行中返回 true
 * @author: lkc
 */
bool Motor_HfiStart_IsRunning(void);

/**
 * @description: 查询最近一次 HFI 启动是否成功。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_HfiStart_Succeeded(void);

/**
 * @description: 读取 HFI 启动给出的电角度。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {float} 电角度 (rad)
 * @author: lkc
 */
float Motor_HfiStart_GetAngle(void);

/**
 * @description: 读取 HFI 启动置信度。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {float} 置信度，无量纲
 * @author: lkc
 */
float Motor_HfiStart_GetConfidence(void);

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 组装一条故障轨迹样本。
 * @detail: 只读组装故障记录样本，不访问 ADC 寄存器、不提交 PWM、不修改控制状态。
 *          正常由 ISR 调用；停机任务可在短临界区用于耗时基准。ADC 与 abc 是当前输入，控制量是此前结果。
 *          motor 与 sample 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {const MOTOR_PARAM_S *} motor         电机对象，不可为空
 * @param {uint32_t} entry_us                   控制入口或基准时间 (us)
 * @param {float} dt                            控制周期 (s)
 * @param {motor_fault_trace_sample_t *} sample 输出样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_FocIsr_CaptureFaultTraceSample(const MOTOR_PARAM_S *motor, uint32_t entry_us, float dt,
                                        motor_fault_trace_sample_t *sample);
#ifdef __cplusplus
}
#endif
#endif
