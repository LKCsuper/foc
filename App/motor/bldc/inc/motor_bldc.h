/*
 * @Description: 默认 BLDC 六步运行模块公共接口
 * @Author: lkc
 */
#ifndef MOTOR_BLDC_H
#define MOTOR_BLDC_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "motor_bldc_status.h"
#include "motor_config.h"
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#if MOTOR_CFG_BLDC

/**
 * @description: 以反电势闭环方式启动默认 BLDC 实例。
 * @detail: 开环 FOC 已使能、故障已锁存、方向非法，或占空比不在 0.08～0.10 时拒绝启动。
 *          自动模式还要求 BEMF 采样侧为 PWM 向上计数。成功后占用主 PWM 出口。
 *          由任务或 Shell 调用，不在控制中断里调用。
 * @param {motor_bldc_direction_t} direction 换相方向
 * @param {float} duty 目标占空比，无量纲，自动模式允许 0.08～0.10
 * @return {bool} 启动成功返回 true
 */
bool Motor_Bldc_Start(motor_bldc_direction_t direction, float duty);

/**
 * @description: 以固定扇区周期的开环六步启动默认 BLDC 实例。
 * @detail: 不检查自动模式占空比窗口和采样侧。占空比须大于 0 且不大于 0.25。
 *          由任务或 Shell 调用，不在控制中断里调用。
 * @param {motor_bldc_direction_t} direction 换相方向
 * @param {float} duty 目标占空比，无量纲，允许范围 (0, 0.25]
 * @param {float} sector_period_s 固定换相周期 (s)
 * @return {bool} 启动成功返回 true
 */
bool Motor_Bldc_StartOpenLoop(motor_bldc_direction_t direction, float duty, float sector_period_s);

/**
 * @description: 选择 BEMF 采样落在 PWM 向上或向下计数侧。
 * @detail: 六步正在运行时拒绝修改。停机后由任务或 Shell 调用。
 * @param {bool} upward true=PWM 向上计数侧
 * @return {bool} 设置成功返回 true
 */
bool Motor_Bldc_SetSampleSide(bool upward);

/**
 * @description: 在停机时写入 BEMF 积分门限和最短换相周期。
 * @detail: 积分门限允许 0.00005～0.010 V·s，最短周期允许 0.0005～0.100 s。
 *          写入后关闭积分自动检测和占空比周期自适应。运行中拒绝修改。
 * @param {float} integral_threshold_vs BEMF 积分换相门限 (V·s)
 * @param {float} minimum_period_s 闭环最短换相周期 (s)
 * @return {bool} 参数合法且当前未运行时返回 true
 */
bool Motor_Bldc_SetTuning(float integral_threshold_vs, float minimum_period_s);

/**
 * @description: 停止六步输出，并继续占用主 PWM 出口。
 * @detail: 关断 PWM，不把控制权交还 FOC。由任务或 Shell 调用。
 * @return {void} 无
 */
void Motor_Bldc_Stop(void);

/**
 * @description: 停止六步输出并退出 BLDC，交还 FOC 主路径。
 * @detail: 由任务或 Shell 调用。退出后 ADC 中断不再进入 BLDC 步进。
 * @return {void} 无
 */
void Motor_Bldc_Exit(void);

/**
 * @description: 查询 BLDC 是否占用主 PWM 出口。
 * @detail: 供 ADC 中断分派读取，调用本身不关中断。
 * @return {bool} 已占用返回 true
 */
bool Motor_Bldc_IsSelected(void);

/**
 * @description: 复制默认 BLDC 实例的只读状态。
 * @detail: 在短临界区内复制，不打印、不阻塞。由任务或 Shell 调用。
 * @param {motor_bldc_status_t *} status 输出快照，不可为空
 * @return {bool} 输出参数有效返回 true
 */
bool Motor_Bldc_GetStatus(motor_bldc_status_t *status);

/**
 * @description: 推进默认 BLDC 实例一个 ADC 中断周期。
 * @detail: 仅供主中断模式分派调用；采样、保护和换相细节由 BLDC ISR 模块处理。
 * @param {MOTOR_PARAM_S *} motor_param 当前电机采样与保护数据，不可为空
 * @return {void} 无
 */
void Motor_Bldc_RunIsr(MOTOR_PARAM_S *motor_param);
#else
/**
 * @description: BLDC 关闭：永不占用 PWM 出口，ADC 中断始终走 FOC。
 * @return {bool} 本实现固定返回 false
 */
static inline bool Motor_Bldc_IsSelected(void) { return false; }
/**
 * @description: 模块关闭时的兼容空实现；不执行对应功能。
 * @param {MOTOR_PARAM_S *} motor_param 兼容参数，本实现不使用
 * @return {void} 无
 */
static inline void Motor_Bldc_RunIsr(MOTOR_PARAM_S *motor_param) { (void)motor_param; }
#endif

/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
