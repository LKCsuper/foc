/*
 * @Description: 通用速度 PI 外环接口（支持状态实例；默认单实例包装保留）
 * @Author: lkc
 *
 * 学习要点：速度环输出是 Iq 目标，不是直接 PWM；反馈无效时不得闭环。
 * State API 供主机测试/将来多实例；无参包装走 s_speed 默认实例。
 * 速度 PI 由 foc pid 任务调用 Run；ISR 只 SetRunAllowed/Reset，并读 Iq。
 */
#ifndef MOTOR_SPEED_H
#define MOTOR_SPEED_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "motor_param.h"
#include "motor_speed_core.h"
/* Private define ------------------------------------------------------------*/
/* VESC s_pid_ramp_erpms_s：速度环给定按电转速爬，机械 RPM/s = 本值 / 极对数 */
#define MOTOR_SPEED_RAMP_ERPMS_S 25000.0f
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 运行指定状态对象的速度 PI，输出 Iq 目标
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @param {MOTOR_PARAM_S} *motor 配置与目标来源，不可为空
 * @param {float} dt 控制周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_RunState(motor_speed_state_t *state, MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 上电初始化默认单实例速度环
 * @return {*}
 * @author: lkc
 */
void Motor_Speed_Init(void);

/**
 * @description: 更新默认实例的真实机械转速反馈
 * @param {float} rpm    机械转速 (RPM)，反转为负数
 * @param {bool} valid   true=反馈可信；false=当前不能用于速度闭环
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedFeedback_Set(float rpm, bool valid);

/**
 * @description: 运行默认实例速度 PI 外环
 * @param {MOTOR_PARAM_S} *motor 电机实时参数
 * @param {float} dt              控制周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_Run(MOTOR_PARAM_S *motor, float dt);
/** 已按配置周期调度的任务入口，保留运行门控但不重复分频；dt单位s。 */
void Motor_SpeedLoop_RunScheduled(MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 允许或禁止默认实例速度 PI（由 ISR 在接管门槛处设置）
 * @detail: allowed=false 时同时复位 PI，避免任务在启动 Iq 阶段后台积分。
 * @param {bool} allowed true=foc pid 任务可运行 PI
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_SetRunAllowed(bool allowed);

/**
 * @description: 清除默认实例速度 PI 状态
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_Reset(void);

/**
 * @description: 用当前等效 Iq 预置默认实例速度 PI
 * @param {float} iq  预置 Iq 目标 (A)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_PrimeIq(float iq);

/**
 * @description: 读取默认实例当前 Iq 目标
 * @return {float} Iq 目标 (A)
 * @author: lkc
 */
float Motor_SpeedLoop_GetIqTarget(void);

/**
 * @description: 读取默认实例实测机械转速
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetMeasuredRpm(void);

/**
 * @description: 读取默认实例速度误差
 * @return {float} 速度误差 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetError(void);

/**
 * @description: 查询默认实例速度反馈是否可信
 * @return {bool} true=反馈可信
 * @author: lkc
 */
bool Motor_SpeedLoop_IsFeedbackValid(void);
/* 只读查询ISR发布的速度PI运行门控。 */
bool Motor_SpeedLoop_IsRunAllowed(void);

/**
 * @description: 读取最近一次速度 PI 使用的同一帧目标、反馈、误差和 Iq。
 * @param {motor_speed_snapshot_t *} snapshot  输出帧，不可为空
 * @return {bool} true=已写出；snapshot 为空返回 false
 * @author: lkc
 */
bool Motor_SpeedLoop_GetSnapshot(motor_speed_snapshot_t *snapshot);

/**
 * @description: 读取默认实例PI更新序号，供任务在调用前后对比。
 * @detail: 仅任务诊断使用，不构成多字段一致快照；复位仍沿用现有序号语义。
 * @return {uint32_t} PI更新计数，可回绕
 */
uint32_t Motor_SpeedLoop_GetUpdateCount(void);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif
