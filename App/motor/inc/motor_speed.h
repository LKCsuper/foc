/*
 * @Description: 通用速度 PI 外环接口
 * @Author: lkc
 */
#ifndef _MOTOR_SPEED_H
#define _MOTOR_SPEED_H

#include <stdbool.h>
#include "motor_param.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @description: 上电初始化速度环模块。
 * @detail: 清除速度反馈、PI 积分和输出。
 * @return {*}
 * @author: lkc
 */
void Motor_Speed_Init(void);

/**
 * @description: 更新速度环的真实机械转速反馈。
 * @param {float} rpm    机械转速 (RPM)，反转为负数
 * @param {bool} valid   true=反馈可信；false=当前不能用于速度闭环
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedFeedback_Set(float rpm, bool valid);

/**
 * @description: 运行速度 PI 外环，计算电流内环的 Iq 目标。
 * @detail: 函数内部按照 motor->speed_loop_hz 自动分频。
 * @param {MOTOR_PARAM_S} *motor  电机实时参数
 * @param {float} dt              控制周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_Run(MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 清除速度 PI 状态（分频计时/积分/误差/输出目标）。
 * @detail: 不清除反馈源最近一次提供的速度。
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_Reset(void);

/**
 * @description: 用当前等效 Iq 预置速度 PI 积分与输出。
 * @detail: 无感开环切闭环时使用，减小接管瞬间的电流阶跃。
 * @param {float} iq  预置 Iq 目标 (A)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_PrimeIq(float iq);

/**
 * @description: 读取速度 PI 输出的 Iq 目标。
 * @return {float} Iq 目标 (A)
 * @author: lkc
 */
float Motor_SpeedLoop_GetIqTarget(void);

/**
 * @description: 读取最近一次有效的实测机械转速。
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetMeasuredRpm(void);

/**
 * @description: 读取最近一次速度环误差。
 * @return {float} 速度误差 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetError(void);

/**
 * @description: 查询当前是否有可信的机械转速反馈。
 * @return {bool} true=反馈有效
 * @author: lkc
 */
bool Motor_SpeedLoop_IsFeedbackValid(void);

#ifdef __cplusplus
}
#endif

#endif
