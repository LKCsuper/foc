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
/* Private define ------------------------------------------------------------*/
/* VESC s_pid_ramp_erpms_s：速度环给定按电转速爬，机械 RPM/s = 本值 / 极对数 */
#define MOTOR_SPEED_RAMP_ERPMS_S 25000.0f
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 速度环运行时状态（不持久化）
 */
typedef struct {
    float measured_rpm; /* 反馈机械转速 RPM */
    bool feedback_valid;
    float loop_time; /* 分频累计时间 s */
    float integral;  /* PI 积分 A */
    float error;     /* 速度误差 RPM */
    float iq_target; /* Iq 目标 A */
    bool zero_brake_active;    /* 零速制动已锁定进入时的方向 */
    bool zero_brake_released;  /* 已越过零点，在新的非零指令前保持释放 */
    int8_t zero_brake_direction;
} motor_speed_state_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化指定速度环状态对象，清零反馈、PI 与分频时间
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_Speed_InitState(motor_speed_state_t *state);

/**
 * @description: 更新指定状态对象的机械转速反馈
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @param {float} rpm 机械转速 (RPM)
 * @param {bool} valid true=反馈可信
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedFeedback_SetState(motor_speed_state_t *state, float rpm, bool valid);

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
 * @description: 清除指定状态对象的 PI（保留反馈转速）
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_ResetState(motor_speed_state_t *state);

/**
 * @description: 预置指定状态对象的 Iq 积分与输出
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @param {float} iq 预置 Iq (A)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_PrimeIqState(motor_speed_state_t *state, float iq);

/**
 * @description: 读取指定状态当前 Iq 目标
 * @param {const motor_speed_state_t} *state  状态对象；NULL 返回 0
 * @return {float} Iq 目标 (A)
 * @author: lkc
 */
float Motor_SpeedLoop_GetIqTargetState(const motor_speed_state_t *state);

/**
 * @description: 读取指定状态实测机械转速
 * @param {const motor_speed_state_t} *state  状态对象；NULL 返回 0
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetMeasuredRpmState(const motor_speed_state_t *state);

/**
 * @description: 读取指定状态速度误差（目标−实测）
 * @param {const motor_speed_state_t} *state  状态对象；NULL 返回 0
 * @return {float} 速度误差 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetErrorState(const motor_speed_state_t *state);

/**
 * @description: 查询指定状态速度反馈是否可信
 * @param {const motor_speed_state_t} *state  状态对象；NULL 返回 false
 * @return {bool} true=反馈可信，可用于闭环
 * @author: lkc
 */
bool Motor_SpeedLoop_IsFeedbackValidState(const motor_speed_state_t *state);

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
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif
