/*
 * @Description: 编码器多圈位置环（位置 P 串速度 PI，输出电压）
 * @Author: lkc
 *
 * 位置相对上电或 pos zero 时的编码器累计计数。位置误差换成转速后，由本模块的速度 PI
 * 输出电压。ol_target_rpm 在位置环使能时保持 0，避免电压速度模式的起动前馈介入。
 */
#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "motor_config.h"
#include "motor_position_core.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 用当前累计计数作为零点，并装入默认比例与转速上限。
 * @return {void} 无
 * @author: lkc
 */
#if MOTOR_CFG_POSITION
void Motor_Position_Init(void);

/**
 * @description: 把当前位置记为零，并清掉目标与速度指令。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_Zero(void);

/**
 * @description: 开关位置环。
 * @detail: 打开前要求编码器零点有效且无信号故障。关闭时速度目标清零。
 * @param {bool} enable  true=使能
 * @return {bool} true=已设置；false=编码器未就绪
 * @author: lkc
 */
bool Motor_Position_Enable(bool enable);

/**
 * @description: 设置绝对目标位置。
 * @param {float} target_deg  相对零点的目标 (deg)，须为有限值
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Position_SetTargetDeg(float target_deg);

/**
 * @description: 在当前目标上叠加一段位移。
 * @param {float} delta_deg  增量 (deg)，须为有限值
 * @return {bool} true=相加后目标仍为有限值
 * @author: lkc
 */
bool Motor_Position_MoveDeg(float delta_deg);

/**
 * @description: 设置位置比例。
 * @param {float} kp_rpm_per_deg  比例 (RPM/deg)，范围 (0, 100]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Position_SetKp(float kp_rpm_per_deg);

/**
 * @description: 设置位置环给出的速度上限。
 * @param {float} max_rpm  限幅 (RPM)，范围 [1, 当前电机允许上限]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Position_SetMaxRpm(float max_rpm);

/**
 * @description: 推进一拍位置环，速度 PI 的电压写入 position_hold_v。
 * @detail: 未使能、电机未启动或编码器故障时退出位置保持并清积分。
 *          位置误差始终换成转速，再由速度 PI 出电压。到达目标后积分保留。
 * @param {float} dt  调用周期 (s)。非正或大于 0.05 s 时本拍不积分
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_Run(float dt);

/**
 * @description: 获取位置环状态，并用当前累计计数刷新返回快照中的位置、误差和速度目标。
 * @detail: 只刷新快照，不推进速度 PI，也不修改保持电压和积分。
 * @param {motor_position_state_t} *state  输出，为空则忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_GetState(motor_position_state_t *state);

/**
 * @description: 查询位置环是否使能。
 * @return {bool} true=已使能
 * @author: lkc
 */
bool Motor_Position_IsEnabled(void);
#else
static inline void Motor_Position_Init(void) {}
static inline void Motor_Position_Zero(void) {}
static inline bool Motor_Position_Enable(bool enable)
{
    (void)enable;
    return false;
}
static inline bool Motor_Position_SetTargetDeg(float target_deg)
{
    (void)target_deg;
    return false;
}
static inline bool Motor_Position_MoveDeg(float delta_deg)
{
    (void)delta_deg;
    return false;
}
static inline bool Motor_Position_SetKp(float kp_rpm_per_deg)
{
    (void)kp_rpm_per_deg;
    return false;
}
static inline bool Motor_Position_SetMaxRpm(float max_rpm)
{
    (void)max_rpm;
    return false;
}
static inline void Motor_Position_Run(float dt) { (void)dt; }
static inline void Motor_Position_GetState(motor_position_state_t *state) { (void)state; }
static inline bool Motor_Position_IsEnabled(void) { return false; }
#endif
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
