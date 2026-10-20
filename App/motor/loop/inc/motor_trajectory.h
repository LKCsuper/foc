/*
 * @Description: 与硬件无关的一维梯形运动轨迹
 * @Author: lkc
 *
 * 状态由调用者持有。不读取编码器、硬件或全局电机，也不阻塞。
 * 位置、速度、加速度使用同一套位置单位；时间单位是 s。
 */
#ifndef MOTOR_TRAJECTORY_H
#define MOTOR_TRAJECTORY_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 一拍轨迹限制（不持久化，由调用者每拍传入）
 */
typedef struct {
    float velocity_limit;      /* 速度上限 (位置单位/s)，须为正有限值 */
    float acceleration_limit;  /* 加速时的速度变化率 (位置单位/s²)，须为正有限值 */
    float deceleration_limit;  /* 减速或反向时的速度变化率 (位置单位/s²)，须为正有限值 */
    float position_tolerance;  /* 到位位置窗 (位置单位)，须为非负有限值 */
    float velocity_tolerance;  /* 到位速度窗 (位置单位/s)，须为非负有限值 */
    float maximum_dt_s;        /* 允许积分的最大步长 (s)，须为正有限值 */
} motor_trajectory_config_t;

/**
 * @description: 轨迹规划状态（不持久化）
 */
typedef struct {
    float position;      /* 当前规划位置 (位置单位) */
    float velocity;      /* 当前规划速度 (位置单位/s) */
    float acceleration;  /* 上一拍加速度 (位置单位/s²) */
    float goal;          /* 目标位置 (位置单位) */
    bool active;         /* true=尚未进入到位窗 */
} motor_trajectory_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 用当前位置和速度建立一条静止目标的轨迹。
 * @detail: 目标等于当前位置，active 为 false。不访问硬件。
 *          trajectory 为空，或位置、速度非有限时失败，且不改已有状态。
 * @param {motor_trajectory_t *} trajectory  轨迹状态，不可为空
 * @param {float} position                   当前位置 (位置单位)
 * @param {float} velocity                   当前速度 (位置单位/s)
 * @return {bool} 初始化成功返回 true
 * @author: lkc
 */
bool Motor_Trajectory_Init(motor_trajectory_t *trajectory, float position, float velocity);

/**
 * @description: 更换目标位置并标记轨迹尚未到位。
 * @detail: 不改当前规划位置和速度。trajectory 为空或目标非有限时失败。
 * @param {motor_trajectory_t *} trajectory  已初始化的轨迹状态，不可为空
 * @param {float} goal                       目标位置 (位置单位)
 * @return {bool} 目标已写入返回 true
 * @author: lkc
 */
bool Motor_Trajectory_SetGoal(motor_trajectory_t *trajectory, float goal);

/**
 * @description: 按剩余制动距离推进一拍梯形轨迹。
 * @detail: 期望速度不超过速度上限，也不超过 sqrt(2 × 减速度 × 剩余距离)。
 *          需要减速或反向时用减速度限制，否则用加速度限制。本拍会越过目标时收到目标并清速度。
 *          dt 非正或大于 maximum_dt_s 时不积分，加速度写 0，仍返回 true。
 *          状态、配置或 dt 非法时返回 false，且不改轨迹。
 * @param {motor_trajectory_t *} trajectory           轨迹状态，不可为空
 * @param {const motor_trajectory_config_t *} config  本拍限制，不可为空
 * @param {float} dt_s                                本拍时间 (s)
 * @return {bool} 本拍可计算返回 true
 * @author: lkc
 */
bool Motor_Trajectory_Step(motor_trajectory_t *trajectory, const motor_trajectory_config_t *config, float dt_s);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
