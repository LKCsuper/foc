/*
 * @Description: 一维梯形运动轨迹；按剩余制动距离提前减速
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stddef.h>
#include "motor_trajectory.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static float trajectory_clamp(float value, float limit);
static bool trajectory_config_valid(const motor_trajectory_config_t *config);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把数值钳到对称区间。
 * @param {float} value  待钳位值
 * @param {float} limit  对称限幅，须为非负
 * @return {float} 钳位结果，范围 [-limit, limit]
 */
static float trajectory_clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/**
 * @description: 检查轨迹限制的有限性和符号。
 * @param {const motor_trajectory_config_t *} config  本拍限制
 * @return {bool} 可用于本拍计算返回 true
 */
static bool trajectory_config_valid(const motor_trajectory_config_t *config)
{
    return config != NULL && isfinite(config->velocity_limit) && isfinite(config->acceleration_limit) &&
           isfinite(config->deceleration_limit) && isfinite(config->position_tolerance) &&
           isfinite(config->velocity_tolerance) && isfinite(config->maximum_dt_s) &&
           config->velocity_limit > 0.0f && config->acceleration_limit > 0.0f &&
           config->deceleration_limit > 0.0f && config->position_tolerance >= 0.0f &&
           config->velocity_tolerance >= 0.0f && config->maximum_dt_s > 0.0f;
}

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
bool Motor_Trajectory_Init(motor_trajectory_t *trajectory, float position, float velocity)
{
    if (trajectory == NULL || !isfinite(position) || !isfinite(velocity)) return false;
    trajectory->position = position;
    trajectory->velocity = velocity;
    trajectory->acceleration = 0.0f;
    trajectory->goal = position;
    trajectory->active = false;
    return true;
}

/**
 * @description: 更换目标位置并标记轨迹尚未到位。
 * @detail: 不改当前规划位置和速度。trajectory 为空或目标非有限时失败。
 * @param {motor_trajectory_t *} trajectory  已初始化的轨迹状态，不可为空
 * @param {float} goal                       目标位置 (位置单位)
 * @return {bool} 目标已写入返回 true
 * @author: lkc
 */
bool Motor_Trajectory_SetGoal(motor_trajectory_t *trajectory, float goal)
{
    if (trajectory == NULL || !isfinite(goal)) return false;
    trajectory->goal = goal;
    trajectory->active = true;
    return true;
}

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
bool Motor_Trajectory_Step(motor_trajectory_t *trajectory, const motor_trajectory_config_t *config, float dt_s)
{
    float error;
    float direction;
    float brake_velocity;
    float desired_velocity;
    float velocity_step;
    float old_velocity;
    float next_position;
    float rate_limit;

    if (trajectory == NULL || !trajectory_config_valid(config) || !isfinite(dt_s) ||
        !isfinite(trajectory->position) || !isfinite(trajectory->velocity) || !isfinite(trajectory->goal)) {
        return false;
    }
    if (!(dt_s > 0.0f) || dt_s > config->maximum_dt_s) {
        trajectory->acceleration = 0.0f;
        return true;
    }
    error = trajectory->goal - trajectory->position;
    if (fabsf(error) <= config->position_tolerance &&
        fabsf(trajectory->velocity) <= config->velocity_tolerance) {
        trajectory->position = trajectory->goal;
        trajectory->velocity = 0.0f;
        trajectory->acceleration = 0.0f;
        trajectory->active = false;
        return true;
    }

    direction = error >= 0.0f ? 1.0f : -1.0f;
    brake_velocity = sqrtf(2.0f * config->deceleration_limit * fabsf(error));
    desired_velocity = direction * fminf(config->velocity_limit, brake_velocity);
    old_velocity = trajectory->velocity;
    rate_limit = (old_velocity * desired_velocity < 0.0f || fabsf(desired_velocity) < fabsf(old_velocity))
                     ? config->deceleration_limit
                     : config->acceleration_limit;
    velocity_step = trajectory_clamp(desired_velocity - old_velocity, rate_limit * dt_s);
    trajectory->velocity = old_velocity + velocity_step;
    trajectory->acceleration = velocity_step / dt_s;
    next_position = trajectory->position + 0.5f * (old_velocity + trajectory->velocity) * dt_s;
    if ((trajectory->goal - trajectory->position) * (trajectory->goal - next_position) <= 0.0f) {
        trajectory->position = trajectory->goal;
        trajectory->velocity = 0.0f;
        trajectory->acceleration = 0.0f;
        trajectory->active = false;
    } else {
        trajectory->position = next_position;
    }
    return true;
}
#ifdef __cplusplus
}
#endif
