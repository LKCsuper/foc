/*
 * @Description: 关节阻抗控制纯算法
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "common_assert.h"
#include <stddef.h>
#include <string.h>
#include "motor_joint_control.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static float Motor_JointControl_Clamp(float value, float limit);
static bool Motor_JointControl_ConfigValid(const motor_joint_control_config_t *config);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把数值限制在对称区间内。
 * @param {float} value  输入
 * @param {float} limit  绝对值上限，须为正
 * @return {float} 限幅后的值
 * @author: lkc
 */
static float Motor_JointControl_Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/**
 * @description: 检查配置里的增益、限幅和时间上限都是有限值且符号正确。
 * @param {const motor_joint_control_config_t *} config  待检查配置
 * @return {bool} true=可以用于初始化或在线更新
 * @author: lkc
 */
static bool Motor_JointControl_ConfigValid(const motor_joint_control_config_t *config)
{
    if (config == NULL || !isfinite(config->kp_output_per_rad) || !isfinite(config->kd_output_per_rad_s) ||
        !isfinite(config->ki_output_per_rad_s) || !isfinite(config->output_limit) ||
        !isfinite(config->integral_limit) || !isfinite(config->integral_max_error_rad) ||
        !isfinite(config->integral_max_velocity_rad_s) || !isfinite(config->integral_max_target_rad_s) ||
        !isfinite(config->target_reset_threshold_rad) || !isfinite(config->dt_max_s)) {
        return false;
    }
    return config->kp_output_per_rad >= 0.0f && config->kd_output_per_rad_s >= 0.0f &&
           config->ki_output_per_rad_s >= 0.0f && config->output_limit > 0.0f &&
           config->integral_limit >= 0.0f &&
           config->integral_max_error_rad >= 0.0f && config->integral_max_velocity_rad_s >= 0.0f &&
           config->integral_max_target_rad_s >= 0.0f && config->target_reset_threshold_rad >= 0.0f &&
           config->dt_max_s > 0.0f;
}

/**
 * @description: 初始化关节控制实例并校验配置。
 * @param {motor_joint_control_t *} control  控制实例，成功后被清零再写入配置
 * @param {const motor_joint_control_config_t *} config  刚度、阻尼、积分和限幅
 * @return {bool} true=配置有效且初始化完成
 * @author: lkc
 */
bool Motor_JointControl_Init(motor_joint_control_t *control, const motor_joint_control_config_t *config)
{
    if (control == NULL || !Motor_JointControl_ConfigValid(config)) return false;
    memset(control, 0, sizeof(*control));
    control->config = *config;
    return true;
}

/**
 * @description: 在线更新控制配置，保留并按新上限约束积分。
 * @param {motor_joint_control_t *} control  已初始化的控制实例
 * @param {const motor_joint_control_config_t *} config  新配置
 * @return {bool} true=配置有效
 * @author: lkc
 */
bool Motor_JointControl_Configure(motor_joint_control_t *control, const motor_joint_control_config_t *config)
{
    if (control == NULL || !Motor_JointControl_ConfigValid(config)) return false;
    control->config = *config;
    control->integral_output = Motor_JointControl_Clamp(control->integral_output, config->integral_limit);
    control->output = Motor_JointControl_Clamp(control->output, config->output_limit);
    return true;
}

/**
 * @description: 设置完整命令；位置目标明显变化时清除旧落点积分。
 * @param {motor_joint_control_t *} control  已初始化的控制实例
 * @param {const motor_joint_control_command_t *} command  位置 (rad)、速度 (rad/s) 和前馈
 * @return {bool} true=命令有限且已接收
 * @author: lkc
 */
bool Motor_JointControl_SetCommand(motor_joint_control_t *control,
                                   const motor_joint_control_command_t *command)
{
    if (control == NULL || command == NULL || !isfinite(command->position_rad) ||
        !isfinite(command->velocity_rad_s) || !isfinite(command->feedforward_output)) {
        return false;
    }
    if (fabsf(command->position_rad - control->command.position_rad) >=
        control->config.target_reset_threshold_rad) {
        control->integral_output = 0.0f;
    }
    control->command = *command;
    return true;
}

/**
 * @description: 推进一步阻抗控制。
 * @detail: 速度无效时不使用阻尼且禁止积分。enabled=false 时清除输出和积分。
 * @param {motor_joint_control_t *} control  已初始化的控制实例
 * @param {const motor_joint_control_feedback_t *} feedback  实测位置 (rad) 和速度 (rad/s)
 * @param {float} dt  本周期时间 (s)。非正或大于 dt_max_s 时本拍不积分
 * @param {bool} enabled  false=清输出和积分并返回 0
 * @return {float} 限幅后的控制输出，单位与配置中的 output 一致
 * @author: lkc
 */
float Motor_JointControl_Step(motor_joint_control_t *control,
                              const motor_joint_control_feedback_t *feedback,
                              float dt,
                              bool enabled)
{
    float base_output;
    float candidate;
    float unsaturated;
    bool winding_up;

    if (control == NULL || feedback == NULL || !isfinite(feedback->position_rad) ||
        !isfinite(feedback->velocity_rad_s)) {
        if (control != NULL) {
            control->output = 0.0f;
            control->integral_active = false;
        }
        return 0.0f;
    }
    control->position_error_rad = control->command.position_rad - feedback->position_rad;
    control->velocity_error_rad_s = feedback->velocity_valid
                                        ? control->command.velocity_rad_s - feedback->velocity_rad_s
                                        : 0.0f;
    if (!enabled) {
        Motor_JointControl_Reset(control);
        return 0.0f;
    }
    if (!(dt > 0.0f) || dt > control->config.dt_max_s || !isfinite(dt)) {
        dt = 0.0f;
    }

    base_output = control->config.kp_output_per_rad * control->position_error_rad;
    base_output += control->config.kd_output_per_rad_s * control->velocity_error_rad_s;
    base_output += control->command.feedforward_output;
    control->integral_active = feedback->velocity_valid &&
                               fabsf(control->command.velocity_rad_s) <=
                                   control->config.integral_max_target_rad_s &&
                               fabsf(feedback->velocity_rad_s) <=
                                   control->config.integral_max_velocity_rad_s &&
                               fabsf(control->position_error_rad) <= control->config.integral_max_error_rad;

    candidate = control->integral_output;
    if (control->integral_active && dt > 0.0f) {
        candidate += control->config.ki_output_per_rad_s * control->position_error_rad * dt;
        candidate = Motor_JointControl_Clamp(candidate, control->config.integral_limit);
    }
    unsaturated = base_output + candidate;
    winding_up = (unsaturated > control->config.output_limit && control->position_error_rad > 0.0f) ||
                 (unsaturated < -control->config.output_limit && control->position_error_rad < 0.0f);
    if (control->integral_active && !winding_up) {
        control->integral_output = candidate;
    }
    control->output = Motor_JointControl_Clamp(base_output + control->integral_output,
                                               control->config.output_limit);
    return control->output;
}

/**
 * @description: 清除积分、误差和输出，配置与命令保持不变。
 * @param {motor_joint_control_t *} control  控制实例，为空则忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_JointControl_Reset(motor_joint_control_t *control)
{
    COMMON_ASSERT(control != NULL);
    control->position_error_rad = 0.0f;
    control->velocity_error_rad_s = 0.0f;
    control->integral_output = 0.0f;
    control->output = 0.0f;
    control->integral_active = false;
}

#ifdef __cplusplus
}
#endif
