/*
 * @Description: 关节阻抗控制纯算法，不访问编码器、PWM、RTOS或全局电机
 * @Author: lkc
 */
#ifndef MOTOR_JOINT_CONTROL_H
#define MOTOR_JOINT_CONTROL_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    float kp_output_per_rad;             /* 位置刚度，输出单位/rad */
    float kd_output_per_rad_s;           /* 速度阻尼，输出单位/(rad/s) */
    float ki_output_per_rad_s;           /* 静态修正积分，输出单位/(rad*s) */
    float output_limit;                  /* 合成输出绝对值上限 */
    float integral_limit;                /* 积分输出绝对值上限 */
    float integral_max_error_rad;        /* 允许积分的最大位置误差 */
    float integral_max_velocity_rad_s;   /* 允许积分的最大实测速度 */
    float integral_max_target_rad_s;     /* 允许积分的最大目标速度 */
    float target_reset_threshold_rad;    /* 目标跳变达到该值时清积分 */
    float dt_max_s;                      /* 单拍最大有效时间 */
} motor_joint_control_config_t;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float feedforward_output;
} motor_joint_control_command_t;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    bool velocity_valid;
} motor_joint_control_feedback_t;

typedef struct {
    motor_joint_control_config_t config;
    motor_joint_control_command_t command;
    float position_error_rad;
    float velocity_error_rad_s;
    float integral_output;
    float output;
    bool integral_active;
} motor_joint_control_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化关节控制实例并校验配置。
 * @param {motor_joint_control_t *} control  控制实例，成功后被清零再写入配置
 * @param {const motor_joint_control_config_t *} config  刚度、阻尼、积分和限幅
 * @return {bool} true=配置有效且初始化完成
 * @author: lkc
 */
bool Motor_JointControl_Init(motor_joint_control_t *control, const motor_joint_control_config_t *config);

/**
 * @description: 在线更新控制配置，保留并按新上限约束积分。
 * @param {motor_joint_control_t *} control  已初始化的控制实例
 * @param {const motor_joint_control_config_t *} config  新配置
 * @return {bool} true=配置有效
 * @author: lkc
 */
bool Motor_JointControl_Configure(motor_joint_control_t *control, const motor_joint_control_config_t *config);

/**
 * @description: 设置完整命令；位置目标明显变化时清除旧落点积分。
 * @param {motor_joint_control_t *} control  已初始化的控制实例
 * @param {const motor_joint_control_command_t *} command  位置 (rad)、速度 (rad/s) 和前馈
 * @return {bool} true=命令有限且已接收
 * @author: lkc
 */
bool Motor_JointControl_SetCommand(motor_joint_control_t *control,
                                   const motor_joint_control_command_t *command);

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
                              bool enabled);

/**
 * @description: 清除积分、误差和输出，配置与命令保持不变。
 * @param {motor_joint_control_t *} control  控制实例，不可为空，违反约定触发 COMMON_ASSERT
 * @return {void} 无
 * @author: lkc
 */
void Motor_JointControl_Reset(motor_joint_control_t *control);

#ifdef __cplusplus
}
#endif
#endif
