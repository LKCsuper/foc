/*
 * @Description: 纯位置串级控制（位置 P 串速度 PI）
 * @Author: lkc
 *
 * 状态由调用者持有。不读取编码器、硬件或全局电机，也不阻塞。
 */
#ifndef MOTOR_POSITION_CONTROL_H
#define MOTOR_POSITION_CONTROL_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 位置串级控制配置（不持久化，由调用者每拍传入）
 */
typedef struct {
    float position_kp_rpm_per_deg;   /* 位置比例 (RPM/deg)，须为正有限值 */
    float maximum_rpm;               /* 速度目标限幅 (RPM)，须为正有限值 */
    float velocity_kp_v_per_rpm;     /* 速度比例 (V/RPM)，须为非负有限值 */
    float velocity_ki_v_per_rpm_s;   /* 速度积分 (V/(RPM·s))，须为非负有限值 */
    float voltage_limit_v;           /* 输出与积分限幅 (V)，须为正有限值 */
    float acceleration_limit_rpm_s;  /* 速度命令变化率 (RPM/s)，须为正有限值 */
    float feedforward_v_per_rpm;     /* 运动电压前馈 (V/RPM)，须为非负有限值 */
    float maximum_dt_s;              /* 允许积分的最大步长 (s)，须为正有限值 */
} motor_position_control_config_t;

/**
 * @description: 位置串级控制状态（不持久化）
 */
typedef struct {
    float velocity_integrator_v; /* 速度积分项 (V) */
    float commanded_rpm;         /* 加速度限制后的速度命令 (RPM) */
} motor_position_control_t;

/**
 * @description: 一拍位置控制输入
 */
typedef struct {
    bool enabled;         /* true=允许保持；false 时清积分并输出 0 */
    bool motor_running;   /* true=电机已启动 */
    bool feedback_valid;  /* true=位置反馈可用 */
    bool speed_valid;     /* false 时本拍速度反馈按 0 RPM 计 */
    float target_deg;     /* 目标位置 (deg)，须为有限值 */
    float position_deg;   /* 当前位置 (deg)，须为有限值 */
    float measured_rpm;   /* 测量转速 (RPM)，须为有限值 */
    float dt;             /* 本拍时间 (s)，须为有限值；非正或超过 maximum_dt_s 时不积分 */
} motor_position_control_input_t;

/**
 * @description: 一拍位置控制输出
 */
typedef struct {
    bool hold_active; /* true=本拍速度 PI 有效，output_v 可写入保持电压 */
    float error_deg;  /* 目标减当前位置 (deg) */
    float target_rpm; /* 位置 P 给出的速度目标 (RPM)；未保持时为 0 */
    float output_v;   /* 速度 PI 电压 (V)，已限幅 */
} motor_position_control_output_t;

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 按配置检查并清零位置控制状态。
 * @detail: 不访问硬件。配置含非有限值或增益、限幅不满足符号要求时失败，且不改 control。
 * @param {motor_position_control_t *} control              控制状态，不可为空
 * @param {const motor_position_control_config_t *} config  控制配置，不可为空
 * @return {bool} 初始化成功返回 true
 * @author: lkc
 */
bool Motor_PositionControl_Init(motor_position_control_t *control,
                                const motor_position_control_config_t *config);

/**
 * @description: 清除速度积分。
 * @detail: 不改配置。control 为空则直接返回。
 * @param {motor_position_control_t *} control 控制状态
 * @return {void} 无
 * @author: lkc
 */
void Motor_PositionControl_Reset(motor_position_control_t *control);

/**
 * @description: 推进一拍位置 P 串速度 PI。
 * @detail: 位置误差先换成转速并限幅，再由速度 PI 出电压。积分只在误差没有把输出
 *          继续推向电压限的一侧时更新，积分本身钳在 ±voltage_limit_v。
 *          未使能、电机未运行或位置反馈无效时清积分，目标转速和电压为 0。
 *          参数非法或输入非有限时返回 false，且不写 output。
 * @param {motor_position_control_t *} control              控制状态，不可为空
 * @param {const motor_position_control_config_t *} config  控制配置，不可为空
 * @param {const motor_position_control_input_t *} input    本拍输入，不可为空
 * @param {motor_position_control_output_t *} output        本拍输出，不可为空
 * @return {bool} 本拍计算完成返回 true
 * @author: lkc
 */
bool Motor_PositionControl_Step(motor_position_control_t *control,
                                const motor_position_control_config_t *config,
                                const motor_position_control_input_t *input,
                                motor_position_control_output_t *output);

#ifdef __cplusplus
}
#endif
#endif
