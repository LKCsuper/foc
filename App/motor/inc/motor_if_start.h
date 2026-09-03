/*
 * @Description: I/F 启动派生量计算状态机
 * @Author: lkc
 */
#ifndef MOTOR_IF_START_H
#define MOTOR_IF_START_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float min_rpm;
    float phase_boost;
    float min_iq_scale;
} motor_if_start_config_t;

typedef struct {
    float target_rpm;
    float current_rpm;
    bool rotor_position_known;
} motor_if_start_input_t;

typedef struct {
    bool first_cycle;
    float angle_rpm;
    float iq_scale;
    float phase_boost;
} motor_if_start_t;

/**
 * @description: 复位 I/F 启动状态机并恢复默认运行值。
 * @param {motor_if_start_t} *state  I/F 启动状态机实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void MotorIfStart_Reset(motor_if_start_t *state);

/**
 * @description: 规整 I/F 启动配置，使取值合法。
 * @param {motor_if_start_config_t} *config  I/F 启动配置结构体，被原地规整
 * @return {*}
 * @author: lkc
 */
void MotorIfStart_NormalizeConfig(motor_if_start_config_t *config);

/**
 * @description: 置为“转子已对齐/位置已知”的中性状态。
 * @param {motor_if_start_t} *state  I/F 启动状态机实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void MotorIfStart_SetNeutral(motor_if_start_t *state);

/**
 * @description: V/F 起步，直接设定角度积分转速。
 * @param {motor_if_start_t} *state  I/F 启动状态机实例，可为 NULL
 * @param {float} rpm                角度积分转速 (RPM)
 * @return {*}
 * @author: lkc
 */
void MotorIfStart_SetVfRpm(motor_if_start_t *state, float rpm);

/**
 * @description: 每 FOC 周期更新一次 I/F 启动派生量。
 * @param {motor_if_start_t} *state                I/F 启动状态机实例
 * @param {const motor_if_start_config_t} *config  I/F 启动配置
 * @param {const motor_if_start_input_t} *input    本拍输入（目标/当前转速、转子位置是否已知）
 * @return {*}
 * @author: lkc
 */
void MotorIfStart_Step(motor_if_start_t *state,
                       const motor_if_start_config_t *config,
                       const motor_if_start_input_t *input);

#ifdef __cplusplus
}
#endif

#endif
