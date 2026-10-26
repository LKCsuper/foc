/*
 * @Description: I/F 启动派生量计算状态机
 * @Author: lkc
 */
#ifndef MOTOR_IF_START_H
#define MOTOR_IF_START_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* I/F 启动配置：最低转速、起步相位补偿与最低 Iq 缩放。 */
typedef struct {
    float min_rpm;       /* 最低转速 (RPM) */
    float phase_boost;   /* 起步相位补偿 */
    float min_iq_scale;  /* 最低 Iq 缩放 (0..1) */
} motor_if_start_config_t;

/* I/F 启动输入：目标/当前转速与转子位置是否已知。 */
typedef struct {
    float target_rpm;          /* 目标转速 (RPM) */
    float current_rpm;         /* 当前转速 (RPM) */
    bool rotor_position_known; /* true=转子位置已知 */
} motor_if_start_input_t;

/* I/F 启动派生量状态。 */
typedef struct {
    bool first_cycle;   /* true=首个控制周期 */
    float angle_rpm;    /* 角度积分转速 (RPM) */
    float iq_scale;     /* Iq 缩放 (0..1) */
    float phase_boost;  /* 起步相位补偿 */
} motor_if_start_t;

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 复位 I/F 启动状态机并恢复默认运行值。
 * @param {motor_if_start_t} *state  I/F 启动状态机实例，可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void MotorIfStart_Reset(motor_if_start_t *state);

/**
 * @description: 规整 I/F 启动配置，使取值合法。
 * @param {motor_if_start_config_t} *config  I/F 启动配置结构体，被原地规整
 * @return {void} 无
 * @author: lkc
 */
void MotorIfStart_NormalizeConfig(motor_if_start_config_t *config);

/**
 * @description: 置为“转子已对齐/位置已知”的中性状态。
 * @param {motor_if_start_t} *state  I/F 启动状态机实例，可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void MotorIfStart_SetNeutral(motor_if_start_t *state);

/**
 * @description: V/F 起步，直接设定角度积分转速。
 * @param {motor_if_start_t} *state  I/F 启动状态机实例，可为 NULL
 * @param {float} rpm                角度积分转速 (RPM)
 * @return {void} 无
 * @author: lkc
 */
void MotorIfStart_SetVfRpm(motor_if_start_t *state, float rpm);

/**
 * @description: 每 FOC 周期更新一次 I/F 启动派生量。
 * @param {motor_if_start_t} *state                I/F 启动状态机实例
 * @param {const motor_if_start_config_t} *config  I/F 启动配置
 * @param {const motor_if_start_input_t} *input    本拍输入（目标/当前转速、转子位置是否已知）
 * @return {void} 无
 * @author: lkc
 */
void MotorIfStart_Step(motor_if_start_t *state,
                       const motor_if_start_config_t *config,
                       const motor_if_start_input_t *input);

#ifdef __cplusplus
}
#endif

#endif
