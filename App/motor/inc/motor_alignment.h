/*
 * @Description: I/F 启动前转子预对齐状态机
 * @Author: lkc
 */
#ifndef MOTOR_ALIGNMENT_H
#define MOTOR_ALIGNMENT_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    bool enabled;
    float current;
    float duration;
    float angle;
} motor_alignment_config_t;

typedef struct {
    bool stopped;
    bool running;
    bool complete;
    float elapsed;
} motor_alignment_t;

typedef struct {
    bool hold_position;
    float angle;
    float id_target;
    float iq_target;
} motor_alignment_output_t;

typedef enum {
    MOTOR_ALIGNMENT_SKIPPED = 0,
    MOTOR_ALIGNMENT_STARTED,
    MOTOR_ALIGNMENT_RUNNING,
    MOTOR_ALIGNMENT_FINISHED
} motor_alignment_result_t;

/**
 * @description: 复位对齐状态机，恢复到“已停止”的初始状态。
 * @param {motor_alignment_t} *state  对齐状态机；为 NULL 时直接返回
 * @return {*}
 * @author: lkc
 */
void MotorAlignment_Reset(motor_alignment_t *state);

/**
 * @description: 归一化对齐配置，将非法负值钳位为 0。
 * @param {motor_alignment_config_t} *config  对齐配置；为 NULL 时直接返回
 * @return {*}
 * @author: lkc
 */
void MotorAlignment_NormalizeConfig(motor_alignment_config_t *config);

/**
 * @description: 将状态机标记为“跳过对齐”。
 * @param {motor_alignment_t} *state  对齐状态机；为 NULL 时直接返回
 * @return {*}
 * @author: lkc
 */
void MotorAlignment_MarkSkipped(motor_alignment_t *state);

/**
 * @description: 将状态机标记为“对齐完成”。
 * @param {motor_alignment_t} *state  对齐状态机；为 NULL 时直接返回
 * @return {*}
 * @author: lkc
 */
void MotorAlignment_MarkComplete(motor_alignment_t *state);

/**
 * @description: 推进对齐状态机一个控制周期。
 * @param {motor_alignment_t} *state                  对齐状态机
 * @param {const motor_alignment_config_t} *config    对齐配置
 * @param {float} dt                                  本拍时间步长 (s)
 * @param {motor_alignment_output_t} *output          输出：保持位置、角度、id/iq 给定
 * @return {motor_alignment_result_t} SKIPPED/STARTED/RUNNING/FINISHED
 * @author: lkc
 */
motor_alignment_result_t MotorAlignment_Step(
    motor_alignment_t *state,
    const motor_alignment_config_t *config,
    float dt,
    motor_alignment_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
