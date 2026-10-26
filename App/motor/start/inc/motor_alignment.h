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
#include "common_assert.h"
#include <stddef.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* 对齐配置：使能、电流、时长与定位电角。 */
typedef struct {
    bool enabled;    /* true=启用预对齐 */
    float current;   /* 对齐 D 轴电流 (A) */
    float duration;  /* 保持时长 (s) */
    float angle;     /* 定位电角 (rad) */
} motor_alignment_config_t;

/* 对齐状态机状态。 */
typedef struct {
    bool stopped;   /* true=已停止 */
    bool running;   /* true=正在保持 */
    bool complete;  /* true=已对齐完成 */
    float elapsed;  /* 已保持时长 (s) */
} motor_alignment_t;

/* 对齐每拍输出。 */
typedef struct {
    bool hold_position;  /* true=本拍保持位置 */
    float angle;         /* 定位电角 (rad) */
    float id_target;     /* D 轴电流给定 (A) */
    float iq_target;     /* Q 轴电流给定 (A) */
} motor_alignment_output_t;

/* 对齐结果码。 */
typedef enum {
    MOTOR_ALIGNMENT_SKIPPED = 0,  /* 未执行 */
    MOTOR_ALIGNMENT_STARTED,      /* 本拍启动 */
    MOTOR_ALIGNMENT_RUNNING,      /* 保持中 */
    MOTOR_ALIGNMENT_FINISHED      /* 已完成 */
} motor_alignment_result_t;

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 复位对齐状态机，恢复到“已停止”的初始状态。
 * @param {motor_alignment_t} *state  对齐状态机；为 NULL 时直接返回
 * @return {void} 无
 * @author: lkc
 */
void MotorAlignment_Reset(motor_alignment_t *state);

/**
 * @description: 归一化对齐配置，将非法负值钳位为 0。
 * @param {motor_alignment_config_t} *config  对齐配置；为 NULL 时直接返回
 * @return {void} 无
 * @author: lkc
 */
void MotorAlignment_NormalizeConfig(motor_alignment_config_t *config);
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 热路径共享实现；与NormalizeConfig保持同一套规则，不缓存配置。
 * @param {motor_alignment_config_t *} config 配置，不可为空
 * @return {void} 无
 */
static inline void MotorAlignment_NormalizeConfigInline(motor_alignment_config_t *config)
{
    COMMON_ASSERT(config != NULL);
    if (config->current < 0.0f) config->current = 0.0f;
    if (config->duration < 0.0f) config->duration = 0.0f;
}

/**
 * @description: 将状态机标记为“跳过对齐”。
 * @param {motor_alignment_t} *state  对齐状态机；为 NULL 时直接返回
 * @return {void} 无
 * @author: lkc
 */
void MotorAlignment_MarkSkipped(motor_alignment_t *state);

/**
 * @description: 将状态机标记为“对齐完成”。
 * @param {motor_alignment_t} *state  对齐状态机；为 NULL 时直接返回
 * @return {void} 无
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
