/*
 * @Description: 统一转子反馈选择（开环/无感/编码器/霍尔）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 控制角经本模块选择，不在 motor_control 里散落 if。
 *   2) angle_valid / speed_valid 分开：失效速度不得当作有效零速交给速度环。
 *   3) 编码器/霍尔本切片仅占位（valid=false，角回退开环）；闭环有感后续再补。
 */
#ifndef MOTOR_FEEDBACK_H
#define MOTOR_FEEDBACK_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"
/* Private typedef -----------------------------------------------------------*/

/**
 * @description: 反馈来源（请求源；实际 used_source 可能因失效回退）。
 */
typedef enum {
    MOTOR_FEEDBACK_SRC_OPENLOOP = 0, /* 强制开环电角 */
    MOTOR_FEEDBACK_SRC_SENSORLESS,   /* 无感（未闭环时内部仍回退开环角） */
    MOTOR_FEEDBACK_SRC_ENCODER,      /* 增量编码器（本切片未就绪） */
    MOTOR_FEEDBACK_SRC_HALL,         /* 霍尔表解码（本切片未就绪） */
} motor_feedback_source_t;

/**
 * @description: 一拍反馈采样结果。
 */
typedef struct {
    float elec_angle_rad;              /* 供 Park/反 Park 的电角度 (rad) */
    float mech_rpm;                    /* 机械转速 (RPM)；speed_valid=false 时勿当有效零速 */
    bool angle_valid;                  /* 电角是否来自选定传感器且可信 */
    bool speed_valid;                  /* 转速是否可信；false 时禁止 SpeedFeedback_Set(..., true) */
    motor_feedback_source_t used_source; /* 实际采用的来源（可能回退开环） */
} motor_feedback_sample_t;

/**
 * @description: 反馈选择实例。
 */
typedef struct {
    motor_feedback_source_t source;
} motor_feedback_t;

/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 复位反馈选择为默认无感源（与历史 GetCurrentTheta 行为一致）。
 * @param {motor_feedback_t *} fb 实例，不可为空
 */
void Motor_Feedback_Reset(motor_feedback_t *fb);

/**
 * @description: 设置请求的反馈来源。
 * @param {motor_feedback_t *} fb 实例
 * @param {motor_feedback_source_t} source 来源
 */
void Motor_Feedback_SetSource(motor_feedback_t *fb, motor_feedback_source_t source);

/**
 * @description: 读取请求的反馈来源。
 * @param {const motor_feedback_t *} fb 实例
 * @return {motor_feedback_source_t} 当前请求源；fb 空则 OPENLOOP
 */
motor_feedback_source_t Motor_Feedback_GetSource(const motor_feedback_t *fb);

/**
 * @description: 按请求源解析本拍电角/转速有效标志（不写速度环、不访问 PWM）。
 * @detail: ENCODER/HALL 未就绪时 angle_valid/speed_valid=false，elec_angle 回退 open_loop_theta。
 * @param {motor_feedback_t *} fb 实例
 * @param {const MOTOR_PARAM_S *} motor 电机（查 sensorless_enable 等），可为空（当开环）
 * @param {float} open_loop_theta 开环/I/F 电角 (rad)
 * @param {motor_feedback_sample_t *} out 输出，不可为空
 */
void Motor_Feedback_Sample(motor_feedback_t *fb, const MOTOR_PARAM_S *motor, float open_loop_theta,
                           motor_feedback_sample_t *out);

/**
 * @description: 默认单实例复位为无感源。
 */
void Motor_Feedback_ResetDefault(void);

/**
 * @description: 默认单实例设置来源。
 * @param {motor_feedback_source_t} source 来源
 */
void Motor_Feedback_SetSourceDefault(motor_feedback_source_t source);

/**
 * @description: 读取默认单实例请求的反馈来源。
 * @return {motor_feedback_source_t} 当前请求源
 */
motor_feedback_source_t Motor_Feedback_GetSourceDefault(void);

/**
 * @description: 是否为有感角源（编码器/霍尔），用于跳过无感更新与开环限速。
 * @param {motor_feedback_source_t} source 请求源
 * @return {bool} true=ENCODER 或 HALL
 */
bool Motor_Feedback_IsSensoredSource(motor_feedback_source_t source);

/**
 * @description: 默认单实例采样。
 * @param {const MOTOR_PARAM_S *} motor 电机
 * @param {float} open_loop_theta 开环电角 (rad)
 * @param {motor_feedback_sample_t *} out 输出
 */
void Motor_Feedback_SampleDefault(const MOTOR_PARAM_S *motor, float open_loop_theta,
                                  motor_feedback_sample_t *out);

#ifdef __cplusplus
}
#endif
#endif
