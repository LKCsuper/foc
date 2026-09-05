/*
 * @Description: AB 增量编码器（TIM4 正交 4 倍频）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 电角绝对有效仅在零点标定（Detect）之后；上电仅清零计数不等于已标定。
 *   2) 速度由计数差分/回绕估算；angle_valid 与 speed_valid 分开。
 *   3) *State API 供主机双实例测试；默认包装读 HAL。
 */
#ifndef _MOTOR_ENCODER_H
#define _MOTOR_ENCODER_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/**
 * @description: 编码器运行时（计数差分、零点与速度有效标志）。
 */
typedef struct {
    uint32_t counts_per_rev; /* 每机械圈计数（含倍频） */
    bool inverted;           /* true=方向取反 */
    float elec_offset_rad;   /* 电角度零点偏置 (rad) */
    bool zero_valid;         /* Detect 成功后为 true */

    uint16_t last_count; /* 上一拍计数 */
    bool have_last;      /* 是否已有上一拍（首拍速度无效） */
    float mech_rpm;      /* 机械转速 (RPM) */
    bool speed_valid;    /* 本拍速度是否可信 */
} motor_encoder_runtime_t;

/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 初始化运行时（不访问硬件）。
 * @param {motor_encoder_runtime_t *} rt 状态
 * @param {uint32_t} counts_per_rev 每圈计数；0 则按 1 处理
 * @param {bool} inverted 方向取反
 */
void Motor_Encoder_RuntimeInit(motor_encoder_runtime_t *rt, uint32_t counts_per_rev, bool inverted);

/**
 * @description: 用新计数与 dt 更新速度（处理计数回绕）；不改零点。
 * @param {motor_encoder_runtime_t *} rt 状态
 * @param {uint16_t} count 当前计数
 * @param {float} dt_s 距上一拍时间 (s)；过小/过大则本拍 speed_valid=false
 */
void Motor_Encoder_UpdateState(motor_encoder_runtime_t *rt, uint16_t count, float dt_s);

/**
 * @description: 由运行时与当前计数计算机械角。
 * @param {const motor_encoder_runtime_t *} rt 状态
 * @param {uint16_t} count 当前计数
 * @return {float} 机械角度 (rad)，[0, 2π)
 */
float Motor_Encoder_GetMechAngleState(const motor_encoder_runtime_t *rt, uint16_t count);

/**
 * @description: 由运行时计算电角度。
 * @param {const motor_encoder_runtime_t *} rt 状态
 * @param {uint16_t} count 当前计数
 * @param {float} pole_pairs 极对数
 * @return {float} 电角度 (rad)，[0, 2π)
 */
float Motor_Encoder_GetElecAngleState(const motor_encoder_runtime_t *rt, uint16_t count, float pole_pairs);

/**
 * @description: 读取已更新的机械转速。
 * @param {const motor_encoder_runtime_t *} rt 状态
 * @return {float} 机械转速 (RPM)；speed_valid=false 时勿当有效零速
 */
float Motor_Encoder_GetMechRpmState(const motor_encoder_runtime_t *rt);

/**
 * @description: 速度是否有效。
 */
bool Motor_Encoder_IsSpeedValidState(const motor_encoder_runtime_t *rt);

/**
 * @description: 电角零点是否已标定（绝对电角可用）。
 */
bool Motor_Encoder_IsZeroValidState(const motor_encoder_runtime_t *rt);

/**
 * @description: 将当前位置标为电角 0（写入 offset，置 zero_valid）。
 * @param {motor_encoder_runtime_t *} rt 状态
 * @param {uint16_t} count 当前计数
 * @param {float} pole_pairs 极对数
 */
void Motor_Encoder_MarkZeroState(motor_encoder_runtime_t *rt, uint16_t count, float pole_pairs);

/**
 * @description: 清除零点与速度历史（Reset 用）。
 */
void Motor_Encoder_ClearZeroState(motor_encoder_runtime_t *rt);

/**
 * @description: 初始化 AB 编码器定时器（HAL + 默认运行时）。
 */
void Motor_Encoder_Init(void);

/**
 * @description: 读取正交解码计数值。
 * @return {uint16_t} 当前计数
 */
uint16_t Motor_Encoder_GetCount(void);

/**
 * @description: 计数清零并清除电角度偏置与速度历史。
 */
void Motor_Encoder_Reset(void);

/**
 * @description: 用 HAL 计数与 dt 更新默认实例速度。
 * @param {float} dt_s 控制/采样周期 (s)
 */
void Motor_Encoder_Update(float dt_s);

/**
 * @description: 机械角度（默认实例 + HAL 计数）。
 * @return {float} 机械角度 (rad)
 */
float Motor_Encoder_GetMechAngle(void);

/**
 * @description: 电角度（默认实例；依赖全局 motor.ol_pole_pairs）。
 * @return {float} 电角度 (rad)
 */
float Motor_Encoder_GetElecAngle(void);

/**
 * @description: 默认实例机械转速 (RPM)。
 */
float Motor_Encoder_GetMechRpm(void);

/**
 * @description: 默认实例速度是否有效。
 */
bool Motor_Encoder_IsSpeedValid(void);

/**
 * @description: 默认实例电角零点是否已标定。
 */
bool Motor_Encoder_IsZeroValid(void);

/**
 * @description: 把当前位置记为电角度 0（无 Z 时的零点标定）。
 */
void Motor_Encoder_Detect(void);

#ifdef __cplusplus
}
#endif
#endif
