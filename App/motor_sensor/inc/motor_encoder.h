/*
 * @Description: AB 增量编码器（TIM4 正交 4 倍频）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 电角绝对有效仅在零点标定（Detect）之后；上电仅清零计数不等于已标定。
 *   2) 速度由计数差分/回绕估算；angle_valid 与 speed_valid 分开。
 *   3) *State API 供主机双实例测试；默认包装读 HAL。
 */
#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 编码器运行时（计数差分、零点偏置与速度有效标志）
 */
typedef struct {
    uint32_t counts_per_rev; /* 每机械圈计数（含倍频） */
    bool inverted;           /* true=方向取反 */
    float elec_offset_rad;   /* 电角度零点偏置 (rad) */
    bool zero_valid;         /* Detect/MarkZero 成功后为 true */

    uint16_t last_count; /* 上一拍计数 */
    bool have_last;      /* 是否已有上一拍（首拍速度无效） */
    float mech_rpm;      /* 机械转速 (RPM) */
    bool speed_valid;    /* 本拍速度是否可信 */
} motor_encoder_runtime_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化编码器运行时（不访问硬件）
 * @detail: 清零点与速度历史；counts_per_rev 为 0 时按 1 处理
 * @param {motor_encoder_runtime_t} *rt  运行时状态，不可为空
 * @param {uint32_t} counts_per_rev      每机械圈计数（含正交倍频）
 * @param {bool} inverted                true=机械正方向与计数方向相反
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_RuntimeInit(motor_encoder_runtime_t *rt, uint32_t counts_per_rev, bool inverted);

/**
 * @description: 用新计数与采样间隔更新机械转速（处理计数回绕）
 * @detail: 不修改电角零点；dt 过小/过大或首拍时 speed_valid=false
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count               当前正交解码计数值
 * @param {float} dt_s                   距上一拍时间 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_UpdateState(motor_encoder_runtime_t *rt, uint16_t count, float dt_s);

/**
 * @description: 由运行时与当前计数计算单圈机械角
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count                     当前计数值
 * @return {float} 机械角度 (rad)，归一化到 [0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetMechAngleState(const motor_encoder_runtime_t *rt, uint16_t count);

/**
 * @description: 由运行时、当前计数与极对数计算电角度
 * @detail: elec = wrap(mech × pole_pairs − elec_offset)；绝对有效须 zero_valid=true
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count                     当前计数值
 * @param {float} pole_pairs                   极对数；小于 1 时按 1 处理
 * @return {float} 电角度 (rad)，归一化到 [0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetElecAngleState(const motor_encoder_runtime_t *rt, uint16_t count, float pole_pairs);

/**
 * @description: 读取 UpdateState 后缓存的机械转速
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @return {float} 机械转速 (RPM)；speed_valid=false 时勿当作有效零速
 * @author: lkc
 */
float Motor_Encoder_GetMechRpmState(const motor_encoder_runtime_t *rt);

/**
 * @description: 查询本拍机械转速是否可信
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @return {bool} true=speed_valid；rt 为空返回 false
 * @author: lkc
 */
bool Motor_Encoder_IsSpeedValidState(const motor_encoder_runtime_t *rt);

/**
 * @description: 查询电角零点是否已标定（绝对电角可用）
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @return {bool} true=zero_valid；rt 为空返回 false
 * @author: lkc
 */
bool Motor_Encoder_IsZeroValidState(const motor_encoder_runtime_t *rt);

/**
 * @description: 将当前位置标为电角度 0（写入 offset 并置 zero_valid）
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count               当前计数值
 * @param {float} pole_pairs             极对数
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_MarkZeroState(motor_encoder_runtime_t *rt, uint16_t count, float pole_pairs);

/**
 * @description: 清除电角零点与速度差分历史（Reset 路径用）
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_ClearZeroState(motor_encoder_runtime_t *rt);

/**
 * @description: 初始化 AB 编码器硬件与默认运行时实例
 * @detail: 调用 Hal_Encoder_Init，按硬件每圈计数 RuntimeInit，并将计数清零
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Init(void);

/**
 * @description: 读取正交解码当前计数值
 * @return {uint16_t} 当前计数值（硬件计数器宽度内）
 * @author: lkc
 */
uint16_t Motor_Encoder_GetCount(void);

/**
 * @description: 硬件计数清零，并清除默认实例的电角偏置与速度历史
 * @detail: 清零不等于已完成电角零点标定（zero_valid 会被清掉）
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Reset(void);

/**
 * @description: 用 HAL 当前计数与采样间隔更新默认实例转速
 * @param {float} dt_s  控制/采样周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Update(float dt_s);

/**
 * @description: 读取默认实例当前机械角（结合 HAL 计数）
 * @return {float} 机械角度 (rad)，[0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetMechAngle(void);

/**
 * @description: 读取默认实例当前电角
 * @detail: 极对数取自全局 motor.ol_pole_pairs；绝对有效须先 Detect
 * @return {float} 电角度 (rad)，[0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetElecAngle(void);

/**
 * @description: 读取默认实例已更新的机械转速
 * @return {float} 机械转速 (RPM)；无效时勿当有效零速
 * @author: lkc
 */
float Motor_Encoder_GetMechRpm(void);

/**
 * @description: 查询默认实例本拍转速是否有效
 * @return {bool} true=速度可信
 * @author: lkc
 */
bool Motor_Encoder_IsSpeedValid(void);

/**
 * @description: 查询默认实例电角零点是否已标定
 * @return {bool} true=绝对电角可用
 * @author: lkc
 */
bool Motor_Encoder_IsZeroValid(void);

/**
 * @description: 无 Z 相时的零点标定：把当前位置记为电角度 0
 * @detail: 使用当前 HAL 计数与 motor.ol_pole_pairs 调用 MarkZeroState
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Detect(void);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
