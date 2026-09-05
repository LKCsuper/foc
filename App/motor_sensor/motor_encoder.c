/*
 * @Description: AB 增量编码器（TIM4 PB6/PB7 正交 4 倍频）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "motor_encoder.h"
#include "motor_param.h"
#include "motor_math.h"
#include "hal_sensor.h"
/* Private define ------------------------------------------------------------*/
#ifndef MOTOR_ENCODER_INVERTED
#define MOTOR_ENCODER_INVERTED (false)
#endif
#define MOTOR_ENCODER_DT_MIN_S (1.0e-6f)
#define MOTOR_ENCODER_DT_MAX_S (0.1f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_encoder_runtime_t s_encoder;
/* Private function prototypes -----------------------------------------------*/
static float Motor_Encoder_Wrap2Pi(float angle);
static int32_t Motor_Encoder_DeltaCounts(uint16_t now, uint16_t last, uint32_t cpr);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将角度循环归一化到 [0, 2π)
 * @param {float} angle  输入角度 (rad)
 * @return {float} 归一化后的角度 (rad)
 * @author: lkc
 */
static float Motor_Encoder_Wrap2Pi(float angle)
{
    while (angle >= (2.0f * (float)M_PI)) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < 0.0f) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

/**
 * @description: 计算最短路径的有符号计数增量（处理回绕）
 * @param {uint16_t} now   当前计数值
 * @param {uint16_t} last  上一拍计数值
 * @param {uint32_t} cpr   每圈计数；0 时按 1
 * @return {int32_t} 有符号计数差（最短路径）
 * @author: lkc
 */
static int32_t Motor_Encoder_DeltaCounts(uint16_t now, uint16_t last, uint32_t cpr)
{
    int32_t period = (cpr > 0U) ? (int32_t)cpr : 1;
    int32_t delta = (int32_t)now - (int32_t)last;

    if (delta > (period / 2)) {
        delta -= period;
    } else if (delta < -(period / 2)) {
        delta += period;
    }
    return delta;
}

/**
 * @description: 初始化编码器运行时（不访问硬件）
 * @detail: 清零点与速度历史；counts_per_rev 为 0 时按 1 处理
 * @param {motor_encoder_runtime_t} *rt  运行时状态，不可为空
 * @param {uint32_t} counts_per_rev      每机械圈计数（含正交倍频）
 * @param {bool} inverted                true=机械正方向与计数方向相反
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_RuntimeInit(motor_encoder_runtime_t *rt, uint32_t counts_per_rev, bool inverted)
{
    if (rt == NULL) {
        return;
    }
    rt->counts_per_rev = (counts_per_rev == 0U) ? 1U : counts_per_rev;
    rt->inverted = inverted;
    rt->elec_offset_rad = 0.0f;
    rt->zero_valid = false;
    rt->last_count = 0U;
    rt->have_last = false;
    rt->mech_rpm = 0.0f;
    rt->speed_valid = false;
}

/**
 * @description: 用新计数与采样间隔更新机械转速（处理计数回绕）
 * @detail: 不修改电角零点；dt 过小/过大或首拍时 speed_valid=false
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count               当前正交解码计数值
 * @param {float} dt_s                   距上一拍时间 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_UpdateState(motor_encoder_runtime_t *rt, uint16_t count, float dt_s)
{
    int32_t delta;
    float turns_per_s;

    if (rt == NULL) {
        return;
    }

    if (!rt->have_last || dt_s < MOTOR_ENCODER_DT_MIN_S || dt_s > MOTOR_ENCODER_DT_MAX_S) {
        rt->last_count = count;
        rt->have_last = true;
        rt->mech_rpm = 0.0f;
        rt->speed_valid = false;
        return;
    }

    delta = Motor_Encoder_DeltaCounts(count, rt->last_count, rt->counts_per_rev);
    if (rt->inverted) {
        delta = -delta;
    }
    turns_per_s = ((float)delta / (float)rt->counts_per_rev) / dt_s;
    rt->mech_rpm = turns_per_s * 60.0f;
    rt->speed_valid = true;
    rt->last_count = count;
}

/**
 * @description: 由运行时与当前计数计算单圈机械角
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count                     当前计数值
 * @return {float} 机械角度 (rad)，归一化到 [0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetMechAngleState(const motor_encoder_runtime_t *rt, uint16_t count)
{
    float turns;
    uint32_t cpr;

    if (rt == NULL) {
        return 0.0f;
    }
    cpr = rt->counts_per_rev;
    if (cpr == 0U) {
        cpr = 1U;
    }
    turns = (float)count / (float)cpr;
    if (rt->inverted) {
        turns = 1.0f - turns;
    }
    return Motor_Encoder_Wrap2Pi(turns * 2.0f * (float)M_PI);
}

/**
 * @description: 由运行时、当前计数与极对数计算电角度
 * @detail: elec = wrap(mech × pole_pairs − elec_offset)；绝对有效须 zero_valid=true
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count                     当前计数值
 * @param {float} pole_pairs                   极对数；小于 1 时按 1 处理
 * @return {float} 电角度 (rad)，归一化到 [0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetElecAngleState(const motor_encoder_runtime_t *rt, uint16_t count, float pole_pairs)
{
    float pp = pole_pairs;

    if (rt == NULL) {
        return 0.0f;
    }
    if (pp < 1.0f) {
        pp = 1.0f;
    }
    return Motor_Encoder_Wrap2Pi(Motor_Encoder_GetMechAngleState(rt, count) * pp - rt->elec_offset_rad);
}

/**
 * @description: 读取 UpdateState 后缓存的机械转速
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @return {float} 机械转速 (RPM)；speed_valid=false 时勿当作有效零速
 * @author: lkc
 */
float Motor_Encoder_GetMechRpmState(const motor_encoder_runtime_t *rt)
{
    return rt == NULL ? 0.0f : rt->mech_rpm;
}

/**
 * @description: 查询本拍机械转速是否可信
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @return {bool} true=speed_valid；rt 为空返回 false
 * @author: lkc
 */
bool Motor_Encoder_IsSpeedValidState(const motor_encoder_runtime_t *rt)
{
    return rt != NULL && rt->speed_valid;
}

/**
 * @description: 查询电角零点是否已标定（绝对电角可用）
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @return {bool} true=zero_valid；rt 为空返回 false
 * @author: lkc
 */
bool Motor_Encoder_IsZeroValidState(const motor_encoder_runtime_t *rt)
{
    return rt != NULL && rt->zero_valid;
}

/**
 * @description: 将当前位置标为电角度 0（写入 offset 并置 zero_valid）
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count               当前计数值
 * @param {float} pole_pairs             极对数
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_MarkZeroState(motor_encoder_runtime_t *rt, uint16_t count, float pole_pairs)
{
    if (rt == NULL) {
        return;
    }
    rt->elec_offset_rad = 0.0f;
    rt->elec_offset_rad = Motor_Encoder_GetElecAngleState(rt, count, pole_pairs);
    rt->zero_valid = true;
}

/**
 * @description: 清除电角零点与速度差分历史（Reset 路径用）
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_ClearZeroState(motor_encoder_runtime_t *rt)
{
    if (rt == NULL) {
        return;
    }
    rt->elec_offset_rad = 0.0f;
    rt->zero_valid = false;
    rt->have_last = false;
    rt->mech_rpm = 0.0f;
    rt->speed_valid = false;
}

/**
 * @description: 初始化 AB 编码器硬件与默认运行时实例
 * @detail: 调用 Hal_Encoder_Init，按硬件每圈计数 RuntimeInit，并将计数清零
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Init(void)
{
    Hal_Encoder_Init();
    Motor_Encoder_RuntimeInit(&s_encoder, Hal_Encoder_CountsPerRev(), MOTOR_ENCODER_INVERTED);
    Hal_Encoder_SetCount(0U);
    printf("Encoder AB: %u count/rev\r\n", (unsigned)Hal_Encoder_CountsPerRev());
}

/**
 * @description: 读取正交解码当前计数值
 * @return {uint16_t} 当前计数值（硬件计数器宽度内）
 * @author: lkc
 */
uint16_t Motor_Encoder_GetCount(void)
{
    return Hal_Encoder_GetCount();
}

/**
 * @description: 硬件计数清零，并清除默认实例的电角偏置与速度历史
 * @detail: 清零不等于已完成电角零点标定（zero_valid 会被清掉）
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Reset(void)
{
    Hal_Encoder_SetCount(0U);
    Motor_Encoder_ClearZeroState(&s_encoder);
}

/**
 * @description: 用 HAL 当前计数与采样间隔更新默认实例转速
 * @param {float} dt_s  控制/采样周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Update(float dt_s)
{
    Motor_Encoder_UpdateState(&s_encoder, Hal_Encoder_GetCount(), dt_s);
}

/**
 * @description: 读取默认实例当前机械角（结合 HAL 计数）
 * @return {float} 机械角度 (rad)，[0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetMechAngle(void)
{
    return Motor_Encoder_GetMechAngleState(&s_encoder, Hal_Encoder_GetCount());
}

/**
 * @description: 读取默认实例当前电角
 * @detail: 极对数取自全局 motor.ol_pole_pairs；绝对有效须先 Detect
 * @return {float} 电角度 (rad)，[0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetElecAngle(void)
{
    float pole_pairs = motor.ol_pole_pairs;

    return Motor_Encoder_GetElecAngleState(&s_encoder, Hal_Encoder_GetCount(), pole_pairs);
}

/**
 * @description: 读取默认实例已更新的机械转速
 * @return {float} 机械转速 (RPM)；无效时勿当有效零速
 * @author: lkc
 */
float Motor_Encoder_GetMechRpm(void)
{
    return Motor_Encoder_GetMechRpmState(&s_encoder);
}

/**
 * @description: 查询默认实例本拍转速是否有效
 * @return {bool} true=速度可信
 * @author: lkc
 */
bool Motor_Encoder_IsSpeedValid(void)
{
    return Motor_Encoder_IsSpeedValidState(&s_encoder);
}

/**
 * @description: 查询默认实例电角零点是否已标定
 * @return {bool} true=绝对电角可用
 * @author: lkc
 */
bool Motor_Encoder_IsZeroValid(void)
{
    return Motor_Encoder_IsZeroValidState(&s_encoder);
}

/**
 * @description: 无 Z 相时的零点标定：把当前位置记为电角度 0
 * @detail: 使用当前 HAL 计数与 motor.ol_pole_pairs 调用 MarkZeroState
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Detect(void)
{
    float pole_pairs = motor.ol_pole_pairs;

    Motor_Encoder_MarkZeroState(&s_encoder, Hal_Encoder_GetCount(), pole_pairs);
    printf("Encoder detect: count=%u mech=%.3f rad elec0=%.3f rad zero_ok=%d\r\n",
           (unsigned)Motor_Encoder_GetCount(), Motor_Encoder_GetMechAngle(), Motor_Encoder_GetElecAngle(),
           Motor_Encoder_IsZeroValid() ? 1 : 0);
}

#ifdef __cplusplus
}
#endif
