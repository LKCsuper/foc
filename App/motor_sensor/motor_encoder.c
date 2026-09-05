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
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef MOTOR_ENCODER_INVERTED
#define MOTOR_ENCODER_INVERTED (false)
#endif
#define MOTOR_ENCODER_DT_MIN_S (1.0e-6f)
#define MOTOR_ENCODER_DT_MAX_S (0.1f)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_encoder_runtime_t s_encoder;
/* Private function prototypes -----------------------------------------------*/
static float Motor_Encoder_Wrap2Pi(float angle);
static int32_t Motor_Encoder_DeltaCounts(uint16_t now, uint16_t last, uint32_t cpr);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将角度循环归一化到 [0, 2π)。
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
 * @description: 计算最短回绕差分（有符号计数增量）。
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
 * @description: 初始化运行时（不访问硬件）。
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
 * @description: 用新计数与 dt 更新速度。
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
 * @description: 机械角。
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
 * @description: 电角度。
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
 * @description: 机械转速。
 */
float Motor_Encoder_GetMechRpmState(const motor_encoder_runtime_t *rt)
{
    return rt == NULL ? 0.0f : rt->mech_rpm;
}

/**
 * @description: 速度是否有效。
 */
bool Motor_Encoder_IsSpeedValidState(const motor_encoder_runtime_t *rt)
{
    return rt != NULL && rt->speed_valid;
}

/**
 * @description: 零点是否已标定。
 */
bool Motor_Encoder_IsZeroValidState(const motor_encoder_runtime_t *rt)
{
    return rt != NULL && rt->zero_valid;
}

/**
 * @description: 标定电角零点。
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
 * @description: 清除零点与速度历史。
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
 * @description: 初始化 AB 编码器定时器。
 */
void Motor_Encoder_Init(void)
{
    Hal_Encoder_Init();
    Motor_Encoder_RuntimeInit(&s_encoder, Hal_Encoder_CountsPerRev(), MOTOR_ENCODER_INVERTED);
    Hal_Encoder_SetCount(0U);
    printf("Encoder AB: %u count/rev\r\n", (unsigned)Hal_Encoder_CountsPerRev());
}

/**
 * @description: 读取正交解码计数值。
 */
uint16_t Motor_Encoder_GetCount(void)
{
    return Hal_Encoder_GetCount();
}

/**
 * @description: 计数清零并清除偏置与速度历史。
 */
void Motor_Encoder_Reset(void)
{
    Hal_Encoder_SetCount(0U);
    Motor_Encoder_ClearZeroState(&s_encoder);
}

/**
 * @description: 用 HAL 计数更新默认实例速度。
 */
void Motor_Encoder_Update(float dt_s)
{
    Motor_Encoder_UpdateState(&s_encoder, Hal_Encoder_GetCount(), dt_s);
}

/**
 * @description: 机械角度。
 */
float Motor_Encoder_GetMechAngle(void)
{
    return Motor_Encoder_GetMechAngleState(&s_encoder, Hal_Encoder_GetCount());
}

/**
 * @description: 电角度。
 */
float Motor_Encoder_GetElecAngle(void)
{
    float pole_pairs = motor.ol_pole_pairs;

    return Motor_Encoder_GetElecAngleState(&s_encoder, Hal_Encoder_GetCount(), pole_pairs);
}

/**
 * @description: 机械转速。
 */
float Motor_Encoder_GetMechRpm(void)
{
    return Motor_Encoder_GetMechRpmState(&s_encoder);
}

/**
 * @description: 速度是否有效。
 */
bool Motor_Encoder_IsSpeedValid(void)
{
    return Motor_Encoder_IsSpeedValidState(&s_encoder);
}

/**
 * @description: 零点是否已标定。
 */
bool Motor_Encoder_IsZeroValid(void)
{
    return Motor_Encoder_IsZeroValidState(&s_encoder);
}

/**
 * @description: 把当前位置记为电角度 0。
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
