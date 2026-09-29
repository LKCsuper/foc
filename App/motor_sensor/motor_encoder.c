/*
 * @Description: 编码器角度、速度与零点（MT6701 绝对计数 + ODrive 临界阻尼 PLL）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#ifndef MOTOR_ENCODER_HOST_TEST
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#endif
#include "motor_encoder.h"
#include "motor_param.h"
#include "motor_math.h"
#include "motor.h"
#include "motor_protect.h"
#include "hal_motor.h"
#include "hal_sensor.h"
#include "motor_mt6701.h"
#include "flash_storage.h"
/* Private define ------------------------------------------------------------*/
#ifndef MOTOR_ENCODER_INVERTED
#define MOTOR_ENCODER_INVERTED (false)
#endif
#define MOTOR_ENCODER_DT_MIN_S (1.0e-6f)
#define MOTOR_ENCODER_DT_MAX_S (0.1f)
/*
 * MT6701 14 bit / 10 kHz 在低速时每帧只有 0/1 count 变化。
 * 带宽由电机头提供：小电机低速齿槽明显时使用较低带宽，大电机保留较快响应。
 * PLL 只估算速度；FOC 电角度仍直接取最新 MT6701 绝对计数，不受该带宽影响。
 */
#define MOTOR_ENCODER_MAX_RPM (55000.0f)
#define MOTOR_ENCODER_JUMP_MARGIN_COUNTS (2.0f)
#define MOTOR_ENCODER_SAMPLE_TIMEOUT_S (0.002f)
#define MOTOR_ENCODER_CORRECTION_DT_MAX_S (0.0004f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_encoder_runtime_t s_encoder;
static uint32_t s_encoder_sample_sequence;
static uint32_t s_encoder_sample_time_us;
static bool s_encoder_have_sequence;
#ifndef MOTOR_ENCODER_HOST_TEST
static struct {
    volatile bool active;
    volatile float theta_rad;
    volatile float id_target;
} s_encoder_detect;
#endif
/* Private function prototypes -----------------------------------------------*/
static float Motor_Encoder_Wrap2Pi(float angle);
static float Motor_Encoder_WrapCpr(float pos, float cpr);
static float Motor_Encoder_WrapDeltaCpr(float delta, float cpr);
static float Motor_Encoder_MeasuredCounts(const motor_encoder_runtime_t *rt, uint16_t count);
static void Motor_Encoder_Seed(motor_encoder_runtime_t *rt, uint16_t count);
static bool Motor_Encoder_PllPredict(motor_encoder_runtime_t *rt, float dt_s);
static void Motor_Encoder_PllCorrect(motor_encoder_runtime_t *rt, float measured, float dt_s);
static void Motor_Encoder_CorrectFreshSample(motor_encoder_runtime_t *rt, uint16_t count, float sample_dt_s,
                                             float correction_dt_s);
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
 * @description: 把 PLL 位置折回 [0, cpr)
 * @param {float} pos  位置 (count)
 * @param {float} cpr  每圈计数
 * @return {float} 折回后的位置 (count)
 */
static float Motor_Encoder_WrapCpr(float pos, float cpr)
{
    if (cpr <= 0.0f) {
        return 0.0f;
    }
    while (pos >= cpr) {
        pos -= cpr;
    }
    while (pos < 0.0f) {
        pos += cpr;
    }
    return pos;
}

/**
 * @description: 把计数误差折到 ±cpr/2
 * @param {float} delta  未折回的误差 (count)
 * @param {float} cpr    每圈计数
 * @return {float} 最短路径误差 (count)
 */
static float Motor_Encoder_WrapDeltaCpr(float delta, float cpr)
{
    float half = 0.5f * cpr;

    if (delta > half) {
        delta -= cpr;
    } else if (delta < -half) {
        delta += cpr;
    }
    return delta;
}

/**
 * @description: 把原始计数变到 PLL 使用的测量位置
 * @detail: 反向在进 PLL 之前做完，速度符号与电角一致。
 * @param {const motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count                     原始计数
 * @return {float} 测量位置 (count)，范围 [0, cpr)
 */
static float Motor_Encoder_MeasuredCounts(const motor_encoder_runtime_t *rt, uint16_t count)
{
    float cpr = (float)rt->counts_per_rev;
    float measured = (float)count;

    if (cpr < 1.0f) {
        cpr = 1.0f;
    }
    if (rt->inverted) {
        measured = cpr - measured;
        if (measured >= cpr) {
            measured -= cpr;
        }
    }
    return measured;
}

/**
 * @description: 用当前计数重锚 PLL，本拍速度无效
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count               原始计数
 */
static void Motor_Encoder_Seed(motor_encoder_runtime_t *rt, uint16_t count)
{
    rt->last_count = count;
    rt->have_last = true;
    rt->pll_inverted = rt->inverted;
    rt->pll_pos = Motor_Encoder_MeasuredCounts(rt, count);
    rt->pll_vel = 0.0f;
    rt->edge_age_s = 0.0f;
    rt->sample_age_s = 0.0f;
    rt->mech_rpm_raw = 0.0f;
    rt->mech_rpm = 0.0f;
    rt->speed_valid = false;
    rt->sample_stale = false;
}

/**
 * @description: ODrive 临界阻尼 PLL。kp=2·bw，ki=0.25·kp²
 * @detail: 误差用连续位置，不用取整。dt·kp≥1 时离散化不稳定，钉在测量并清速度。
 *          |vel| < 0.5·dt·ki 时打成 0，静止不再留一个量化残差。
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {float} measured               本拍测量位置 (count)
 * @param {float} dt_s                   采样间隔 (s)
 */
static bool Motor_Encoder_PllPredict(motor_encoder_runtime_t *rt, float dt_s)
{
    const float kp = 2.0f * MOTOR_ENCODER_PLL_BW_RAD_S;
    float cpr = (float)rt->counts_per_rev;

    if (cpr < 1.0f) {
        cpr = 1.0f;
    }
    if ((dt_s * kp) >= 1.0f) {
        rt->pll_vel = 0.0f;
        rt->mech_rpm_raw = 0.0f;
        rt->mech_rpm = 0.0f;
        rt->speed_valid = false;
        return false;
    }

    rt->pll_pos += dt_s * rt->pll_vel;
    rt->pll_pos = Motor_Encoder_WrapCpr(rt->pll_pos, cpr);
    return true;
}

static void Motor_Encoder_PllCorrect(motor_encoder_runtime_t *rt, float measured, float dt_s)
{
    const float kp = 2.0f * MOTOR_ENCODER_PLL_BW_RAD_S;
    const float ki = 0.25f * kp * kp;
    float cpr = (float)rt->counts_per_rev;
    float delta_pos;
    float snap;
    float vel_max;
    float vel_before_snap;

    if (cpr < 1.0f) {
        cpr = 1.0f;
    }
    delta_pos = Motor_Encoder_WrapDeltaCpr(measured - rt->pll_pos, cpr);
    rt->pll_pos += dt_s * kp * delta_pos;
    rt->pll_pos = Motor_Encoder_WrapCpr(rt->pll_pos, cpr);
    rt->pll_vel += dt_s * ki * delta_pos;

    vel_max = MOTOR_ENCODER_MAX_RPM * cpr / 60.0f;
    if (rt->pll_vel > vel_max) {
        rt->pll_vel = vel_max;
    } else if (rt->pll_vel < -vel_max) {
        rt->pll_vel = -vel_max;
    }

    vel_before_snap = rt->pll_vel;
    snap = 0.5f * dt_s * ki;
    if (fabsf(rt->pll_vel) < snap) {
        rt->pll_vel = 0.0f;
    }
    rt->mech_rpm_raw = vel_before_snap / cpr * 60.0f;
    rt->mech_rpm = rt->pll_vel / cpr * 60.0f;
    rt->speed_valid = true;
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
    rt->pll_inverted = inverted;
    rt->pll_pos = 0.0f;
    rt->pll_vel = 0.0f;
    rt->edge_age_s = 0.0f;
    rt->mech_rpm_raw = 0.0f;
    rt->mech_rpm = 0.0f;
    rt->speed_valid = false;
    rt->signal_fault = false;
    rt->sample_stale = false;
    rt->sample_age_s = 0.0f;
    rt->invalid_count = 0U;
}

/**
 * @description: 用新计数更新 ODrive PLL 的位置与转速
 * @detail: 不修改电角零点；dt 过小/过大或首拍时 speed_valid=false。带宽约 100 rad/s。
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count               当前正交解码计数值
 * @param {float} dt_s                   距上一拍时间 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_UpdateState(motor_encoder_runtime_t *rt, uint16_t count, float dt_s)
{
    if (rt == NULL) {
        return;
    }

    if (!rt->have_last || rt->pll_inverted != rt->inverted || dt_s < MOTOR_ENCODER_DT_MIN_S ||
        dt_s > MOTOR_ENCODER_DT_MAX_S || (dt_s * 2.0f * MOTOR_ENCODER_PLL_BW_RAD_S) >= 1.0f) {
        Motor_Encoder_Seed(rt, count);
        return;
    }

    Motor_Encoder_PredictState(rt, dt_s);
    Motor_Encoder_CorrectFreshSample(rt, count, dt_s, dt_s);
}

static void Motor_Encoder_CorrectFreshSample(motor_encoder_runtime_t *rt, uint16_t count, float sample_dt_s,
                                             float correction_dt_s)
{
    int32_t delta;
    float max_delta;

    delta = Motor_Encoder_DeltaCounts(count, rt->last_count, rt->counts_per_rev);
    if (rt->inverted) {
        delta = -delta;
    }
    /* 允许量化产生的 1~2 count，但拒绝超过 MT6701 转速能力的突跳。坏帧不拉 PLL。 */
    max_delta = MOTOR_ENCODER_MAX_RPM * (float)rt->counts_per_rev * sample_dt_s / 60.0f;
    max_delta += MOTOR_ENCODER_JUMP_MARGIN_COUNTS;
    if (fabsf((float)delta) > max_delta) {
        rt->speed_valid = false;
        rt->signal_fault = true;
        rt->sample_stale = false;
        rt->sample_age_s = 0.0f;
        rt->invalid_count++;
        return;
    }

    /* last_count 也是 FOC 使用的最新已验收角度，坏帧不得覆盖。 */
    rt->last_count = count;
    rt->signal_fault = false;
    rt->sample_stale = false;
    rt->sample_age_s = 0.0f;
    if (delta != 0) {
        rt->edge_age_s = 0.0f;
    }

    Motor_Encoder_PllCorrect(rt, Motor_Encoder_MeasuredCounts(rt, count), correction_dt_s);
}

/**
 * @description: 没有新传感器帧时只按 PLL 速度预测位置。
 * @detail: 采样年龄超过超时后清 speed_valid，不把缺帧当成零速。
 * @param {motor_encoder_runtime_t *} rt  运行时状态
 * @param {float} dt_s                    距上一拍时间 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_PredictState(motor_encoder_runtime_t *rt, float dt_s)
{
    float cpr;

    if (rt == NULL || !rt->have_last || dt_s < MOTOR_ENCODER_DT_MIN_S || dt_s > MOTOR_ENCODER_DT_MAX_S) {
        return;
    }
    if (!Motor_Encoder_PllPredict(rt, dt_s)) {
        return;
    }

    rt->edge_age_s += dt_s;
    rt->sample_age_s += dt_s;
    cpr = (float)((rt->counts_per_rev == 0U) ? 1U : rt->counts_per_rev);
    rt->mech_rpm_raw = rt->pll_vel / cpr * 60.0f;
    rt->mech_rpm = rt->mech_rpm_raw;
    if (rt->sample_age_s > MOTOR_ENCODER_SAMPLE_TIMEOUT_S) {
        rt->sample_stale = true;
        rt->speed_valid = false;
    }
}

/**
 * @description: 由原始计数计算单圈机械角（标定用，不含 PLL 插值）
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

float Motor_Encoder_GetRawMechRpmState(const motor_encoder_runtime_t *rt)
{
    return rt == NULL ? 0.0f : rt->mech_rpm_raw;
}

float Motor_Encoder_GetEdgeAgeState(const motor_encoder_runtime_t *rt)
{
    return rt == NULL ? 0.0f : rt->edge_age_s;
}

bool Motor_Encoder_IsSignalFaultState(const motor_encoder_runtime_t *rt)
{
    return rt != NULL && rt->signal_fault;
}

uint32_t Motor_Encoder_GetInvalidCountState(const motor_encoder_runtime_t *rt)
{
    return rt == NULL ? 0U : rt->invalid_count;
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
    rt->pll_inverted = rt->inverted;
    rt->pll_pos = 0.0f;
    rt->pll_vel = 0.0f;
    rt->edge_age_s = 0.0f;
    rt->sample_age_s = 0.0f;
    rt->mech_rpm_raw = 0.0f;
    rt->mech_rpm = 0.0f;
    rt->speed_valid = false;
    rt->signal_fault = false;
    rt->sample_stale = false;
}

/**
 * @description: 初始化 MT6701 绝对编码器与默认运行时实例
 * @detail: 初始化 I2C2 与 14 位角度运行时；首次成功采样前不建立有效零点
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Init(void)
{
    Motor_MT6701_Init();
    Motor_Encoder_RuntimeInit(&s_encoder, Hal_Encoder_CountsPerRev(), MOTOR_ENCODER_INVERTED);
    s_encoder_sample_sequence = 0U;
    s_encoder_sample_time_us = 0U;
    s_encoder_have_sequence = false;
    printf("Encoder MT6701 %s: %u count/rev\r\n", Motor_MT6701_InterfaceName(),
           (unsigned)Hal_Encoder_CountsPerRev());
}

/**
 * @description: 读取最近一次 MT6701 绝对角度原始值
 * @return {uint16_t} 14 位角度计数，范围 [0, 16384)
 * @author: lkc
 */
uint16_t Motor_Encoder_GetCount(void)
{
    return Hal_Encoder_GetCount();
}

/**
 * @description: 清除默认实例的电角偏置与速度历史
 * @detail: MT6701 是绝对角度传感器，复位软件状态不会也不应改写传感器角度
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Reset(void)
{
    Motor_Encoder_ClearZeroState(&s_encoder);
    s_encoder_sample_sequence = 0U;
    s_encoder_sample_time_us = 0U;
    s_encoder_have_sequence = false;
}

/**
 * @description: 用 HAL 当前计数与采样间隔更新默认实例转速
 * @param {float} dt_s  控制/采样周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Update(float dt_s)
{
    motor_mt6701_status_t mt;
    uint32_t sequence;
    float sample_dt_s;
    float correction_dt_s;

    Motor_MT6701_GetStatus(&mt);
    sequence = mt.sample_sequence;
    Motor_Encoder_PredictState(&s_encoder, dt_s);
    if (mt.valid && (!s_encoder_have_sequence || sequence != s_encoder_sample_sequence)) {
        sample_dt_s = (float)(uint32_t)(mt.sample_time_us - s_encoder_sample_time_us) * 1.0e-6f;
        s_encoder_sample_sequence = sequence;
        s_encoder_sample_time_us = mt.sample_time_us;
        s_encoder_have_sequence = true;
        if (!s_encoder.have_last || s_encoder.pll_inverted != s_encoder.inverted || s_encoder.sample_stale ||
            sample_dt_s < MOTOR_ENCODER_DT_MIN_S || sample_dt_s > MOTOR_ENCODER_DT_MAX_S) {
            Motor_Encoder_Seed(&s_encoder, Hal_Encoder_GetCount());
        } else {
            correction_dt_s = sample_dt_s;
            if (correction_dt_s > MOTOR_ENCODER_CORRECTION_DT_MAX_S) {
                correction_dt_s = MOTOR_ENCODER_CORRECTION_DT_MAX_S;
            }
            Motor_Encoder_CorrectFreshSample(&s_encoder, Hal_Encoder_GetCount(), sample_dt_s, correction_dt_s);
        }
    }
}

/**
 * @description: 读取默认实例最新已验收的机械角（不使用 PLL 预测位置）
 * @return {float} 机械角度 (rad)，[0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetMechAngle(void)
{
    uint16_t count = s_encoder.have_last ? s_encoder.last_count : Hal_Encoder_GetCount();

    return Motor_Encoder_GetMechAngleState(&s_encoder, count);
}

/**
 * @description: 读取默认实例当前电角
 * @detail: 极对数取自全局 motor.ol_pole_pairs；绝对有效须先 Detect
 * @return {float} 电角度 (rad)，[0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetElecAngle(void)
{
    uint16_t count = s_encoder.have_last ? s_encoder.last_count : Hal_Encoder_GetCount();

    /* 极对数下限由 GetElecAngleState 统一保证。 */
    return Motor_Encoder_GetElecAngleState(&s_encoder, count, motor.ol_pole_pairs);
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

float Motor_Encoder_GetRawMechRpm(void)
{
    return Motor_Encoder_GetRawMechRpmState(&s_encoder);
}

float Motor_Encoder_GetEdgeAge(void)
{
    return Motor_Encoder_GetEdgeAgeState(&s_encoder);
}

/**
 * @description: 读取默认实例距最近一次 SSI/I2C 新帧的时间。
 * @return {float} 采样年龄 (s)
 * @author: lkc
 */
float Motor_Encoder_GetSampleAge(void)
{
    return s_encoder.sample_age_s;
}

bool Motor_Encoder_IsSignalFault(void)
{
    return !Motor_MT6701_IsValid() || s_encoder.sample_stale || Motor_Encoder_IsSignalFaultState(&s_encoder);
}

uint32_t Motor_Encoder_GetInvalidCount(void)
{
    return Motor_Encoder_GetInvalidCountState(&s_encoder);
}

/**
 * @description: 查询默认实例本拍转速是否有效
 * @return {bool} true=速度可信
 * @author: lkc
 */
bool Motor_Encoder_IsSpeedValid(void)
{
    return Motor_MT6701_IsValid() && Motor_Encoder_IsSpeedValidState(&s_encoder);
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

#ifndef MOTOR_ENCODER_HOST_TEST
/**
 * @description: 查询编码器电角零点对齐是否正在占用控制输出。
 * @return {bool} true=对齐进行中，本拍输出由检测目标决定
 * @author: lkc
 */
bool Motor_Encoder_DetectIsActive(void)
{
    return s_encoder_detect.active;
}

/**
 * @description: 读取编码器对齐阶段的固定电角和 Id 目标。
 * @param {float *} theta_rad   输出电角度 (rad)，不可为空
 * @param {float *} id_target   输出 Id 目标 (A)，不可为空
 * @param {float *} iq_target   输出 Iq 目标 (A)，不可为空；对齐时为 0
 * @return {bool} true=对齐进行中且三个指针都已写出
 * @author: lkc
 */
bool Motor_Encoder_DetectGetTargets(float *theta_rad, float *id_target, float *iq_target)
{
    if (!s_encoder_detect.active || theta_rad == NULL || id_target == NULL || iq_target == NULL) {
        return false;
    }
    *theta_rad = s_encoder_detect.theta_rad;
    *id_target = s_encoder_detect.id_target;
    *iq_target = 0.0f;
    return true;
}

/**
 * @description: 把已保存的编码器零点和方向装回运行时。
 * @param {float} offset_rad  电角零点 (rad)；valid 为 false 时忽略
 * @param {bool} inverted     true=机械正方向与计数方向相反
 * @param {bool} valid        true=零点可用
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_ApplyCalibration(float offset_rad, bool inverted, bool valid)
{
    s_encoder.inverted = inverted;
    s_encoder.elec_offset_rad = valid ? Motor_Encoder_Wrap2Pi(offset_rad) : 0.0f;
    s_encoder.zero_valid = valid;
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
    float current = motor.ol_iq_set;
    int32_t direction_delta;
    uint16_t direction_start;
    float measured;
    float forced;
    float diff;
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    const uint32_t scan_points = 140U;
    const float scan_range = 2.0f * (float)M_PI * pole_pairs;
    uint32_t step;
    bool success = false;
    motor_detect_result_t detect = {0};

    if (!Motor_MT6701_IsValid()) {
        printf("Encoder detect rejected: MT6701 %s data invalid\r\n", Motor_MT6701_InterfaceName());
        return;
    }

    if (motor.ol_enable || Motor_Protect_IsFaultLatched() || current <= 0.0f || current > 0.5f) {
        printf("Encoder detect rejected: stop motor, clear fault, set oliq 0.1..0.5A\r\n");
        return;
    }

    printf("Encoder detect: align theta=0 Id=%.2fA\r\n", (double)current);
    Motor_Protect_ClearFault();
    motor.ol_enable = true;
    (void)Motor_ClearRuntimeControl(&motor);

    s_encoder_detect.id_target = 0.0f;
    s_encoder_detect.theta_rad = 0.0f;
    s_encoder_detect.active = true;
    Hal_Motor_PwmStart();

    for (step = 0U; step < 500U; step++) {
        if (Motor_Protect_IsFaultLatched()) {
            break;
        }
        s_encoder_detect.id_target = current * (float)(step + 1U) / 500.0f;
        osDelay(pdMS_TO_TICKS(1U));
    }
    if (!Motor_Protect_IsFaultLatched()) {
        osDelay(pdMS_TO_TICKS(700U));
        direction_start = Hal_Encoder_GetCount();
        for (step = 0U; step < 1000U; step++) {
            if (Motor_Protect_IsFaultLatched() || !Motor_MT6701_IsValid()) {
                break;
            }
            s_encoder_detect.theta_rad = (0.5f * (float)M_PI) * (float)(step + 1U) / 1000.0f;
            osDelay(pdMS_TO_TICKS(1U));
        }
        direction_delta = Motor_Encoder_DeltaCounts(Hal_Encoder_GetCount(), direction_start,
                                                     Hal_Encoder_CountsPerRev());
        if (!Motor_Protect_IsFaultLatched() && Motor_MT6701_IsValid() &&
            (direction_delta > 100 || direction_delta < -100)) {
            s_encoder.inverted = direction_delta < 0;
            printf("Encoder direction: delta=%ld inverted=%d\r\n", (long)direction_delta,
                   s_encoder.inverted ? 1 : 0);
            for (step = 1000U; step > 0U; step--) {
                if (Motor_Protect_IsFaultLatched()) {
                    break;
                }
                s_encoder_detect.theta_rad = (0.5f * (float)M_PI) * (float)(step - 1U) / 1000.0f;
                osDelay(pdMS_TO_TICKS(1U));
            }
            if (!Motor_Protect_IsFaultLatched()) {
                osDelay(pdMS_TO_TICKS(300U));
                s_encoder.elec_offset_rad = 0.0f;
                printf("Encoder offset scan: forward/back one mechanical turn\r\n");
                for (step = 0U; step < scan_points; step++) {
                    s_encoder_detect.theta_rad = scan_range * (float)(step + 1U) / (float)scan_points;
                    osDelay(pdMS_TO_TICKS(50U));
                    if (Motor_Protect_IsFaultLatched() || !Motor_MT6701_IsValid()) {
                        break;
                    }
                    measured = Motor_Encoder_GetMechAngleState(&s_encoder, Hal_Encoder_GetCount()) * pole_pairs;
                    forced = fmodf(s_encoder_detect.theta_rad, 2.0f * (float)M_PI);
                    diff = measured - forced;
                    sin_sum += sinf(diff);
                    cos_sum += cosf(diff);
                }
                for (step = scan_points; step > 0U && !Motor_Protect_IsFaultLatched(); step--) {
                    s_encoder_detect.theta_rad = scan_range * (float)(step - 1U) / (float)scan_points;
                    osDelay(pdMS_TO_TICKS(50U));
                    if (!Motor_MT6701_IsValid()) {
                        break;
                    }
                    measured = Motor_Encoder_GetMechAngleState(&s_encoder, Hal_Encoder_GetCount()) * pole_pairs;
                    forced = fmodf(s_encoder_detect.theta_rad, 2.0f * (float)M_PI);
                    diff = measured - forced;
                    sin_sum += sinf(diff);
                    cos_sum += cosf(diff);
                }
                success = !Motor_Protect_IsFaultLatched() && Motor_MT6701_IsValid() &&
                          (fabsf(sin_sum) + fabsf(cos_sum) > 1.0f);
                if (success) {
                    s_encoder.elec_offset_rad = Motor_Encoder_Wrap2Pi(atan2f(sin_sum, cos_sum));
                    s_encoder.zero_valid = true;
                    printf("Encoder offset average: %.3f rad (%.1f deg)\r\n", (double)s_encoder.elec_offset_rad,
                           (double)(s_encoder.elec_offset_rad * 180.0f / (float)M_PI));
                }
            }
        } else {
            printf("Encoder direction FAILED: movement delta=%ld\r\n", (long)direction_delta);
        }
    }

    s_encoder_detect.active = false;
    s_encoder_detect.theta_rad = 0.0f;
    s_encoder_detect.id_target = 0.0f;
    (void)Motor_RequestStop(&motor);
    Hal_Motor_PwmStop();
    (void)Motor_ClearRuntimeControl(&motor);

    if (!success) {
        printf("Encoder detect FAILED: %s\r\n", Motor_Protect_GetFaultName(Motor_Protect_GetFault()));
        return;
    }

    motor.encoder_offset_rad = s_encoder.elec_offset_rad;
    motor.encoder_inverted = s_encoder.inverted;
    motor.encoder_calibrated = true;
    detect.version = MOTOR_DETECT_RESULT_VERSION;
    if (FlashStorage_Save(&motor, &detect)) {
        printf("Encoder calibration saved to flash\r\n");
    } else {
        printf("Encoder calibration save FAILED\r\n");
    }

    printf("Encoder detect: count=%u mech=%.3f rad elec0=%.3f rad zero_ok=%d\r\n",
           (unsigned)Motor_Encoder_GetCount(), Motor_Encoder_GetMechAngle(), Motor_Encoder_GetElecAngle(),
           Motor_Encoder_IsZeroValid() ? 1 : 0);
}
#endif

#ifdef __cplusplus
}
#endif
