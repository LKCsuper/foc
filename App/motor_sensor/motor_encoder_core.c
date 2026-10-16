/* 编码器纯算法：绝对计数、回绕、PLL、零点；不访问硬件或全局电机。 */
#include "motor_encoder_core.h"
#include "common_assert.h"
#include <math.h>
#include <stddef.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define MOTOR_ENCODER_DT_MIN_S (1.0e-6f)
#define MOTOR_ENCODER_DT_MAX_S (0.1f)
static float Motor_Encoder_Wrap2Pi(float angle);
static float Motor_Encoder_WrapCpr(float pos, float cpr);
static float Motor_Encoder_WrapDeltaCpr(float delta, float cpr);
static float Motor_Encoder_MeasuredCounts(const motor_encoder_runtime_t *rt, uint16_t count);
static bool Motor_Encoder_PllPredict(motor_encoder_runtime_t *rt, float dt_s);
static void Motor_Encoder_PllCorrect(motor_encoder_runtime_t *rt, float measured, float dt_s);
static int32_t Motor_Encoder_DeltaCounts(uint16_t now, uint16_t last, uint32_t cpr);
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

void Motor_Encoder_ApplyCalibrationState(motor_encoder_runtime_t *rt, float offset_rad, bool inverted, bool valid)
{
    COMMON_ASSERT(rt != NULL);
    rt->inverted = inverted;
    rt->elec_offset_rad = valid ? Motor_Encoder_Wrap2Pi(offset_rad) : 0.0f;
    rt->zero_valid = valid;
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
    if (cpr <= 0.0f) return 0.0f;
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
 * @description: 用当前计数重锚 PLL，本拍速度无效。
 * @detail: 不改累计机械计数。缺帧或非法周期里的位移补不回来，但位置环零点不会被清成 0。
 * @param {motor_encoder_runtime_t *} rt  运行时状态
 * @param {uint16_t} count               原始计数
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_SeedState(motor_encoder_runtime_t *rt, uint16_t count)
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
 * @description: 按当前 PLL 速度外推一拍位置。
 * @detail: dt·kp≥1 时离散化不稳定，清速度并返回 false，调用方不再校正。
 * @param {motor_encoder_runtime_t *} rt  运行时状态
 * @param {float} dt_s                    预测步长 (s)
 * @return {bool} true=位置已外推；false=步长过大，速度已清零
 * @author: lkc
 */
static bool Motor_Encoder_PllPredict(motor_encoder_runtime_t *rt, float dt_s)
{
    const float kp = 2.0f * rt->config.pll_bandwidth_rad_s;
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

/**
 * @description: 用测量位置校正 ODrive 临界阻尼 PLL。
 * @detail: kp=2·bw，ki=0.25·kp²。误差按连续位置折到 ±cpr/2，不用取整。
 *          |vel| < 0.5·dt·ki 时打成 0，静止不留量化残差。mech_rpm_raw 取吸附前的速度。
 * @param {motor_encoder_runtime_t *} rt  运行时状态
 * @param {float} measured               本拍测量位置 (count)
 * @param {float} dt_s                   校正步长 (s)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Encoder_PllCorrect(motor_encoder_runtime_t *rt, float measured, float dt_s)
{
    const float kp = 2.0f * rt->config.pll_bandwidth_rad_s;
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

    vel_max = rt->config.max_rpm * cpr / 60.0f;
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
 * @detail: 清零点与速度历史；非法计数范围或配置返回false且保留原状态
 * @param {motor_encoder_runtime_t} *rt  运行时状态，不可为空
 * @param {uint32_t} counts_per_rev      每机械圈计数（含正交倍频）
 * @param {bool} inverted                true=机械正方向与计数方向相反
 * @return {*}
 * @author: lkc
 */
bool Motor_Encoder_InitState(motor_encoder_runtime_t *rt, uint32_t counts_per_rev, bool inverted,
                             const motor_encoder_config_t *config)
{
    if (rt == NULL || config == NULL || counts_per_rev == 0U || counts_per_rev > 65536U ||
        !isfinite(config->pll_bandwidth_rad_s) || config->pll_bandwidth_rad_s <= 0.0f ||
        !isfinite(config->max_rpm) || config->max_rpm <= 0.0f ||
        !isfinite(config->jump_margin_counts) || config->jump_margin_counts < 0.0f ||
        !isfinite(config->sample_timeout_s) || config->sample_timeout_s <= 0.0f) {
        return false;
    }
    rt->config = *config;
    rt->counts_per_rev = (counts_per_rev == 0U) ? 1U : counts_per_rev;
    rt->inverted = inverted;
    rt->elec_offset_rad = 0.0f;
    rt->zero_valid = false;
    rt->last_count = 0U;
    rt->accumulated_count = 0U;
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
    return true;
}

/**
 * @description: 用新计数更新 ODrive PLL 的位置与转速
 * @detail: 不修改电角零点；dt 过小/过大或首拍时 speed_valid=false。PLL带宽由实例配置指定。
 * @param {motor_encoder_runtime_t} *rt  运行时状态
 * @param {uint16_t} count               当前正交解码计数值
 * @param {float} dt_s                   距上一拍时间 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_UpdateState(motor_encoder_runtime_t *rt, uint16_t count, float dt_s)
{
    COMMON_ASSERT(rt != NULL);

    if (!rt->have_last || rt->pll_inverted != rt->inverted || dt_s < MOTOR_ENCODER_DT_MIN_S ||
        dt_s > MOTOR_ENCODER_DT_MAX_S || (dt_s * 2.0f * rt->config.pll_bandwidth_rad_s) >= 1.0f) {
        Motor_Encoder_SeedState(rt, count);
        return;
    }

    Motor_Encoder_PredictState(rt, dt_s);
    Motor_Encoder_CorrectSampleState(rt, count, dt_s, dt_s);
}

/**
 * @description: 验收一帧新计数并校正 PLL。
 * @detail: 计数突跳超过本拍转速能力加余量时记信号故障，不覆盖 last_count，也不拉 PLL。
 * @param {motor_encoder_runtime_t *} rt  运行时状态
 * @param {uint16_t} count               新的原始计数
 * @param {float} sample_dt_s           距上一帧成功采样的时间 (s)，用于突跳门限
 * @param {float} correction_dt_s       PLL 校正步长 (s)，可小于 sample_dt_s
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_CorrectSampleState(motor_encoder_runtime_t *rt, uint16_t count, float sample_dt_s,
                                             float correction_dt_s)
{
    int32_t delta;
    float max_delta;

    delta = Motor_Encoder_DeltaCounts(count, rt->last_count, rt->counts_per_rev);
    if (rt->inverted) {
        delta = -delta;
    }
    /* 允许量化产生的 1~2 count，但拒绝超过 MT6701 转速能力的突跳。坏帧不拉 PLL。 */
    max_delta = rt->config.max_rpm * (float)rt->counts_per_rev * sample_dt_s / 60.0f;
    max_delta += rt->config.jump_margin_counts;
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
    rt->accumulated_count += (uint32_t)delta;
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
void Motor_Encoder_PredictWithTimeoutState(motor_encoder_runtime_t *rt, float dt_s, float timeout_s)
{
    float cpr;

    COMMON_ASSERT(rt != NULL);
    if (!rt->have_last || dt_s < MOTOR_ENCODER_DT_MIN_S || dt_s > MOTOR_ENCODER_DT_MAX_S) return;
    if (!Motor_Encoder_PllPredict(rt, dt_s)) return;

    rt->edge_age_s += dt_s;
    rt->sample_age_s += dt_s;
    cpr = (float)((rt->counts_per_rev == 0U) ? 1U : rt->counts_per_rev);
    rt->mech_rpm_raw = rt->pll_vel / cpr * 60.0f;
    rt->mech_rpm = rt->mech_rpm_raw;
    if (rt->sample_age_s > timeout_s) {
        rt->sample_stale = true;
        rt->speed_valid = false;
    }
}

void Motor_Encoder_PredictState(motor_encoder_runtime_t *rt, float dt_s)
{
    Motor_Encoder_PredictWithTimeoutState(rt, dt_s, rt->config.sample_timeout_s);
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

    if (rt == NULL) return 0.0f;
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

    if (rt == NULL) return 0.0f;
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
 * @description: 读取近零吸附前的机械转速。
 * @param {const motor_encoder_runtime_t *} rt  运行时状态
 * @return {float} 原始机械转速 (RPM)；rt 为空返回 0
 * @author: lkc
 */
float Motor_Encoder_GetRawMechRpmState(const motor_encoder_runtime_t *rt)
{
    return rt == NULL ? 0.0f : rt->mech_rpm_raw;
}

/**
 * @description: 读取距最近一次计数变化的时间。
 * @param {const motor_encoder_runtime_t *} rt  运行时状态
 * @return {float} 边沿年龄 (s)；rt 为空返回 0
 * @author: lkc
 */
float Motor_Encoder_GetEdgeAgeState(const motor_encoder_runtime_t *rt)
{
    return rt == NULL ? 0.0f : rt->edge_age_s;
}

/**
 * @description: 查询是否检测到不合理计数跳变。
 * @param {const motor_encoder_runtime_t *} rt  运行时状态
 * @return {bool} true=当前信号故障；rt 为空返回 false
 * @author: lkc
 */
bool Motor_Encoder_IsSignalFaultState(const motor_encoder_runtime_t *rt)
{
    return rt != NULL && rt->signal_fault;
}

/**
 * @description: 读取累计被拒绝的异常样本数。
 * @param {const motor_encoder_runtime_t *} rt  运行时状态
 * @return {uint32_t} 被拒绝样本数；rt 为空返回 0
 * @author: lkc
 */
uint32_t Motor_Encoder_GetInvalidCountState(const motor_encoder_runtime_t *rt)
{
    return rt == NULL ? 0U : rt->invalid_count;
}

/**
 * @description: 读取每帧验收后累计的机械计数。
 * @detail: 正反方向已按 inverted 统一。返回值按 uint32_t 自然回绕，两次读数作 int32_t 差可得到短窗口增量。
 * @param {const motor_encoder_runtime_t *} rt  运行时状态
 * @return {uint32_t} 累计机械计数（模 2^32）；rt 为空返回 0
 * @author: lkc
 */
uint32_t Motor_Encoder_GetAccumulatedCountState(const motor_encoder_runtime_t *rt)
{
    return rt == NULL ? 0U : rt->accumulated_count;
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
    COMMON_ASSERT(rt != NULL);
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
    COMMON_ASSERT(rt != NULL);
    rt->elec_offset_rad = 0.0f;
    rt->zero_valid = false;
    rt->have_last = false;
    rt->accumulated_count = 0U;
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

