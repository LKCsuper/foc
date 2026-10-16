/*
 * @Description: 龙伯格反电势观测器（αβ 静止坐标系）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_luenberger.h"
#include "common_assert.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_LUENBERGER_PI (3.14159265358979323846f)
#define MOTOR_LUENBERGER_TWO_PI (2.0f * MOTOR_LUENBERGER_PI)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static float Motor_Luenberger_WrapPi(float angle);
static void Motor_Luenberger_ClearOutput(motor_luenberger_output_t *output);
static void Motor_Luenberger_ResolveGains(const motor_luenberger_config_t *config, float *current_gain,
                                          float *bemf_gain);
static void Motor_Luenberger_Publish(motor_luenberger_t *obs, const motor_luenberger_config_t *config, float e_alpha_old,
                                     float e_beta_old, motor_luenberger_output_t *output);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将角度循环归一化到 [−π, π)
 * @param {float} angle  待归一化角度 (rad)
 * @return {float} [−π, π)
 * @author: lkc
 */
static float Motor_Luenberger_WrapPi(float angle)
{
    while (angle >= MOTOR_LUENBERGER_PI) {
        angle -= MOTOR_LUENBERGER_TWO_PI;
    }
    while (angle < -MOTOR_LUENBERGER_PI) {
        angle += MOTOR_LUENBERGER_TWO_PI;
    }
    return angle;
}

/**
 * @description: 清空本拍输出
 * @param {motor_luenberger_output_t} *output  输出；可为 NULL
 * @return {*}
 * @author: lkc
 */
static void Motor_Luenberger_ClearOutput(motor_luenberger_output_t *output)
{
    if (output != NULL) {
        memset(output, 0, sizeof(*output));
    }
}

/**
 * @description: 由配置得到 l1、l2；未填写时按默认 ωn、ζ 整定
 * @param {const motor_luenberger_config_t} *config  配置
 * @param {float} *current_gain                      输出 l1 (1/s)
 * @param {float} *bemf_gain                         输出 l2 (V/(A·s))
 * @return {*}
 * @author: lkc
 */
static void Motor_Luenberger_ResolveGains(const motor_luenberger_config_t *config, float *current_gain, float *bemf_gain)
{
    float inductance = config->inductance;
    float wn;
    float l1;

    if (config->current_gain > 0.0f && config->bemf_gain > 0.0f) {
        *current_gain = config->current_gain;
        *bemf_gain = config->bemf_gain;
        return;
    }

    wn = MOTOR_LUENBERGER_DEFAULT_WN;
    l1 = 2.0f * MOTOR_LUENBERGER_DEFAULT_ZETA * wn - config->resistance / inductance;
    *current_gain = l1 > 0.0f ? l1 : 0.0f;
    *bemf_gain = wn * wn * inductance;
}

/**
 * @description: 由当前 ê 计算模长、电角、电角速度，并写入状态与输出
 * @param {motor_luenberger_t} *obs                  实例
 * @param {const motor_luenberger_config_t} *config  配置
 * @param {float} e_alpha_old                        积分前 êα (V)
 * @param {float} e_beta_old                         积分前 êβ (V)
 * @param {motor_luenberger_output_t} *output        本拍输出
 * @return {*}
 * @author: lkc
 */
static void Motor_Luenberger_Publish(motor_luenberger_t *obs, const motor_luenberger_config_t *config, float e_alpha_old,
                                     float e_beta_old, motor_luenberger_output_t *output)
{
    float flux = config->flux_linkage;
    float cross;
    float angle_raw;

    obs->magnitude = sqrtf(obs->e_alpha * obs->e_alpha + obs->e_beta * obs->e_beta);
    cross = e_alpha_old * obs->e_beta - e_beta_old * obs->e_alpha;
    if (obs->magnitude > 0.0f && flux >= MOTOR_LUENBERGER_MIN_LINKAGE) {
        if (fabsf(cross) > 1.0e-12f) {
            obs->speed_e = (cross >= 0.0f ? 1.0f : -1.0f) * (obs->magnitude / flux);
        } else {
            obs->speed_e = (obs->speed_e >= 0.0f ? 1.0f : -1.0f) * (obs->magnitude / flux);
        }
    } else {
        obs->speed_e = 0.0f;
    }

    /* 正转 e=ωψ[−sin, cos] → θ=atan2(−eα, eβ)；反转矢量反向，用 −e。 */
    if (obs->speed_e >= 0.0f) {
        angle_raw = atan2f(-obs->e_alpha, obs->e_beta);
    } else {
        angle_raw = atan2f(obs->e_alpha, -obs->e_beta);
    }
    obs->angle = Motor_Luenberger_WrapPi(angle_raw + config->phase_offset);

    output->valid = true;
    output->bemf_ok = obs->magnitude > MOTOR_LUENBERGER_MIN_BEMF;
    output->e_alpha = obs->e_alpha;
    output->e_beta = obs->e_beta;
    output->magnitude = obs->magnitude;
    output->angle = obs->angle;
    output->speed_e = obs->speed_e;
}

/**
 * @description: 复位龙伯格观测状态
 * @param {motor_luenberger_t} *obs  实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Luenberger_Reset(motor_luenberger_t *obs)
{
    if (obs != NULL) {
        memset(obs, 0, sizeof(*obs));
    }
}

/**
 * @description: 规整龙伯格观测配置
 * @param {motor_luenberger_config_t} *config  配置，被原地修改；可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Luenberger_NormalizeConfig(motor_luenberger_config_t *config)
{
    COMMON_ASSERT(config != NULL);
    if (!(config->resistance >= 0.0f) || !isfinite(config->resistance)) {
        config->resistance = 0.0f;
    }
    if (!isfinite(config->inductance) || config->inductance < MOTOR_LUENBERGER_MIN_INDUCTANCE) {
        config->inductance = MOTOR_LUENBERGER_MIN_INDUCTANCE;
    }
    if (!isfinite(config->flux_linkage) || config->flux_linkage < MOTOR_LUENBERGER_MIN_LINKAGE) {
        config->flux_linkage = MOTOR_LUENBERGER_MIN_LINKAGE;
    }
    if (!isfinite(config->current_gain) || config->current_gain < 0.0f) {
        config->current_gain = 0.0f;
    }
    if (!isfinite(config->bemf_gain) || config->bemf_gain < 0.0f) {
        config->bemf_gain = 0.0f;
    }
    if (!isfinite(config->phase_offset)) {
        config->phase_offset = 0.0f;
    }
}

/**
 * @description: 用当前电流、假设电角和电角速度初始化观测器
 * @param {motor_luenberger_t} *obs                  实例
 * @param {const motor_luenberger_config_t} *config  配置
 * @param {float} angle                              假设电角 (rad)
 * @param {float} omega_e                            假设电角速度 (rad/s)
 * @param {float} i_alpha                            α 轴电流 (A)
 * @param {float} i_beta                             β 轴电流 (A)
 * @return {*}
 * @author: lkc
 */
void Motor_Luenberger_Seed(motor_luenberger_t *obs, const motor_luenberger_config_t *config, float angle, float omega_e,
                           float i_alpha, float i_beta)
{
    float flux;
    float wrapped;
    float s;
    float c;

    COMMON_ASSERT(obs != NULL && config != NULL);
    if (!isfinite(angle) || !isfinite(omega_e) || !isfinite(i_alpha) || !isfinite(i_beta)) return;

    flux = config->flux_linkage >= MOTOR_LUENBERGER_MIN_LINKAGE ? config->flux_linkage : MOTOR_LUENBERGER_MIN_LINKAGE;
    wrapped = Motor_Luenberger_WrapPi(angle);
    s = sinf(wrapped);
    c = cosf(wrapped);

    obs->i_hat_alpha = i_alpha;
    obs->i_hat_beta = i_beta;
    obs->e_alpha = -omega_e * flux * s;
    obs->e_beta = omega_e * flux * c;
    obs->magnitude = fabsf(omega_e) * flux;
    obs->speed_e = omega_e;
    obs->angle = Motor_Luenberger_WrapPi(wrapped + config->phase_offset);
    obs->initialized = true;
}

/**
 * @description: 推进一拍龙伯格反电势观测
 * @param {motor_luenberger_t} *obs                  实例
 * @param {const motor_luenberger_config_t} *config  配置
 * @param {float} v_alpha                            α 轴电压 (V)
 * @param {float} v_beta                             β 轴电压 (V)
 * @param {float} i_alpha                            α 轴电流 (A)
 * @param {float} i_beta                             β 轴电流 (A)
 * @param {float} dt                                 控制周期 (s)
 * @param {motor_luenberger_output_t} *output        本拍输出
 * @return {bool} true=本拍观测有效
 * @author: lkc
 */
bool Motor_Luenberger_Step(motor_luenberger_t *obs, const motor_luenberger_config_t *config, float v_alpha, float v_beta,
                           float i_alpha, float i_beta, float dt, motor_luenberger_output_t *output)
{
    float resistance;
    float inductance;
    float current_gain;
    float bemf_gain;
    float inv_l;
    float eps_alpha;
    float eps_beta;
    float e_alpha_old;
    float e_beta_old;
    float di_hat_alpha;
    float di_hat_beta;
    float de_alpha;
    float de_beta;

    Motor_Luenberger_ClearOutput(output);
    if (obs == NULL || config == NULL || output == NULL) return false;
    if (!config->enabled || !(dt > 0.0f) || !isfinite(dt) || !isfinite(v_alpha) || !isfinite(v_beta) ||
        !isfinite(i_alpha) || !isfinite(i_beta) || !(config->inductance >= MOTOR_LUENBERGER_MIN_INDUCTANCE)) {
        Motor_Luenberger_Reset(obs);
        return false;
    }

    resistance = config->resistance >= 0.0f ? config->resistance : 0.0f;
    inductance = config->inductance;
    Motor_Luenberger_ResolveGains(config, &current_gain, &bemf_gain);
    inv_l = 1.0f / inductance;

    if (!obs->initialized) {
        Motor_Luenberger_Seed(obs, config, 0.0f, 0.0f, i_alpha, i_beta);
    }

    /*
     * 电流误差 ε = i − î。
     * ê 偏小 → 实际 di/dt 更负 → i 落到 î 下面 → ε<0 → dê = −l2·ε 把 ê 抬上去。
     * 上一拍 ω̂ 用来旋转 ê，补偿反电势本身在转、不是常值扰动。
     */
    eps_alpha = i_alpha - obs->i_hat_alpha;
    eps_beta = i_beta - obs->i_hat_beta;
    di_hat_alpha = inv_l * (v_alpha - resistance * obs->i_hat_alpha - obs->e_alpha) + current_gain * eps_alpha;
    di_hat_beta = inv_l * (v_beta - resistance * obs->i_hat_beta - obs->e_beta) + current_gain * eps_beta;
    de_alpha = -obs->speed_e * obs->e_beta - bemf_gain * eps_alpha;
    de_beta = obs->speed_e * obs->e_alpha - bemf_gain * eps_beta;

    e_alpha_old = obs->e_alpha;
    e_beta_old = obs->e_beta;
    obs->i_hat_alpha += di_hat_alpha * dt;
    obs->i_hat_beta += di_hat_beta * dt;
    obs->e_alpha += de_alpha * dt;
    obs->e_beta += de_beta * dt;

    if (!isfinite(obs->i_hat_alpha) || !isfinite(obs->i_hat_beta) || !isfinite(obs->e_alpha) ||
        !isfinite(obs->e_beta)) {
        Motor_Luenberger_Reset(obs);
        return false;
    }

    Motor_Luenberger_Publish(obs, config, e_alpha_old, e_beta_old, output);
    if (!isfinite(obs->magnitude) || !isfinite(obs->angle) || !isfinite(obs->speed_e)) {
        Motor_Luenberger_Reset(obs);
        Motor_Luenberger_ClearOutput(output);
        return false;
    }
    return true;
}

/**
 * @description: 读取原始观测电角
 * @param {const motor_luenberger_t} *obs  实例；NULL 返回 0
 * @return {float} 电角 (rad)
 * @author: lkc
 */
float Motor_Luenberger_GetAngle(const motor_luenberger_t *obs)
{
    return obs == NULL ? 0.0f : obs->angle;
}

/**
 * @description: 读取估算反电势模长
 * @param {const motor_luenberger_t} *obs  实例；NULL 返回 0
 * @return {float} |ê| (V)
 * @author: lkc
 */
float Motor_Luenberger_GetMagnitude(const motor_luenberger_t *obs)
{
    return obs == NULL ? 0.0f : obs->magnitude;
}

/**
 * @description: 读取估算电角速度
 * @param {const motor_luenberger_t} *obs  实例；NULL 返回 0
 * @return {float} 电角速度 (rad/s)
 * @author: lkc
 */
float Motor_Luenberger_GetSpeedE(const motor_luenberger_t *obs)
{
    return obs == NULL ? 0.0f : obs->speed_e;
}

#ifdef __cplusplus
}
#endif
