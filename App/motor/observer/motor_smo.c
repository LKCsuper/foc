/*
 * @Description: 滑模反电势观测器（αβ 静止坐标系，饱和函数）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_smo.h"
#include "common_assert.h"
#include "motor_foc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
#include "motor_config.h"
#if MOTOR_CFG_SENSORLESS
/* Private define ------------------------------------------------------------*/
#define MOTOR_SMO_PI (3.14159265358979323846f)
#define MOTOR_SMO_TWO_PI (2.0f * MOTOR_SMO_PI)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static float Motor_Smo_WrapPi(float angle);
static float Motor_Smo_Sigmoid(float error, float eps);
static void Motor_Smo_ClearOutput(motor_smo_output_t *output);
static void Motor_Smo_Publish(motor_smo_t *obs, const motor_smo_config_t *config, float e_alpha_old, float e_beta_old,
                              motor_smo_output_t *output);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将角度循环归一化到 [−π, π)
 * @param {float} angle  待归一化角度 (rad)
 * @return {float} [−π, π)
 * @author: lkc
 */
static float Motor_Smo_WrapPi(float angle)
{
    while (angle >= MOTOR_SMO_PI) {
        angle -= MOTOR_SMO_TWO_PI;
    }
    while (angle < -MOTOR_SMO_PI) {
        angle += MOTOR_SMO_TWO_PI;
    }
    return angle;
}

/**
 * @description: 饱和函数 s / (|s|+ε)，逼近 sign 但不跳变
 * @param {float} error  电流误差 î−i (A)
 * @param {float} eps    边界 (A)，须 >0
 * @return {float} (−1, 1)
 * @author: lkc
 */
static float Motor_Smo_Sigmoid(float error, float eps)
{
    return error / (fabsf(error) + eps);
}

/**
 * @description: 清空本拍输出
 * @param {motor_smo_output_t} *output  输出；可为 NULL
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Smo_ClearOutput(motor_smo_output_t *output)
{
    if (output != NULL) {
        memset(output, 0, sizeof(*output));
    }
}

/**
 * @description: 由当前 ê 计算模长、电角、电角速度，并写入状态与输出
 * @param {motor_smo_t} *obs                  实例
 * @param {const motor_smo_config_t} *config  配置
 * @param {float} e_alpha_old                 旋转前 êα (V)
 * @param {float} e_beta_old                  旋转前 êβ (V)
 * @param {motor_smo_output_t} *output        本拍输出
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Smo_Publish(motor_smo_t *obs, const motor_smo_config_t *config, float e_alpha_old, float e_beta_old,
                              motor_smo_output_t *output)
{
    float flux = config->flux_linkage;
    float cross;
    float angle_raw;

    obs->magnitude = sqrtf(obs->e_alpha * obs->e_alpha + obs->e_beta * obs->e_beta);
    cross = e_alpha_old * obs->e_beta - e_beta_old * obs->e_alpha;
    if (obs->magnitude > 0.0f && flux >= MOTOR_SMO_MIN_LINKAGE) {
        if (fabsf(cross) > 1.0e-12f) {
            obs->speed_e = (cross >= 0.0f ? 1.0f : -1.0f) * (obs->magnitude / flux);
        } else {
            obs->speed_e = (obs->speed_e >= 0.0f ? 1.0f : -1.0f) * (obs->magnitude / flux);
        }
    } else {
        obs->speed_e = 0.0f;
    }

    if (obs->speed_e >= 0.0f) {
        angle_raw = atan2f(-obs->e_alpha, obs->e_beta);
    } else {
        angle_raw = atan2f(obs->e_alpha, -obs->e_beta);
    }
    obs->angle = Motor_Smo_WrapPi(angle_raw + config->phase_offset);

    output->valid = true;
    output->bemf_ok = obs->magnitude > MOTOR_SMO_MIN_BEMF;
    output->e_alpha = obs->e_alpha;
    output->e_beta = obs->e_beta;
    output->magnitude = obs->magnitude;
    output->angle = obs->angle;
    output->speed_e = obs->speed_e;
}

/**
 * @description: 复位滑模观测状态
 * @param {motor_smo_t} *obs  实例，可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_Smo_Reset(motor_smo_t *obs)
{
    if (obs != NULL) {
        memset(obs, 0, sizeof(*obs));
    }
}

/**
 * @description: 规整滑模观测配置
 * @param {motor_smo_config_t} *config  配置，被原地修改；可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_Smo_NormalizeConfig(motor_smo_config_t *config)
{
    COMMON_ASSERT(config != NULL);
    if (!(config->resistance >= 0.0f) || !isfinite(config->resistance)) {
        config->resistance = 0.0f;
    }
    if (!isfinite(config->inductance) || config->inductance < MOTOR_SMO_MIN_INDUCTANCE) {
        config->inductance = MOTOR_SMO_MIN_INDUCTANCE;
    }
    if (!isfinite(config->flux_linkage) || config->flux_linkage < MOTOR_SMO_MIN_LINKAGE) {
        config->flux_linkage = MOTOR_SMO_MIN_LINKAGE;
    }
    if (!isfinite(config->slide_gain) || config->slide_gain < 0.0f) {
        config->slide_gain = 0.0f;
    }
    if (config->slide_gain > MOTOR_SMO_MAX_GAIN) {
        config->slide_gain = MOTOR_SMO_MAX_GAIN;
    }
    if (!isfinite(config->lpf) || config->lpf < 0.0f || config->lpf > 1.0f) {
        config->lpf = 0.0f;
    }
    if (!isfinite(config->sigmoid_eps) || config->sigmoid_eps < 0.0f) {
        config->sigmoid_eps = 0.0f;
    }
    if (!isfinite(config->phase_offset)) {
        config->phase_offset = 0.0f;
    }
}

/**
 * @description: 用当前电流、假设电角和电角速度初始化观测器
 * @param {motor_smo_t} *obs                  实例
 * @param {const motor_smo_config_t} *config  配置
 * @param {float} angle                       假设电角 (rad)
 * @param {float} omega_e                     假设电角速度 (rad/s)
 * @param {float} i_alpha                     α 轴电流 (A)
 * @param {float} i_beta                      β 轴电流 (A)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Smo_Seed(motor_smo_t *obs, const motor_smo_config_t *config, float angle, float omega_e, float i_alpha,
                    float i_beta)
{
    float flux;
    float wrapped;
    float s;
    float c;

    COMMON_ASSERT(obs != NULL && config != NULL);
    if (!isfinite(angle) || !isfinite(omega_e) || !isfinite(i_alpha) || !isfinite(i_beta)) return;

    flux = config->flux_linkage >= MOTOR_SMO_MIN_LINKAGE ? config->flux_linkage : MOTOR_SMO_MIN_LINKAGE;
    wrapped = Motor_Smo_WrapPi(angle);
    sincos_f32(wrapped, &s, &c);

    obs->i_hat_alpha = i_alpha;
    obs->i_hat_beta = i_beta;
    obs->e_alpha = -omega_e * flux * s;
    obs->e_beta = omega_e * flux * c;
    obs->z_alpha = 0.0f;
    obs->z_beta = 0.0f;
    obs->magnitude = fabsf(omega_e) * flux;
    obs->speed_e = omega_e;
    obs->angle = Motor_Smo_WrapPi(wrapped + config->phase_offset);
    obs->initialized = true;
}

/**
 * @description: 推进一拍滑模反电势观测
 * @param {motor_smo_t} *obs                  实例
 * @param {const motor_smo_config_t} *config  配置
 * @param {float} v_alpha                     α 轴电压 (V)
 * @param {float} v_beta                      β 轴电压 (V)
 * @param {float} i_alpha                     α 轴电流 (A)
 * @param {float} i_beta                      β 轴电流 (A)
 * @param {float} dt                          控制周期 (s)
 * @param {motor_smo_output_t} *output        本拍输出
 * @return {bool} true=本拍观测有效
 * @author: lkc
 */
bool Motor_Smo_Step(motor_smo_t *obs, const motor_smo_config_t *config, float v_alpha, float v_beta, float i_alpha,
                    float i_beta, float dt, motor_smo_output_t *output)
{
    float resistance;
    float inductance;
    float inv_l;
    float gain;
    float lpf;
    float eps;
    float e_alpha_old;
    float e_beta_old;
    float rot_delta;
    float rot_c;
    float rot_s;

    Motor_Smo_ClearOutput(output);
    if (obs == NULL || config == NULL || output == NULL) return false;
    if (!config->enabled || !(dt > 0.0f) || !isfinite(dt) || !isfinite(v_alpha) || !isfinite(v_beta) ||
        !isfinite(i_alpha) || !isfinite(i_beta) || !(config->inductance >= MOTOR_SMO_MIN_INDUCTANCE)) {
        Motor_Smo_Reset(obs);
        return false;
    }

    resistance = config->resistance >= 0.0f ? config->resistance : 0.0f;
    inductance = config->inductance;
    inv_l = 1.0f / inductance;
    gain = config->slide_gain > 0.0f ? config->slide_gain : MOTOR_SMO_DEFAULT_GAIN;
    if (gain > MOTOR_SMO_MAX_GAIN) {
        gain = MOTOR_SMO_MAX_GAIN;
    }
    lpf = (config->lpf > 0.0f && config->lpf <= 1.0f) ? config->lpf : MOTOR_SMO_DEFAULT_LPF;
    eps = config->sigmoid_eps > 0.0f ? config->sigmoid_eps : MOTOR_SMO_DEFAULT_EPS;

    if (!obs->initialized) {
        Motor_Smo_Seed(obs, config, 0.0f, 0.0f, i_alpha, i_beta);
    }

    /*
     * s = î − i，z = k·sat(s)。
     * 电流方程只放 ê：15 kHz 上若再减满幅 z，dt·k/L 会大于 1，î 把误差放大。
     * s=0 时 z=0，ê 只按 ω̂ 旋转，Seed 的反电势不会被首拍清掉。
     */
    obs->z_alpha = gain * Motor_Smo_Sigmoid(obs->i_hat_alpha - i_alpha, eps);
    obs->z_beta = gain * Motor_Smo_Sigmoid(obs->i_hat_beta - i_beta, eps);
    obs->i_hat_alpha += dt * inv_l * (v_alpha - resistance * obs->i_hat_alpha - obs->e_alpha);
    obs->i_hat_beta += dt * inv_l * (v_beta - resistance * obs->i_hat_beta - obs->e_beta);

    e_alpha_old = obs->e_alpha;
    e_beta_old = obs->e_beta;
    rot_delta = obs->speed_e * dt;
    sincos_f32(rot_delta, &rot_s, &rot_c);
    /* 旋转矩阵推 ê；z 是电流残差，lpf 约 0.002，避免把满幅 k 当成反电势低通。 */
    obs->e_alpha = e_alpha_old * rot_c - e_beta_old * rot_s + lpf * obs->z_alpha;
    obs->e_beta = e_alpha_old * rot_s + e_beta_old * rot_c + lpf * obs->z_beta;

    if (!isfinite(obs->i_hat_alpha) || !isfinite(obs->i_hat_beta) || !isfinite(obs->e_alpha) ||
        !isfinite(obs->e_beta)) {
        Motor_Smo_Reset(obs);
        return false;
    }

    Motor_Smo_Publish(obs, config, e_alpha_old, e_beta_old, output);
    if (!isfinite(obs->magnitude) || !isfinite(obs->angle) || !isfinite(obs->speed_e)) {
        Motor_Smo_Reset(obs);
        Motor_Smo_ClearOutput(output);
        return false;
    }
    return true;
}

/**
 * @description: 读取原始观测电角
 * @param {const motor_smo_t} *obs  实例；NULL 返回 0
 * @return {float} 电角 (rad)
 * @author: lkc
 */
float Motor_Smo_GetAngle(const motor_smo_t *obs)
{
    return obs == NULL ? 0.0f : obs->angle;
}

/**
 * @description: 读取估算反电势模长
 * @param {const motor_smo_t} *obs  实例；NULL 返回 0
 * @return {float} |ê| (V)
 * @author: lkc
 */
float Motor_Smo_GetMagnitude(const motor_smo_t *obs)
{
    return obs == NULL ? 0.0f : obs->magnitude;
}

/**
 * @description: 读取估算电角速度
 * @param {const motor_smo_t} *obs  实例；NULL 返回 0
 * @return {float} 电角速度 (rad/s)
 * @author: lkc
 */
float Motor_Smo_GetSpeedE(const motor_smo_t *obs)
{
    return obs == NULL ? 0.0f : obs->speed_e;
}

#endif /* MOTOR_CFG_SENSORLESS */
#ifdef __cplusplus
}
#endif
