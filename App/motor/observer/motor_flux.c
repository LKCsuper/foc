/*
 * @Description: Ortega 非线性磁链观测器（αβ 静止坐标系）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_flux.h"
#include "common_assert.h"
#include "motor_foc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_FLUX_PI (3.14159265358979323846f)
#define MOTOR_FLUX_TWO_PI (2.0f * MOTOR_FLUX_PI)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static float Motor_Flux_WrapPi(float angle);
static void Motor_Flux_ClearOutput(motor_flux_output_t *output);
static void Motor_Flux_Publish(motor_flux_t *obs, const motor_flux_config_t *config, float i_alpha, float i_beta,
                               motor_flux_output_t *output);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将角度差循环归一化到 [−π, π)
 * @param {float} angle  待归一化角度 (rad)
 * @return {float} [−π, π)
 * @author: lkc
 */
static float Motor_Flux_WrapPi(float angle)
{
    while (angle >= MOTOR_FLUX_PI) {
        angle -= MOTOR_FLUX_TWO_PI;
    }
    while (angle < -MOTOR_FLUX_PI) {
        angle += MOTOR_FLUX_TWO_PI;
    }
    return angle;
}

/**
 * @description: 清空本拍输出
 * @param {motor_flux_output_t} *output  输出；可为 NULL
 * @return {*}
 * @author: lkc
 */
static void Motor_Flux_ClearOutput(motor_flux_output_t *output)
{
    if (output != NULL) {
        memset(output, 0, sizeof(*output));
    }
}

/**
 * @description: 由当前定子磁链与电流计算转子磁链、模长和电角，并写入状态与输出
 * @param {motor_flux_t} *obs                  实例
 * @param {const motor_flux_config_t} *config  配置
 * @param {float} i_alpha                      α 轴电流 (A)
 * @param {float} i_beta                       β 轴电流 (A)
 * @param {motor_flux_output_t} *output        本拍输出
 * @return {*}
 * @author: lkc
 */
static void Motor_Flux_Publish(motor_flux_t *obs, const motor_flux_config_t *config, float i_alpha, float i_beta,
                               motor_flux_output_t *output)
{
    float flux = config->flux_linkage;

    obs->psi_alpha = obs->x_alpha - config->inductance * i_alpha;
    obs->psi_beta = obs->x_beta - config->inductance * i_beta;
    obs->magnitude = sqrtf(obs->psi_alpha * obs->psi_alpha + obs->psi_beta * obs->psi_beta);
    obs->angle = Motor_Flux_WrapPi(atan2f(obs->psi_beta, obs->psi_alpha) + config->phase_offset);

    output->valid = true;
    output->flux_ok = obs->magnitude > flux * MOTOR_FLUX_MAG_MIN_RATIO &&
                      obs->magnitude < flux * MOTOR_FLUX_MAG_MAX_RATIO;
    output->psi_alpha = obs->psi_alpha;
    output->psi_beta = obs->psi_beta;
    output->magnitude = obs->magnitude;
    output->angle = obs->angle;
}

/**
 * @description: 复位磁链观测状态
 * @param {motor_flux_t} *obs  实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Flux_Reset(motor_flux_t *obs)
{
    if (obs != NULL) {
        memset(obs, 0, sizeof(*obs));
    }
}

/**
 * @description: 规整磁链观测配置
 * @param {motor_flux_config_t} *config  配置，被原地修改；可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Flux_NormalizeConfig(motor_flux_config_t *config)
{
    COMMON_ASSERT(config != NULL);
    if (!(config->resistance >= 0.0f) || !isfinite(config->resistance)) {
        config->resistance = 0.0f;
    }
    if (!(config->inductance >= 0.0f) || !isfinite(config->inductance)) {
        config->inductance = 0.0f;
    }
    if (!isfinite(config->flux_linkage) || config->flux_linkage < MOTOR_FLUX_MIN_LINKAGE) {
        config->flux_linkage = MOTOR_FLUX_MIN_LINKAGE;
    }
    if (!isfinite(config->observer_gain) || config->observer_gain < 0.0f) {
        config->observer_gain = 0.0f;
    }
    if (!isfinite(config->phase_offset)) {
        config->phase_offset = 0.0f;
    }
}

/**
 * @description: 用当前电流和假设电角初始化定子磁链积分器
 * @param {motor_flux_t} *obs                  实例
 * @param {const motor_flux_config_t} *config  配置
 * @param {float} angle                        假设电角 (rad)
 * @param {float} i_alpha                      α 轴电流 (A)
 * @param {float} i_beta                       β 轴电流 (A)
 * @return {*}
 * @author: lkc
 */
void Motor_Flux_Seed(motor_flux_t *obs, const motor_flux_config_t *config, float angle, float i_alpha, float i_beta)
{
    float flux;
    float inductance;
    float wrapped;
    float sine, cosine;

    COMMON_ASSERT(obs != NULL && config != NULL);
    if (!isfinite(angle) || !isfinite(i_alpha) || !isfinite(i_beta)) return;

    flux = config->flux_linkage >= MOTOR_FLUX_MIN_LINKAGE ? config->flux_linkage : MOTOR_FLUX_MIN_LINKAGE;
    inductance = config->inductance >= 0.0f ? config->inductance : 0.0f;
    wrapped = Motor_Flux_WrapPi(angle);

    /* L×I 是电感磁链；ψf×cos/sin 是假定方向上的永磁磁链。 */
    sincos_f32(wrapped, &sine, &cosine);
    obs->psi_alpha = flux * cosine;
    obs->psi_beta = flux * sine;
    obs->x_alpha = inductance * i_alpha + obs->psi_alpha;
    obs->x_beta = inductance * i_beta + obs->psi_beta;
    obs->magnitude = flux;
    obs->angle = Motor_Flux_WrapPi(wrapped + config->phase_offset);
    obs->initialized = true;
}

/**
 * @description: 推进一拍 Ortega 磁链观测
 * @param {motor_flux_t} *obs                  实例
 * @param {const motor_flux_config_t} *config  配置
 * @param {float} v_alpha                      α 轴电压 (V)
 * @param {float} v_beta                       β 轴电压 (V)
 * @param {float} i_alpha                      α 轴电流 (A)
 * @param {float} i_beta                       β 轴电流 (A)
 * @param {float} dt                           控制周期 (s)
 * @param {motor_flux_output_t} *output        本拍输出
 * @return {bool} true=本拍观测有效
 * @author: lkc
 */
bool Motor_Flux_Step(motor_flux_t *obs, const motor_flux_config_t *config, float v_alpha, float v_beta, float i_alpha,
                     float i_beta, float dt, motor_flux_output_t *output)
{
    float resistance;
    float inductance;
    float flux;
    float observer_gain;
    float rotor_flux_alpha;
    float rotor_flux_beta;
    float magnitude_error;
    float x_alpha_dot;
    float x_beta_dot;

    Motor_Flux_ClearOutput(output);
    if (obs == NULL || config == NULL || output == NULL) return false;
    if (!config->enabled || !(dt > 0.0f) || !isfinite(dt) || !isfinite(v_alpha) || !isfinite(v_beta) ||
        !isfinite(i_alpha) || !isfinite(i_beta)) {
        Motor_Flux_Reset(obs);
        return false;
    }

    resistance = config->resistance >= 0.0f ? config->resistance : 0.0f;
    inductance = config->inductance >= 0.0f ? config->inductance : 0.0f;
    flux = config->flux_linkage >= MOTOR_FLUX_MIN_LINKAGE ? config->flux_linkage : MOTOR_FLUX_MIN_LINKAGE;
    observer_gain = config->observer_gain;
    if (!(observer_gain > 0.0f) || !isfinite(observer_gain)) {
        observer_gain = MOTOR_FLUX_GAIN_FALLBACK_NUM / (flux * flux);
    }

    if (!obs->initialized) {
        Motor_Flux_Seed(obs, config, 0.0f, i_alpha, i_beta);
    }

    /*
     * 先用积分前的定子磁链拆出转子磁链：
     *   ψr = ψs − L×I
     * Ortega 幅值误差：
     *   e = ψf² − |ψr|²
     * 只保留 e≤0：磁链偏大时拉回额定圆；偏小时不推出去，避免低速噪声被放大。
     */
    rotor_flux_alpha = obs->x_alpha - inductance * i_alpha;
    rotor_flux_beta = obs->x_beta - inductance * i_beta;
    magnitude_error = flux * flux - (rotor_flux_alpha * rotor_flux_alpha + rotor_flux_beta * rotor_flux_beta);
    if (magnitude_error > 0.0f) {
        magnitude_error = 0.0f;
    }

    /* dψs/dt = v − R×I + (γ/2)·ψr·e */
    x_alpha_dot = v_alpha - resistance * i_alpha + 0.5f * observer_gain * rotor_flux_alpha * magnitude_error;
    x_beta_dot = v_beta - resistance * i_beta + 0.5f * observer_gain * rotor_flux_beta * magnitude_error;
    obs->x_alpha += x_alpha_dot * dt;
    obs->x_beta += x_beta_dot * dt;

    if (!isfinite(obs->x_alpha) || !isfinite(obs->x_beta)) {
        Motor_Flux_Reset(obs);
        return false;
    }

    Motor_Flux_Publish(obs, config, i_alpha, i_beta, output);
    if (!isfinite(obs->magnitude) || !isfinite(obs->angle)) {
        Motor_Flux_Reset(obs);
        Motor_Flux_ClearOutput(output);
        return false;
    }
    return true;
}

/**
 * @description: 读取原始观测电角
 * @param {const motor_flux_t} *obs  实例；NULL 返回 0
 * @return {float} 电角 (rad)
 * @author: lkc
 */
float Motor_Flux_GetAngle(const motor_flux_t *obs)
{
    return obs == NULL ? 0.0f : obs->angle;
}

/**
 * @description: 读取估算转子磁链模长
 * @param {const motor_flux_t} *obs  实例；NULL 返回 0
 * @return {float} |ψr| (Wb)
 * @author: lkc
 */
float Motor_Flux_GetMagnitude(const motor_flux_t *obs)
{
    return obs == NULL ? 0.0f : obs->magnitude;
}

#ifdef __cplusplus
}
#endif
