/*
 * @Description: 脉振高频注入（HFI）低速凸极跟踪
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_hfi.h"
#include "common_assert.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_HFI_PI (3.14159265358979323846f)
#define MOTOR_HFI_TWO_PI (2.0f * MOTOR_HFI_PI)
#define MOTOR_HFI_SPEED_LPF (0.10f) /* 电角速度一阶低通系数，0~1 */
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static float Motor_Hfi_WrapTwoPi(float angle);
static float Motor_Hfi_WrapPi(float angle);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将电角循环归一化到 [0, 2π)
 * @param {float} angle  待归一化角度 (rad)
 * @return {float} [0, 2π)
 * @author: lkc
 */
static float Motor_Hfi_WrapTwoPi(float angle)
{
    while (angle >= MOTOR_HFI_TWO_PI) {
        angle -= MOTOR_HFI_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += MOTOR_HFI_TWO_PI;
    }
    return angle;
}

/**
 * @description: 将角度差循环归一化到 [−π, π)
 * @param {float} angle  待归一化角度 (rad)
 * @return {float} [−π, π)
 * @author: lkc
 */
static float Motor_Hfi_WrapPi(float angle)
{
    while (angle >= MOTOR_HFI_PI) {
        angle -= MOTOR_HFI_TWO_PI;
    }
    while (angle < -MOTOR_HFI_PI) {
        angle += MOTOR_HFI_TWO_PI;
    }
    return angle;
}

/**
 * @description: 复位高频注入状态（角度与速度清零，停止注入）
 * @param {motor_hfi_t} *hfi  实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Hfi_Reset(motor_hfi_t *hfi)
{
    if (hfi != NULL) {
        memset(hfi, 0, sizeof(*hfi));
    }
}

/**
 * @description: 规整高频注入配置
 * @param {motor_hfi_config_t} *config  配置，被原地修改；可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Hfi_NormalizeConfig(motor_hfi_config_t *config)
{
    COMMON_ASSERT(config != NULL);
    if (config->voltage < 0.0f) {
        config->voltage = 0.0f;
    }
    if (config->voltage > MOTOR_HFI_MAX_VOLTAGE) {
        config->voltage = MOTOR_HFI_MAX_VOLTAGE;
    }
    if (config->gain < 0.0f) {
        config->gain = 0.0f;
    }
    if (config->gain > MOTOR_HFI_MAX_GAIN) {
        config->gain = MOTOR_HFI_MAX_GAIN;
    }
    if (config->max_bus_voltage_ratio <= 0.0f || config->max_bus_voltage_ratio > 1.0f) {
        config->max_bus_voltage_ratio = MOTOR_HFI_DEFAULT_BUS_RATIO;
    }
}

/**
 * @description: 写入跟踪初值电角
 * @param {motor_hfi_t} *hfi  实例，可为 NULL
 * @param {float} angle       初始电角 (rad)
 * @return {*}
 * @author: lkc
 */
void Motor_Hfi_SeedAngle(motor_hfi_t *hfi, float angle)
{
    COMMON_ASSERT(hfi != NULL);
    hfi->angle = Motor_Hfi_WrapTwoPi(angle);
}

/**
 * @description: 推进一拍脉振高频注入
 * @detail: 上一拍 Vd_h=p·Vh 后，估计坐标系 ΔIq ≈ p·C·sin(2θ_err)，C=(Ld−Lq)Ts/(2Ld Lq)<0（IPMSM）。
 *          解调 error=−ΔIq·p，正 gain 增大滞后的估计角。首拍只注入。
 * @param {motor_hfi_t} *hfi                  实例
 * @param {const motor_hfi_config_t} *config  配置
 * @param {float} i_alpha                     α 轴电流 (A)
 * @param {float} i_beta                      β 轴电流 (A)
 * @param {float} v_bus                       母线 (V)
 * @param {float} dt                          电流环周期 (s)
 * @param {motor_hfi_output_t} *output        本拍输出
 * @return {bool} true=应叠加 vd_inj
 * @author: lkc
 */
bool Motor_Hfi_Step(motor_hfi_t *hfi, const motor_hfi_config_t *config, float i_alpha, float i_beta, float v_bus,
                    float dt, motor_hfi_output_t *output)
{
    float c;
    float s;
    float iq;
    float voltage;
    float prev_angle;

    if (output != NULL) {
        memset(output, 0, sizeof(*output));
    }
    if (hfi == NULL || config == NULL || output == NULL || !config->enabled || config->voltage <= 0.0f) {
        if (hfi != NULL) {
            hfi->running = false;
            hfi->sample_valid = false;
            hfi->last_vd_inj = 0.0f;
        }
        return false;
    }

    if (!hfi->running) {
        hfi->running = true;
        hfi->sample_valid = false;
        hfi->polarity = 1;
        hfi->prev_iq = 0.0f;
        hfi->last_error = 0.0f;
        hfi->speed_e = 0.0f;
    }

    c = cosf(hfi->angle);
    s = sinf(hfi->angle);
    iq = -i_alpha * s + i_beta * c;

    if (hfi->sample_valid) {
        /* 当前 Iq 由上一拍 polarity 的注入产生；IPMSM 解调取负后正 gain 跟踪真角。 */
        hfi->last_error = -(iq - hfi->prev_iq) * (float)hfi->polarity;
        prev_angle = hfi->angle;
        hfi->angle = Motor_Hfi_WrapTwoPi(hfi->angle + config->gain * hfi->last_error);
        if (dt > 0.0f && isfinite(dt)) {
            hfi->speed_e += MOTOR_HFI_SPEED_LPF * (Motor_Hfi_WrapPi(hfi->angle - prev_angle) / dt - hfi->speed_e);
        }
        hfi->polarity = (int8_t)(-hfi->polarity);
    } else {
        hfi->polarity = 1;
    }

    hfi->prev_iq = iq;
    hfi->sample_valid = true;

    voltage = config->voltage;
    if (v_bus > 0.0f && voltage > v_bus * config->max_bus_voltage_ratio) {
        voltage = v_bus * config->max_bus_voltage_ratio;
    }

    output->apply = true;
    output->vd_inj = (float)hfi->polarity * voltage;
    output->vq_inj = 0.0f;
    output->angle = hfi->angle;
    hfi->last_vd_inj = output->vd_inj;
    return true;
}

/**
 * @description: 读取跟踪电角
 * @param {const motor_hfi_t} *hfi  实例；NULL 返回 0
 * @return {float} 电角 (rad)
 * @author: lkc
 */
float Motor_Hfi_GetAngle(const motor_hfi_t *hfi)
{
    return hfi == NULL ? 0.0f : hfi->angle;
}

/**
 * @description: 读取估计电角速度
 * @param {const motor_hfi_t} *hfi  实例；NULL 返回 0
 * @return {float} 电角速度 (rad/s)
 * @author: lkc
 */
float Motor_Hfi_GetSpeedE(const motor_hfi_t *hfi)
{
    return hfi == NULL ? 0.0f : hfi->speed_e;
}

#ifdef __cplusplus
}
#endif
