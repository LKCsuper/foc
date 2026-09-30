/*
 * @Description: 编码器机械角抗齿槽表
 * @Author: lkc
 *
 * 84 桶对应 12 槽 14 极电机每机械圈的齿槽周期。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <string.h>
#include "motor_anticogging.h"
/* Private define ------------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MOTOR_ANTICOGGING_MAX_VOLTAGE (0.5f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static float s_table[MOTOR_ANTICOGGING_BINS];
static uint16_t s_samples[MOTOR_ANTICOGGING_BINS];
static bool s_learning;
static bool s_calibrated;
static bool s_enabled;
static bool s_have_bin;
static uint16_t s_last_bin;
static uint16_t s_target_samples;
static uint32_t s_total_samples;
/* Private function prototypes -----------------------------------------------*/
static float anticog_wrap(float angle);
static uint16_t anticog_bin(float angle);
static void anticog_finish(void);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把机械角折回 [0, 2π)。
 * @param {float} angle  机械角 (rad)，须为有限值
 * @return {float} 折回后的机械角 (rad)
 * @author: lkc
 */
static float anticog_wrap(float angle)
{
    const float two_pi = 2.0f * (float)M_PI;

    while (angle < 0.0f) {
        angle += two_pi;
    }
    while (angle >= two_pi) {
        angle -= two_pi;
    }
    return angle;
}

/**
 * @description: 把机械角映射到齿槽表桶号。
 * @detail: 一圈均分为 MOTOR_ANTICOGGING_BINS 桶。落到末尾边界时归入 0 桶。
 * @param {float} angle  机械角 (rad)
 * @return {uint16_t} 桶号，范围 [0, MOTOR_ANTICOGGING_BINS)
 * @author: lkc
 */
static uint16_t anticog_bin(float angle)
{
    uint16_t bin = (uint16_t)(anticog_wrap(angle) * (float)MOTOR_ANTICOGGING_BINS / (2.0f * (float)M_PI));

    return bin < MOTOR_ANTICOGGING_BINS ? bin : 0U;
}

/**
 * @description: 把学完的齿槽表收成零均值补偿并打开使用。
 * @detail: 减去整表均值，再把每桶限制在 ±MOTOR_ANTICOGGING_MAX_VOLTAGE。随后结束学习、标记已标定并使能。
 * @return {void} 无
 * @author: lkc
 */
static void anticog_finish(void)
{
    float mean = 0.0f;
    uint16_t i;

    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++) {
        mean += s_table[i];
    }
    mean /= (float)MOTOR_ANTICOGGING_BINS;
    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++) {
        float value = s_table[i] - mean;

        if (value > MOTOR_ANTICOGGING_MAX_VOLTAGE) {
            value = MOTOR_ANTICOGGING_MAX_VOLTAGE;
        }
        if (value < -MOTOR_ANTICOGGING_MAX_VOLTAGE) {
            value = -MOTOR_ANTICOGGING_MAX_VOLTAGE;
        }
        s_table[i] = value;
    }
    s_learning = false;
    s_calibrated = true;
    s_enabled = true;
}

/**
 * @description: 清零齿槽表和学习状态。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Init(void)
{
    Motor_Anticogging_Clear();
}

/**
 * @description: 清空旧表并开始按桶学习齿槽电压。
 * @detail: 学习期间补偿关闭。每个桶要被转子重新进入达到 samples_per_bin 次才算完成。
 * @param {uint16_t} samples_per_bin  每桶目标样本数，允许 1..50
 * @return {bool} true=已开始；样本数超出范围返回 false，状态不变
 * @author: lkc
 */
bool Motor_Anticogging_Start(uint16_t samples_per_bin)
{
    if (samples_per_bin == 0U || samples_per_bin > 50U) {
        return false;
    }
    memset(s_table, 0, sizeof(s_table));
    memset(s_samples, 0, sizeof(s_samples));
    s_target_samples = samples_per_bin;
    s_total_samples = 0U;
    s_have_bin = false;
    s_calibrated = false;
    s_enabled = false;
    s_learning = true;
    return true;
}

/**
 * @description: 停止学习，不清除已有表，也不把未完成的表标成已标定。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Stop(void)
{
    s_learning = false;
    s_have_bin = false;
}

/**
 * @description: 清零齿槽表、样本计数和学习/标定/使能标志。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Clear(void)
{
    memset(s_table, 0, sizeof(s_table));
    memset(s_samples, 0, sizeof(s_samples));
    s_learning = false;
    s_calibrated = false;
    s_enabled = false;
    s_have_bin = false;
    s_target_samples = 0U;
    s_total_samples = 0U;
}

/**
 * @description: 打开或关闭齿槽电压补偿。
 * @detail: 未标定完成时不能打开，原使能状态保持不变。关闭不要求已标定。
 * @param {bool} enabled  true=使用补偿表
 * @return {bool} true=已按请求更新；未标定却要求打开时返回 false
 * @author: lkc
 */
bool Motor_Anticogging_SetEnabled(bool enabled)
{
    if (enabled && !s_calibrated) {
        return false;
    }
    s_enabled = enabled;
    return true;
}

/**
 * @description: 在转子进入新桶时记入一拍 q 轴 PI 电压。
 * @detail: 停在同一桶不重复计数，避免静止点把该桶拉偏。全部桶达到目标样本后调用 anticog_finish。
 *          非学习中，或角度/电压非有限值时直接返回。
 * @param {float} mech_angle_rad  机械角 (rad)
 * @param {float} pi_voltage      本拍速度环给出的 q 轴电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_LearnStep(float mech_angle_rad, float pi_voltage)
{
    uint16_t bin;
    uint16_t i;

    if (!s_learning || !isfinite(mech_angle_rad) || !isfinite(pi_voltage)) {
        return;
    }
    bin = anticog_bin(mech_angle_rad);
    if (s_have_bin && bin == s_last_bin) {
        return;
    }
    s_have_bin = true;
    s_last_bin = bin;
    if (s_samples[bin] < s_target_samples) {
        uint16_t n = s_samples[bin];

        s_table[bin] = (s_table[bin] * (float)n + pi_voltage) / (float)(n + 1U);
        s_samples[bin] = (uint16_t)(n + 1U);
        s_total_samples++;
    }
    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++) {
        if (s_samples[i] < s_target_samples) {
            return;
        }
    }
    anticog_finish();
}

/**
 * @description: 按机械角插值读取齿槽补偿电压。
 * @detail: 未使能、未标定或角度非有限值时返回 0。末桶与 0 桶衔接。
 * @param {float} mech_angle_rad  机械角 (rad)
 * @return {float} q 轴补偿电压 (V)
 * @author: lkc
 */
float Motor_Anticogging_GetVoltage(float mech_angle_rad)
{
    float position;
    float fraction;
    uint16_t bin0;
    uint16_t bin1;

    if (!s_enabled || !s_calibrated || !isfinite(mech_angle_rad)) {
        return 0.0f;
    }
    position = anticog_wrap(mech_angle_rad) * (float)MOTOR_ANTICOGGING_BINS / (2.0f * (float)M_PI);
    bin0 = (uint16_t)position;
    if (bin0 >= MOTOR_ANTICOGGING_BINS) {
        bin0 = 0U;
    }
    bin1 = (uint16_t)((bin0 + 1U) % MOTOR_ANTICOGGING_BINS);
    fraction = position - (float)bin0;
    return s_table[bin0] + (s_table[bin1] - s_table[bin0]) * fraction;
}

/**
 * @description: 复制学习进度和补偿表电压范围。
 * @param {motor_anticogging_status_t *} status  输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_GetStatus(motor_anticogging_status_t *status)
{
    uint16_t i;

    if (status == NULL) {
        return;
    }
    status->learning = s_learning;
    status->calibrated = s_calibrated;
    status->enabled = s_enabled;
    status->target_samples = s_target_samples;
    status->min_samples = s_samples[0];
    status->total_samples = s_total_samples;
    status->min_voltage = s_table[0];
    status->max_voltage = s_table[0];
    for (i = 1U; i < MOTOR_ANTICOGGING_BINS; i++) {
        if (s_samples[i] < status->min_samples) {
            status->min_samples = s_samples[i];
        }
        if (s_table[i] < status->min_voltage) {
            status->min_voltage = s_table[i];
        }
        if (s_table[i] > status->max_voltage) {
            status->max_voltage = s_table[i];
        }
    }
}

#ifdef __cplusplus
}
#endif
