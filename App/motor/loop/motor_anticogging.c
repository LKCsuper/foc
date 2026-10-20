/*
 * @Description: 位置保持电压抗齿槽表
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "common_assert.h"
#include <string.h>
#include "motor_anticogging.h"
/* Private define ------------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static float s_table[MOTOR_ANTICOGGING_BINS];
static float s_sum[MOTOR_ANTICOGGING_BINS];
static uint16_t s_samples[MOTOR_ANTICOGGING_BINS];
static bool s_learning;
static bool s_calibrated;
static bool s_enabled;
static float s_theta;
static float s_vq;
static uint16_t s_target_samples;
static uint32_t s_total_samples;
/* Private function prototypes -----------------------------------------------*/
static float wrap_2pi(float angle);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把角度折回 [0, 2π)。
 * @param {float} angle  角度 (rad)，须为有限值
 * @return {float} 折回后的角度 (rad)
 * @author: lkc
 */
static float wrap_2pi(float angle)
{
    const float two_pi = 2.0f * (float)M_PI;
    while (angle < 0.0f)
        angle += two_pi;
    while (angle >= two_pi)
        angle -= two_pi;
    return angle;
}

/**
 * @description: 清空齿槽表并退出学习。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Init(void)
{
    Motor_Anticogging_Clear();
}

/**
 * @description: 开始一轮齿槽位置扫描。
 * @detail: 清空旧表并关闭补偿。每桶采样次数须在 1～5。
 * @param {uint16_t} samples_per_bin  每桶目标采样次数
 * @return {bool} true=已进入学习；false=次数非法
 * @author: lkc
 */
bool Motor_Anticogging_Start(uint16_t samples_per_bin)
{
    if (samples_per_bin == 0U || samples_per_bin > 5U)
        return false;
    memset(s_table, 0, sizeof(s_table));
    memset(s_sum, 0, sizeof(s_sum));
    memset(s_samples, 0, sizeof(s_samples));
    s_target_samples = samples_per_bin;
    s_total_samples = 0U;
    s_theta = 0.0f;
    s_vq = 0.0f;
    s_calibrated = false;
    s_enabled = false;
    s_learning = true;
    return true;
}

/**
 * @description: 中止学习，不保留未完成的表。
 * @detail: 清学习标志和学习 Vq，不改已标定表的 calibrated/enabled。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Stop(void)
{
    s_learning = false;
    s_vq = 0.0f;
}

/**
 * @description: 清空补偿表、累加器和学习状态。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Clear(void)
{
    memset(s_table, 0, sizeof(s_table));
    memset(s_sum, 0, sizeof(s_sum));
    memset(s_samples, 0, sizeof(s_samples));
    s_learning = false;
    s_calibrated = false;
    s_enabled = false;
    s_theta = 0.0f;
    s_vq = 0.0f;
    s_target_samples = 0U;
    s_total_samples = 0U;
}

/**
 * @description: 开关运行中的齿槽补偿。
 * @detail: 未标定完成时不允许打开。
 * @param {bool} enabled  true=叠加补偿电压
 * @return {bool} true=已设置；false=尚未标定却要求打开
 * @author: lkc
 */
bool Motor_Anticogging_SetEnabled(bool enabled)
{
    if (enabled && !s_calibrated)
        return false;
    s_enabled = enabled;
    return true;
}

/**
 * @description: 查询是否正在学习。
 * @return {bool} true=学习中，控制输出改由学习目标决定
 * @author: lkc
 */
bool Motor_Anticogging_IsLearning(void)
{
    return s_learning;
}

/**
 * @description: 写入本拍学习用的电角和保持电压。
 * @detail: 非学习中或参数非有限值时忽略。电角折回 [0, 2π)。
 * @param {float} encoder_elec_angle_rad  编码器电角 (rad)
 * @param {float} vq_target               维持位置的 q 轴电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_SetLearnTarget(float encoder_elec_angle_rad, float vq_target)
{
    if (!s_learning || !isfinite(encoder_elec_angle_rad) || !isfinite(vq_target))
        return;
    s_theta = wrap_2pi(encoder_elec_angle_rad);
    s_vq = vq_target;
}

/**
 * @description: 读取学习拍要输出的电角和 dq 电压。
 * @detail: d 轴目标固定为 0。非学习中或指针为空时返回 false。
 * @param {float} *theta_rad  输出电角 (rad)
 * @param {float} *vd_target  输出 d 轴电压 (V)
 * @param {float} *vq_target  输出 q 轴电压 (V)
 * @return {bool} true=本拍由学习目标占用输出
 * @author: lkc
 */
bool Motor_Anticogging_GetLearnTargets(float *theta_rad, float *vd_target, float *vq_target)
{
    if (!s_learning || theta_rad == NULL || vd_target == NULL || vq_target == NULL)
        return false;
    *theta_rad = s_theta;
    *vd_target = 0.0f;
    *vq_target = s_vq;
    return true;
}

/**
 * @description: 把一个桶的保持电压记入累加器。
 * @detail: 该桶达到目标次数后拒绝再记。不在学习中、桶号越界或电压非有限值时返回 false。
 * @param {uint16_t} bin      桶号，范围 [0, MOTOR_ANTICOGGING_BINS)
 * @param {float} holding_vq  该位置的保持电压 (V)
 * @return {bool} true=已记入
 * @author: lkc
 */
bool Motor_Anticogging_Record(uint16_t bin, float holding_vq)
{
    if (!s_learning || bin >= MOTOR_ANTICOGGING_BINS || !isfinite(holding_vq) || s_samples[bin] >= s_target_samples)
        return false;
    s_sum[bin] += holding_vq;
    s_samples[bin]++;
    s_total_samples++;
    return true;
}

/**
 * @description: 把各桶平均电压收成零均值补偿表并打开补偿。
 * @detail: 任一桶未达到目标次数则失败，表保持学习前状态。单点限幅 ±0.5 V。
 * @return {bool} true=已标定并使能
 * @author: lkc
 */
bool Motor_Anticogging_Finish(void)
{
    float raw[MOTOR_ANTICOGGING_BINS];
    float mean = 0.0f;
    uint16_t i;
    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++) {
        if (s_samples[i] < s_target_samples)
            return false;
        raw[i] = s_sum[i] / (float)s_samples[i];
    }
    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++)
        mean += raw[i];
    mean /= (float)MOTOR_ANTICOGGING_BINS;
    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++) {
        float value = raw[i] - mean;
        if (value > MOTOR_ANTICOGGING_MAX_VOLTAGE)
            value = MOTOR_ANTICOGGING_MAX_VOLTAGE;
        if (value < -MOTOR_ANTICOGGING_MAX_VOLTAGE)
            value = -MOTOR_ANTICOGGING_MAX_VOLTAGE;
        s_table[i] = value;
    }
    s_learning = false;
    s_vq = 0.0f;
    s_calibrated = true;
    s_enabled = true;
    return true;
}

/**
 * @description: 按机械角插值读取齿槽补偿电压。
 * @detail: 未使能、未标定或角度非有限值时返回 0。桶中心对齐，末桶与 0 桶衔接。
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
    if (!s_enabled || !s_calibrated || !isfinite(mech_angle_rad))
        return 0.0f;
    position = wrap_2pi(mech_angle_rad) * (float)MOTOR_ANTICOGGING_BINS / (2.0f * (float)M_PI) - 0.5f;
    if (position < 0.0f)
        position += (float)MOTOR_ANTICOGGING_BINS;
    bin0 = (uint16_t)position;
    bin1 = (uint16_t)((bin0 + 1U) % MOTOR_ANTICOGGING_BINS);
    fraction = position - (float)bin0;
    return s_table[bin0] + (s_table[bin1] - s_table[bin0]) * fraction;
}

/**
 * @description: 复制学习进度和补偿表电压范围。
 * @param {motor_anticogging_status_t} *status  输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_GetStatus(motor_anticogging_status_t *status)
{
    uint16_t i;
    COMMON_ASSERT(status != NULL);
    status->learning = s_learning;
    status->calibrated = s_calibrated;
    status->enabled = s_enabled;
    status->target_samples = s_target_samples;
    status->min_samples = s_samples[0];
    status->total_samples = s_total_samples;
    status->min_voltage = s_table[0];
    status->max_voltage = s_table[0];
    for (i = 1U; i < MOTOR_ANTICOGGING_BINS; i++) {
        if (s_samples[i] < status->min_samples)
            status->min_samples = s_samples[i];
        if (s_table[i] < status->min_voltage)
            status->min_voltage = s_table[i];
        if (s_table[i] > status->max_voltage)
            status->max_voltage = s_table[i];
    }
}

/**
 * @description: 导出已标定的补偿表和使能标志。
 * @detail: 未标定或仍在学习时失败，不写出表。
 * @param {float} *table   输出，长度须等于 MOTOR_ANTICOGGING_BINS，单位 V
 * @param {uint16_t} bins  调用方桶数，须等于 MOTOR_ANTICOGGING_BINS
 * @param {bool} *enabled  输出当前使能
 * @return {bool} true=已复制
 * @author: lkc
 */
bool Motor_Anticogging_Export(float *table, uint16_t bins, bool *enabled)
{
    if (table == NULL || enabled == NULL || bins != MOTOR_ANTICOGGING_BINS || !s_calibrated || s_learning) return false;
    memcpy(table, s_table, sizeof(s_table));
    *enabled = s_enabled;
    return true;
}

/**
 * @description: 从已校验的表恢复补偿，不重新学习。
 * @detail: 任一点非有限或超出 ±0.5 V 则整表拒绝，原状态不变。
 * @param {const float} *table  补偿电压 (V)，长度 bins
 * @param {uint16_t} bins       须等于 MOTOR_ANTICOGGING_BINS
 * @param {bool} enabled        恢复后是否使能
 * @return {bool} true=已标定并按 enabled 开关
 * @author: lkc
 */
bool Motor_Anticogging_Import(const float *table, uint16_t bins, bool enabled)
{
    uint16_t i;
    if (table == NULL || bins != MOTOR_ANTICOGGING_BINS)
        return false;
    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++) {
        if (!isfinite(table[i]) || table[i] < -MOTOR_ANTICOGGING_MAX_VOLTAGE ||
            table[i] > MOTOR_ANTICOGGING_MAX_VOLTAGE) {
            return false;
        }
    }
    memcpy(s_table, table, sizeof(s_table));
    memset(s_sum, 0, sizeof(s_sum));
    memset(s_samples, 0, sizeof(s_samples));
    s_learning = false;
    s_calibrated = true;
    s_enabled = enabled;
    s_theta = 0.0f;
    s_vq = 0.0f;
    s_target_samples = 0U;
    s_total_samples = 0U;
    return true;
}
#ifdef __cplusplus
}
#endif
