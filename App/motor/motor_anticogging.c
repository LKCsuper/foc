/* @Description: 位置保持电压抗齿槽表 */
#include <math.h>
#include <string.h>
#include "motor_anticogging.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define MOTOR_ANTICOGGING_MAX_VOLTAGE (0.50f)

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

static float wrap_2pi(float angle)
{
    const float two_pi = 2.0f * (float)M_PI;
    while (angle < 0.0f) angle += two_pi;
    while (angle >= two_pi) angle -= two_pi;
    return angle;
}

void Motor_Anticogging_Init(void) { Motor_Anticogging_Clear(); }

bool Motor_Anticogging_Start(uint16_t samples_per_bin)
{
    if (samples_per_bin == 0U || samples_per_bin > 5U) return false;
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

void Motor_Anticogging_Stop(void)
{
    s_learning = false;
    s_vq = 0.0f;
}

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

bool Motor_Anticogging_SetEnabled(bool enabled)
{
    if (enabled && !s_calibrated) return false;
    s_enabled = enabled;
    return true;
}

bool Motor_Anticogging_IsLearning(void) { return s_learning; }

void Motor_Anticogging_SetLearnTarget(float encoder_elec_angle_rad, float vq_target)
{
    if (!s_learning || !isfinite(encoder_elec_angle_rad) || !isfinite(vq_target)) return;
    s_theta = wrap_2pi(encoder_elec_angle_rad);
    s_vq = vq_target;
}

bool Motor_Anticogging_GetLearnTargets(float *theta_rad, float *vd_target, float *vq_target)
{
    if (!s_learning || theta_rad == NULL || vd_target == NULL || vq_target == NULL) return false;
    *theta_rad = s_theta;
    *vd_target = 0.0f;
    *vq_target = s_vq;
    return true;
}

bool Motor_Anticogging_Record(uint16_t bin, float holding_vq)
{
    if (!s_learning || bin >= MOTOR_ANTICOGGING_BINS || !isfinite(holding_vq) ||
        s_samples[bin] >= s_target_samples) return false;
    s_sum[bin] += holding_vq;
    s_samples[bin]++;
    s_total_samples++;
    return true;
}

bool Motor_Anticogging_Finish(void)
{
    float raw[MOTOR_ANTICOGGING_BINS];
    float mean = 0.0f;
    uint16_t i;
    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++) {
        if (s_samples[i] < s_target_samples) return false;
        raw[i] = s_sum[i] / (float)s_samples[i];
    }
    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++) mean += raw[i];
    mean /= (float)MOTOR_ANTICOGGING_BINS;
    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++) {
        float value = raw[i] - mean;
        if (value > MOTOR_ANTICOGGING_MAX_VOLTAGE) value = MOTOR_ANTICOGGING_MAX_VOLTAGE;
        if (value < -MOTOR_ANTICOGGING_MAX_VOLTAGE) value = -MOTOR_ANTICOGGING_MAX_VOLTAGE;
        s_table[i] = value;
    }
    s_learning = false;
    s_vq = 0.0f;
    s_calibrated = true;
    s_enabled = true;
    return true;
}

float Motor_Anticogging_GetVoltage(float mech_angle_rad)
{
    float position;
    float fraction;
    uint16_t bin0;
    uint16_t bin1;
    if (!s_enabled || !s_calibrated || !isfinite(mech_angle_rad)) return 0.0f;
    position = wrap_2pi(mech_angle_rad) * (float)MOTOR_ANTICOGGING_BINS / (2.0f * (float)M_PI) - 0.5f;
    if (position < 0.0f) position += (float)MOTOR_ANTICOGGING_BINS;
    bin0 = (uint16_t)position;
    bin1 = (uint16_t)((bin0 + 1U) % MOTOR_ANTICOGGING_BINS);
    fraction = position - (float)bin0;
    return s_table[bin0] + (s_table[bin1] - s_table[bin0]) * fraction;
}

void Motor_Anticogging_GetStatus(motor_anticogging_status_t *status)
{
    uint16_t i;
    if (status == NULL) return;
    status->learning = s_learning;
    status->calibrated = s_calibrated;
    status->enabled = s_enabled;
    status->target_samples = s_target_samples;
    status->min_samples = s_samples[0];
    status->total_samples = s_total_samples;
    status->min_voltage = s_table[0];
    status->max_voltage = s_table[0];
    for (i = 1U; i < MOTOR_ANTICOGGING_BINS; i++) {
        if (s_samples[i] < status->min_samples) status->min_samples = s_samples[i];
        if (s_table[i] < status->min_voltage) status->min_voltage = s_table[i];
        if (s_table[i] > status->max_voltage) status->max_voltage = s_table[i];
    }
}

bool Motor_Anticogging_Export(float *table, uint16_t bins, bool *enabled)
{
    if (table == NULL || enabled == NULL || bins != MOTOR_ANTICOGGING_BINS || !s_calibrated || s_learning) {
        return false;
    }
    memcpy(table, s_table, sizeof(s_table));
    *enabled = s_enabled;
    return true;
}

bool Motor_Anticogging_Import(const float *table, uint16_t bins, bool enabled)
{
    uint16_t i;
    if (table == NULL || bins != MOTOR_ANTICOGGING_BINS) return false;
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
