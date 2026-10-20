#ifndef MOTOR_DETECT_RL_FIT_H
#define MOTOR_DETECT_RL_FIT_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* 对恒压脉冲窗口拟合 Δψ=L*Δi+b。梯形积分扣除 Ri，全部样本参与拟合。
 * 电压必须对应窗口内实际生效值；本函数不估计死区、延时或器件压降。
 * 返回窗口等效电感，不代表跨工作点的增量电感曲面。 */
static inline bool Motor_Detect_FitPulse(const float *current, uint32_t count, float dt,
                                       float voltage, float resistance, float *inductance)
{
    float flux = 0.0f, mean_i = 0.0f, mean_flux = 0.0f;
    float covariance = 0.0f, variance = 0.0f;
    uint32_t k;

    if (current == 0 || inductance == 0 || count < 3U || !isfinite(dt) || dt <= 0.0f ||
        !isfinite(voltage) || !isfinite(resistance) || resistance < 0.0f) {
        return false;
    }
    for (k = 0U; k < count; k++) {
        float di, df, sample;
        if (!isfinite(current[k])) return false;
        if (k > 0U) {
            flux += (voltage - resistance * (0.5f * current[k - 1U] + 0.5f * current[k])) * dt;
        }
        sample = current[k] - current[0];
        di = sample - mean_i;
        df = flux - mean_flux;
        mean_i += di / (float)(k + 1U);
        mean_flux += df / (float)(k + 1U);
        variance += di * (sample - mean_i);
        covariance += di * (flux - mean_flux);
    }
    if (!isfinite(variance) || variance <= 1.0e-12f || !isfinite(covariance)) return false;
    {
        float value = covariance / variance;
        if (!isfinite(value) || value <= 0.0f) return false;
        *inductance = value;
    }
    return true;
}

#endif
