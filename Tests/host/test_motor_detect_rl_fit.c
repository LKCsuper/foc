#include <assert.h>
#include <stdio.h>
#include "motor_detect_rl_fit.h"

static unsigned random_state = 42U;
static float noise(void)
{
    random_state = 1664525U * random_state + 1013904223U;
    return ((float)(random_state >> 8) / 16777216.0f - 0.5f) * 0.002f;
}

int main(void)
{
    float samples[8], result = -1.0f;
    const float dt = 1.0f / 30000.0f, r = 0.5f, l = 0.001f;
    unsigned k;
    int sign;
    /* 梯形离散 RL：覆盖正负电压、非零初始电流和电阻压降。 */
    for (sign = -1; sign <= 1; sign += 2) {
        float voltage = (float)sign * 0.5f;
        samples[0] = (float)sign * 0.1f;
        for (k = 1; k < 8; k++) {
            samples[k] = ((l - 0.5f * r * dt) * samples[k - 1] + voltage * dt) /
                         (l + 0.5f * r * dt);
        }
        assert(Motor_Detect_FitPulse(samples, 8, dt, voltage, r, &result));
        assert(fabsf(result / l - 1.0f) < 1.0e-4f);
    }
    /* 连续 RL 精确响应独立核对，容许梯形积分的离散误差。 */
    for (k = 0; k < 8; k++) {
        samples[k] = 1.0f - expf(-r * (float)k * dt / l);
    }
    assert(Motor_Detect_FitPulse(samples, 8, dt, 0.5f, r, &result));
    assert(fabsf(result / l - 1.0f) < 0.001f);
    {
        float fit_error = 0.0f, endpoint_error = 0.0f;
        unsigned trial;
        for (trial = 0; trial < 2000; trial++) {
            float endpoint;
            for (k = 0; k < 8; k++) {
                samples[k] = 1.0f - expf(-r * (float)k * dt / l) + noise();
            }
            assert(Motor_Detect_FitPulse(samples, 8, dt, 0.5f, r, &result));
            endpoint = (0.5f - r * 0.5f * (samples[0] + samples[7])) * (7.0f * dt) /
                       (samples[7] - samples[0]);
            fit_error += (result - l) * (result - l);
            endpoint_error += (endpoint - l) * (endpoint - l);
        }
        printf("noise RMSE: window=%g endpoint=%g H\n", sqrtf(fit_error / 2000),
               sqrtf(endpoint_error / 2000));
        assert(fit_error < endpoint_error);
    }
    /* 无激励、非有限值、无效时间与负电感必须拒绝且不发布结果。 */
    for (k = 0; k < 8; k++) samples[k] = 0.1f;
    result = -1.0f;
    assert(!Motor_Detect_FitPulse(samples, 8, dt, 0.5f, r, &result));
    assert(result == -1.0f);
    samples[3] = NAN;
    assert(!Motor_Detect_FitPulse(samples, 8, dt, 0.5f, r, &result));
    assert(!Motor_Detect_FitPulse(samples, 8, 0.0f, 0.5f, r, &result));
    assert(!Motor_Detect_FitPulse(samples, 8, INFINITY, 0.5f, r, &result));
    for (k = 0; k < 8; k++) samples[k] = (float)k * 0.01f;
    assert(!Motor_Detect_FitPulse(samples, 8, dt, -0.5f, 0.0f, &result));
    puts("motor detect RL fit passed");
    return 0;
}
