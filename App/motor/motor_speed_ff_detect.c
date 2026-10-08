/* @Description: 空载速度电压前馈的线性拟合 */
#ifdef __cplusplus
extern "C" {
#endif
#include "motor_speed_ff_detect.h"
#include <math.h>
#include <stddef.h>
void Motor_SpeedFfFit_Init(motor_speed_ff_fit_t *fit)
{
    if (fit == NULL) return;
    fit->count = 0U;
    fit->sum_x = fit->sum_y = fit->sum_xx = fit->sum_xy = fit->sum_yy = 0.0f;
}
bool Motor_SpeedFfFit_Add(motor_speed_ff_fit_t *fit, float rpm, float vq)
{
    float x, y;
    if (fit == NULL || !isfinite(rpm) || !isfinite(vq) || fabsf(rpm) < 1.0f || rpm * vq <= 0.0f ||
        fit->count == UINT16_MAX) return false;
    x = fabsf(rpm);
    y = fabsf(vq);
    fit->count++;
    fit->sum_x += x;
    fit->sum_y += y;
    fit->sum_xx += x * x;
    fit->sum_xy += x * y;
    fit->sum_yy += y * y;
    return true;
}
bool Motor_SpeedFfFit_Solve(const motor_speed_ff_fit_t *fit, motor_speed_ff_result_t *result)
{
    float n, denominator, slope, boost, sse;
    if (fit == NULL || result == NULL || fit->count < 4U) return false;
    n = (float)fit->count;
    denominator = n * fit->sum_xx - fit->sum_x * fit->sum_x;
    if (!isfinite(denominator) || denominator < 1.0f) return false;
    slope = (n * fit->sum_xy - fit->sum_x * fit->sum_y) / denominator;
    boost = (fit->sum_y - slope * fit->sum_x) / n;
    sse = fit->sum_yy + slope * slope * fit->sum_xx + n * boost * boost -
          2.0f * slope * fit->sum_xy - 2.0f * boost * fit->sum_y + 2.0f * slope * boost * fit->sum_x;
    /* 负截距表示测试范围已出现饱和或明显非线性，不能把它伪装成有效的摩擦 boost。 */
    if (!isfinite(slope) || !isfinite(boost) || !isfinite(sse) || slope <= 0.0f || slope > 0.05f ||
        boost < 0.0f || boost > 2.0f) return false;
    if (sse < 0.0f) sse = 0.0f;
    result->slope_v_per_rpm = slope;
    result->boost_v = boost;
    result->rms_error_v = sqrtf(sse / n);
    result->points = fit->count;
    return true;
}
#ifdef __cplusplus
}
#endif
