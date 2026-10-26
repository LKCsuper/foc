/* @Description: 空载速度电压前馈的线性拟合 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_speed_ff_detect.h"
#include "common_assert.h"
#include <math.h>
#include <stddef.h>
#include "motor_config.h"
#if MOTOR_CFG_SPEED
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 清零线性拟合累加器。
 * @param {motor_speed_ff_fit_t *} fit  累加器，不可为空
 * @return {void} 无
 */
void Motor_SpeedFfFit_Init(motor_speed_ff_fit_t *fit)
{
    COMMON_ASSERT(fit != NULL);
    fit->count = 0U;
    fit->sum_x = fit->sum_y = fit->sum_xx = fit->sum_xy = fit->sum_yy = 0.0f;
}

/**
 * @description: 累加一个 (|rpm|, |vq|) 样本。
 * @detail: |rpm| 小于 1 或 rpm 与 vq 异号时拒绝，计数到上限后拒绝。
 * @param {motor_speed_ff_fit_t *} fit  累加器，不可为空
 * @param {float} rpm  转速 (RPM)
 * @param {float} vq   q 轴电压 (V)
 * @return {bool} 已累加返回 true
 */
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
/**
 * @description: 最小二乘求解斜率、截距与拟合误差。
 * @detail: 样本不足 4 个、拟合非法或斜率/截距超出合理范围时失败。
 * @param {const motor_speed_ff_fit_t *} fit  累加器，不可为空
 * @param {motor_speed_ff_result_t *} result  输出结果，不可为空
 * @return {bool} 求解成功返回 true
 */
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
#endif /* MOTOR_CFG_SPEED */
#ifdef __cplusplus
}
#endif
