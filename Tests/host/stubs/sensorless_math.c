/*
 * Minimal math stubs for sensorless host tests (avoid full motor_math.c / IN macro).
 */
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "motor_foc.h"
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：同一角度共用正弦表索引与插值比例，计算 sin 与 cos。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} angle angle 参数
 * @param {float *} sine sine 参数
 * @param {float *} cosine cosine 参数
 * @return {void} 无
 */
void sincos_f32(float angle, float *sine, float *cosine) { *sine = sinf(angle); *cosine = cosf(angle); }

#ifndef ONE_BY_SQRT3
#define ONE_BY_SQRT3 (0.57735026919f)
#endif

/**
 * @description: 主机测试替身：Clarke 变换，将三相电流投影到 αβ 静止坐标系。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} Ia Ia 参数
 * @param {float} Ib Ib 参数
 * @param {float} Ic Ic 参数
 * @param {float *} Ialpha Ialpha 参数
 * @param {float *} Ibeta Ibeta 参数
 * @return {void} 无
 */
void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
    *Ialpha = (2.0f / 3.0f) * Ia - (1.0f / 3.0f) * Ib - (1.0f / 3.0f) * Ic;
    *Ibeta = (Ib - Ic) * ONE_BY_SQRT3;
}

#ifndef SENSORLESS_REAL_MATH
/**
 * @description: 主机测试替身：计算两角度之差并限制在 [-π, π]。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} angle1 angle1 参数
 * @param {float} angle2 angle2 参数
 * @return {float} 测试记录值 difference
 */
float utils_angle_difference_rad(float angle1, float angle2)
{
    float difference = angle1 - angle2;

    while (difference > 3.14159265f) {
        difference -= 2.0f * 3.14159265f;
    }
    while (difference < -3.14159265f) {
        difference += 2.0f * 3.14159265f;
    }
    return difference;
}

/**
 * @description: 主机测试替身：快速近似 atan2，返回 [-π, π] 弧度角。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} y y 参数
 * @param {float} x x 参数
 * @return {float} 极角 (rad)
 */
float utils_fast_atan2(float y, float x)
{
    return atan2f(y, x);
}
#endif
