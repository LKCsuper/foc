/* 角度差：单圈内与旧实现逐值一致、边界与无穷/NaN 处理。 */
#include <assert.h>
#include <float.h>
#include <stdio.h>
#include "motor_math.h"

/**
 * @description: 保留原角差算法作为回归比较基准。
 * @param {float} a a 参数
 * @param {float} b b 参数
 * @return {float} 测试记录值 d
 */
static float original_difference(float a, float b)
{
    float d = a - b;
    while (d < -M_PI) d += 2.0f * M_PI;
    while (d > M_PI) d -= 2.0f * M_PI;
    return d;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    /* 单圈控制输入与旧实现逐值一致，包括反转及跨零点。 */
    for (int i = -256; i <= 256; ++i) {
        for (int j = -256; j <= 256; ++j) {
            float a = (float)i * (2.0f * M_PI / 256.0f);
            float b = (float)j * (2.0f * M_PI / 256.0f);
            assert(utils_float_same_bits(utils_angle_difference_rad_inline(a, b),
                                         utils_angle_difference_rad(a, b)));
            assert(fabsf(utils_angle_difference_rad(a, b) - original_difference(a, b)) < 2e-6f);
        }
    }
    assert(utils_angle_difference_rad((float)M_PI, 0.0f) == (float)M_PI);
    assert(utils_angle_difference_rad(-(float)M_PI, 0.0f) == -(float)M_PI);
    assert(isnan(utils_angle_difference_rad(INFINITY, 0.0f)));
    assert(isnan(utils_angle_difference_rad(0.0f, -INFINITY)));
    assert(isnan(utils_angle_difference_rad(NAN, 0.0f)));
    assert(isnan(utils_angle_difference_rad_inline(INFINITY, 0.0f)));
    assert(isnan(utils_angle_difference_rad_inline(0.0f, -INFINITY)));
    assert(isnan(utils_angle_difference_rad_inline(NAN, 0.0f)));
    assert(utils_float_same_bits(utils_angle_difference_rad_inline(-0.0f, 0.0f), -0.0f));
    assert(utils_angle_difference_rad_inline(M_PI, 0.0f) == M_PI);
    assert(utils_angle_difference_rad_inline(-M_PI, 0.0f) == -M_PI);
    const float values[] = {FLT_MAX, -FLT_MAX, 1e10f, -1e10f, 0.0f, -0.0f,
        M_PI, -M_PI, nextafterf(M_PI, 0.0f), nextafterf(M_PI, INFINITY),
        nextafterf(-M_PI, 0.0f), nextafterf(-M_PI, -INFINITY)};
    for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        for (unsigned j = 0; j < sizeof(values) / sizeof(values[0]); ++j) {
            float d = utils_angle_difference_rad(values[i], values[j]);
            assert(utils_float_same_bits(d, utils_angle_difference_rad_inline(values[i], values[j])));
            assert(isfinite(d));
            assert(d >= -(float)M_PI && d <= (float)M_PI);
        }
    }
    assert(fabsf(utils_angle_difference_rad(100.0f, 0.0f) - (-0.530967f)) < 1e-5f);
    puts("Motor math tests passed");
    return 0;
}
