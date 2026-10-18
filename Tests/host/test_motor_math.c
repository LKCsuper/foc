#include <assert.h>
#include <float.h>
#include <stdio.h>
#include "motor_math.h"

static float original_difference(float a, float b)
{
    float d = a - b;
    while (d < -M_PI) d += 2.0f * M_PI;
    while (d > M_PI) d -= 2.0f * M_PI;
    return d;
}

int main(void)
{
    /* 单圈控制输入与旧实现逐值一致，包括反转及跨零点。 */
    for (int i = -256; i <= 256; ++i) {
        for (int j = -256; j <= 256; ++j) {
            float a = (float)i * (2.0f * M_PI / 256.0f);
            float b = (float)j * (2.0f * M_PI / 256.0f);
            assert(fabsf(utils_angle_difference_rad(a, b) - original_difference(a, b)) < 2e-6f);
        }
    }
    assert(utils_angle_difference_rad((float)M_PI, 0.0f) == (float)M_PI);
    assert(utils_angle_difference_rad(-(float)M_PI, 0.0f) == -(float)M_PI);
    assert(isnan(utils_angle_difference_rad(INFINITY, 0.0f)));
    assert(isnan(utils_angle_difference_rad(0.0f, -INFINITY)));
    assert(isnan(utils_angle_difference_rad(NAN, 0.0f)));
    const float values[] = {FLT_MAX, -FLT_MAX, 1e10f, -1e10f, 0.0f};
    for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        for (unsigned j = 0; j < sizeof(values) / sizeof(values[0]); ++j) {
            float d = utils_angle_difference_rad(values[i], values[j]);
            assert(isfinite(d));
            assert(d >= -(float)M_PI && d <= (float)M_PI);
        }
    }
    assert(fabsf(utils_angle_difference_rad(100.0f, 0.0f) - (-0.530967f)) < 1e-5f);
    puts("Motor math tests passed");
    return 0;
}
