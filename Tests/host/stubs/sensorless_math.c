/*
 * Minimal math stubs for sensorless host tests (avoid full motor_math.c / IN macro).
 */
#include <math.h>
#include "motor_foc.h"

#ifndef ONE_BY_SQRT3
#define ONE_BY_SQRT3 (0.57735026919f)
#endif

void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
    *Ialpha = (2.0f / 3.0f) * Ia - (1.0f / 3.0f) * Ib - (1.0f / 3.0f) * Ic;
    *Ibeta = (Ib - Ic) * ONE_BY_SQRT3;
}

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

float utils_fast_atan2(float y, float x)
{
    return atan2f(y, x);
}
