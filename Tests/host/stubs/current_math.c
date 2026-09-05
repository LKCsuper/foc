/*
 * Clarke/Park stubs for motor_current host tests.
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

void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq)
{
    float s = sinf(Theta);
    float c = cosf(Theta);

    *Id = Ialpha * c + Ibeta * s;
    *Iq = -Ialpha * s + Ibeta * c;
}
