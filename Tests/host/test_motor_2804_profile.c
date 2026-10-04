#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "motor_2804_220kv.h"

static int Near(float a, float b)
{
    return fabsf(a - b) < 1.0e-7f;
}

int main(void)
{
    assert(MOTOR_2804_220KV_POLE_COUNT == 14U);
    assert(Near(MOTOR_FIXED_POLE_PAIRS, 7.0f));
    assert(Near(MOTOR_FIXED_RATED_VOLTAGE, 12.0f));
    assert(Near(MOTOR_FIXED_RATED_CURRENT, 0.5f));
    assert(Near(MOTOR_FIXED_PEAK_CURRENT, 2.0f));
    assert(Near(MOTOR_FIXED_LINE_RESISTANCE, 2.55f));
    assert(Near(MOTOR_FIXED_LINE_INDUCTANCE, 0.00086f));
    assert(Near(MOTOR_FIXED_FLUX_LINKAGE, 0.0035f));
    assert(Near(MOTOR_OPEN_LOOP_IQ_DEFAULT, 0.30f));
    assert(MOTOR_OPEN_LOOP_IQ_DEFAULT <= MOTOR_FIXED_RATED_CURRENT);
    puts("test_motor_2804_profile: pass");
    return 0;
}
