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
    assert(Near(MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG, 1.0f));
    assert(Near(MOTOR_POSITION_VEL_KP_DEFAULT, 0.020f));
    assert(Near(MOTOR_POSITION_VEL_KI_DEFAULT, 0.02f));
    assert(Near(MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT, 0.35f));
    assert(Near(MOTOR_POSITION_ACCEL_RPM_S_DEFAULT, 30.0f));
    assert(Near(MOTOR_POSITION_FF_V_PER_RPM_DEFAULT, 0.0f));
    assert(Near(MOTOR_ENCODER_PLL_BW_RAD_S, 30.0f));
    assert(Near(MOTOR_ENCODER_SPEED_VOLT_PER_AMP, 2.55f));
    assert(MOTOR_ENCODER_SPEED_VOLTAGE_MODE == 1U);
    assert(Near(MOTOR_ENCODER_SPEED_VOLT_LIMIT_V, 9.0f));
    assert(Near(MOTOR_ENCODER_SPEED_FF_V_PER_RPM, 0.00308f));
    assert(Near(MOTOR_POSITION_MAX_RPM_DEFAULT, 2.0f));
    assert(Near(MOTOR_POSITION_MAX_RPM_ALLOWED, 2.0f));
    assert(MOTOR_POSITION_HYBRID_SPEED_ENABLE == 0U);
    assert(MOTOR_POSITION_SMALL_TEST_SEQUENCE == 1U);
    assert(MOTOR_JOINT_TRAJECTORY_ENABLE == 1U);
    assert(Near(MOTOR_JOINT_TRAJECTORY_VEL_DEG_S, 1800.0f));
    assert(Near(MOTOR_JOINT_TRAJECTORY_ACCEL_DEG_S2, 3600.0f));
    assert(Near(MOTOR_JOINT_TRAJECTORY_DECEL_DEG_S2, 5400.0f));
    puts("test_motor_2804_profile: pass");
    return 0;
}
