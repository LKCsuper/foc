#include "motor_reference_trig.h"
#if MOTOR_CFG_REFERENCE_TRIG
#include <math.h>

#if defined(__CC_ARM)
#pragma O2
#endif

#ifdef M_PI
#undef M_PI
#endif
#define M_PI motor_reference_pi
#define PI_2 motor_reference_pi_2
#define my_sin motor_reference_sin
#define my_cos motor_reference_cos
#define arctan motor_reference_arctan
#define arcsin motor_reference_arcsin
#define Q_rsqrt motor_reference_rsqrt
#include "../../../ThirtyLibrary/20261019WQFSBHXM/MyAlgorithm/HARDWARE/myalgorithm.c"
#undef M_PI
#undef PI_2
#undef my_sin
#undef my_cos
#undef arctan
#undef arcsin
#undef Q_rsqrt

void Motor_ReferenceTrigRaw(float angle, int terms, float *sine, float *cosine)
{
    *sine = motor_reference_sin(angle, terms);
    *cosine = motor_reference_cos(angle, terms);
}
#endif /* MOTOR_CFG_REFERENCE_TRIG */
