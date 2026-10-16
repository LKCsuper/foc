/*
 * Clarke/Park stubs for motor_current host tests.
 */
#include <math.h>
#include "motor_foc.h"

float sin_f32(float angle) { return sinf(angle); }
float cos_f32(float angle) { return cosf(angle); }

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
#include "motor_timing.h"
bool Motor_Timing_ProfileIsActive(void) { return false; }
bool Motor_Timing_HandoffDue(void) { return false; }
void Motor_Timing_HandoffBefore(const MOTOR_PARAM_S *motor, float theta, float delta, float id, float iq, float dt)
{ (void)motor; (void)theta; (void)delta; (void)id; (void)iq; (void)dt; }
void Motor_Timing_HandoffTransport(const MOTOR_PARAM_S *motor) { (void)motor; }
void Motor_Timing_HandoffAfter(const MOTOR_PARAM_S *motor, float vd, float vq)
{ (void)motor; (void)vd; (void)vq; }
bool Motor_Timing_ProfileDetailedIsActive(void) { return false; }
uint32_t Motor_Timing_ProfileTimeUs(void) { return 0U; }
void Motor_Timing_ProfileRecord(motor_timing_stage_t stage, uint32_t start) { (void)stage; (void)start; }
