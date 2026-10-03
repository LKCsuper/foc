#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_output.h"
#include "motor_feedback.h"

static motor_feedback_source_t s_source;
static unsigned s_vesc_calls;
static unsigned s_hqd_calls;
static unsigned s_commit_calls;
static uint32_t s_pwm_a;
static uint32_t s_pwm_b;
static uint32_t s_pwm_c;
static float s_applied_alpha;
static float s_applied_beta;

uint32_t Hal_Motor_PwmPeriodCounts(void) { return 1000U; }
void Hal_Motor_PwmSetDuty(uint32_t a, uint32_t b, uint32_t c)
{
    s_pwm_a = a;
    s_pwm_b = b;
    s_pwm_c = c;
}
void Hal_Motor_PwmCommit(void) { s_commit_calls++; }
motor_feedback_source_t Motor_Feedback_GetSourceDefault(void) { return s_source; }
void Motor_Sensorless_SetAppliedVoltage(float alpha, float beta)
{
    s_applied_alpha = alpha;
    s_applied_beta = beta;
}
void inverse_park(float vd, float vq, float theta, float *alpha, float *beta)
{
    (void)theta;
    *alpha = vd;
    *beta = vq;
}
void foc_svm_from_voltage(float alpha, float beta, float v_bus, float max_duty, uint32_t period,
                          uint32_t *a, uint32_t *b, uint32_t *c, uint32_t *sector)
{
    (void)alpha;
    (void)beta;
    (void)v_bus;
    (void)max_duty;
    (void)period;
    s_vesc_calls++;
    *a = 11U;
    *b = 22U;
    *c = 33U;
    *sector = 1U;
}
void svm_hqd(float alpha, float beta, float v_bus, uint32_t period,
             uint32_t *a, uint32_t *b, uint32_t *c, uint32_t *sector)
{
    (void)alpha;
    (void)beta;
    (void)v_bus;
    (void)period;
    s_hqd_calls++;
    *a = 101U;
    *b = 202U;
    *c = 303U;
    *sector = 2U;
}

int main(void)
{
    MOTOR_PARAM_S motor;
    motor_control_result_t result;

    memset(&motor, 0, sizeof(motor));
    Motor_OutputSafe(&motor);
    assert(motor.duty1 == 500U && motor.duty2 == 500U && motor.duty3 == 500U);
    assert(s_pwm_a == 500U && s_pwm_b == 500U && s_pwm_c == 500U);
    assert(s_applied_alpha == 0.0f && s_applied_beta == 0.0f && s_commit_calls == 1U);

    motor.ol_mode = MOTOR_OL_MODE_IF;
    s_source = MOTOR_FEEDBACK_SRC_ENCODER;
    Motor_OutputVoltage(&motor, 2.0f, -3.0f, 1.0f, 24.0f);
    assert(s_vesc_calls == 1U && s_hqd_calls == 0U);
    assert(motor.duty1 == 11U && motor.duty2 == 22U && motor.duty3 == 33U);
    assert(s_pwm_a == 11U && s_pwm_b == 33U && s_pwm_c == 22U);
    assert(s_applied_alpha == 2.0f && s_applied_beta == -3.0f);

    motor.ol_mode = MOTOR_OL_MODE_VF;
    s_source = MOTOR_FEEDBACK_SRC_OPENLOOP;
    result.action = MOTOR_CONTROL_ACTION_OUTPUT_DQ;
    result.vd = 1.0f;
    result.vq = 4.0f;
    result.theta = 0.5f;
    Motor_OutputApplyControlResult(&motor, 24.0f, &result);
    assert(s_vesc_calls == 1U && s_hqd_calls == 1U);
    assert(s_pwm_a == 101U && s_pwm_b == 303U && s_pwm_c == 202U);

    result.action = MOTOR_CONTROL_ACTION_NONE;
    Motor_OutputApplyControlResult(&motor, 24.0f, &result);
    assert(s_commit_calls == 3U);
    puts("test_motor_output: pass");
    return 0;
}
