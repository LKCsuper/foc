#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_output_core.h"

int main(void)
{
    motor_control_result_t proposed = {MOTOR_CONTROL_ACTION_OUTPUT_DQ, 1.0f, 2.0f, 0.3f};
    motor_control_result_t result;
    motor_output_sample_t sample = {750U, 250U, 500U, 1.0f, -1.0f, 0.0f};
    float alpha, beta;
    result = Motor_Control_ArbitrateOutput(true, &proposed);
    assert(result.action == MOTOR_CONTROL_ACTION_SAFE && result.vd == 0.0f && result.vq == 0.0f);
    result = Motor_Control_ArbitrateOutput(false, &proposed);
    assert(result.action == proposed.action && result.theta == proposed.theta);
    assert(Motor_Control_ArbitrateOutput(false, NULL).action == MOTOR_CONTROL_ACTION_NONE);
    assert(Motor_Control_ArbitrateOutput(true, NULL).action == MOTOR_CONTROL_ACTION_SAFE);
    assert(Motor_OutputCore_EstimateVoltage(&sample, 1000U, 24.0f, 0.0f, &alpha, &beta));
    assert(fabsf(alpha - 6.0f) < 1e-6f);
    assert(fabsf(beta + 3.4641016f) < 1e-5f);
    assert(Motor_OutputCore_EstimateVoltage(&sample, 1000U, 24.0f, 0.01f, &alpha, &beta));
    assert(fabsf(alpha - 5.84f) < 1e-5f);
    sample.duty1 = 1001U;
    assert(!Motor_OutputCore_EstimateVoltage(&sample, 1000U, 24.0f, 0.0f, &alpha, &beta));
    assert(alpha == 0.0f && beta == 0.0f);
    sample.duty1 = 500U;
    sample.ia = NAN;
    assert(Motor_OutputCore_EstimateVoltage(&sample, 1000U, 24.0f, 0.0f, &alpha, &beta));
    assert(!Motor_OutputCore_EstimateVoltage(&sample, 1000U, 24.0f, 0.01f, &alpha, &beta));
    assert(!Motor_OutputCore_EstimateVoltage(&sample, 0U, 24.0f, 0.0f, &alpha, &beta));
    puts("Output arbitration/voltage standalone tests passed");
    return 0;
}
