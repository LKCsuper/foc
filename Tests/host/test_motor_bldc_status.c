#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motor_bldc_status.h"

bool Motor_BldcSixStep_IsRunning(const motor_bldc_six_step_t *control)
{
    return control != NULL && control->running;
}

static int Near(float a, float b)
{
    return fabsf(a - b) < 1.0e-7f;
}

int main(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_status_t status;

    memset(&commutator, 0, sizeof(commutator));
    memset(&sensorless, 0, sizeof(sensorless));
    memset(&status, 0xA5, sizeof(status));

    assert(!Motor_BldcStatus_Fill(NULL, true, true, &commutator, &sensorless,
                                  FAULT_CODE_NONE, 0U));
    assert(Motor_BldcStatus_Fill(&status, true, false, NULL, NULL,
                                 FAULT_CODE_ABS_OVER_CURRENT, 99U));
    assert(status.selected && !status.running);
    assert(status.motor_fault == FAULT_CODE_NONE && status.bemf_sample_count == 0U);

    commutator.running = true;
    commutator.duty = 0.09f;
    sensorless.target_duty = 0.10f;
    sensorless.step = 3U;
    sensorless.direction = MOTOR_BLDC_DIRECTION_CCW;
    sensorless.mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
    sensorless.fault = MOTOR_BLDC_SENSORLESS_FAULT_BEMF_LOST;
    sensorless.filtered_bemf_v = 1.25f;
    sensorless.config.bemf_integral_threshold_vs = 0.004f;
    sensorless.config.integral_auto_detect_enabled = true;
    sensorless.detected_integral_mask = 0x3FU;
    sensorless.step_detected_integral_vs[3] = 0.0025f;
    sensorless.step_integral_commutations[3] = 17U;
    sensorless.step_bemf_waveform_v[3][2] = 2.75f;
    sensorless.step_candidate_min_time_s[3][1] = 0.0012f;

    assert(Motor_BldcStatus_Fill(&status, true, false, &commutator, &sensorless,
                                 FAULT_CODE_ABS_OVER_CURRENT, 1234U));
    assert(status.selected && status.running && !status.sample_upward);
    assert(Near(status.duty, 0.09f) && Near(status.target_duty, 0.10f));
    assert(status.step == 3U && status.direction == MOTOR_BLDC_DIRECTION_CCW);
    assert(status.mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP);
    assert(status.fault == MOTOR_BLDC_SENSORLESS_FAULT_BEMF_LOST);
    assert(status.motor_fault == FAULT_CODE_ABS_OVER_CURRENT);
    assert(status.bemf_sample_count == 1234U && Near(status.bemf_v, 1.25f));
    assert(Near(status.bemf_integral_threshold_vs, 0.0025f));
    assert(status.step_integral_commutations[3] == 17U);
    assert(Near(status.step_bemf_waveform_v[3][2], 2.75f));
    assert(Near(status.step_candidate_min_time_s[3][1], 0.0012f));

    sensorless.detected_integral_mask = 0x07U;
    assert(Motor_BldcStatus_Fill(&status, false, true, &commutator, &sensorless,
                                 FAULT_CODE_NONE, 1U));
    assert(Near(status.bemf_integral_threshold_vs, 0.004f));
    puts("test_motor_bldc_status: pass");
    return 0;
}
