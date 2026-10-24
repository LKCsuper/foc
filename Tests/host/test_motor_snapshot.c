#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "motor_speed.h"

#define MOTOR_CRIT_H
static unsigned critical_depth;
static bool in_isr;
static unsigned feedback_reads;
static unsigned speed_reads;
static bool speed_available = true;
static motor_speed_snapshot_t speed_frame;
static mc_fault_code fault;

static uint32_t Motor_Crit_Enter(void) { return critical_depth++; }
static void Motor_Crit_Exit(uint32_t previous)
{
    assert(critical_depth == previous + 1U);
    critical_depth = previous;
}
bool Motor_SpeedLoop_GetSnapshot(motor_speed_snapshot_t *out)
{
    assert(!in_isr && critical_depth != 0U);
    speed_reads++;
    *out = speed_frame;
    return speed_available;
}
void Motor_Feedback_SampleDefault(const MOTOR_PARAM_S *motor, float theta, motor_feedback_sample_t *out)
{
    (void)motor;
    assert(in_isr || critical_depth != 0U);
    feedback_reads++;
    memset(out, 0, sizeof(*out));
    out->elec_angle_rad = theta;
    out->used_source = MOTOR_FEEDBACK_SRC_SENSORLESS;
}
bool Motor_OpenLoop_IsAligning(void) { return false; }
float Motor_OpenLoop_GetAlignElapsed(void) { return 0.0f; }
mc_fault_code Motor_Protect_GetFault(void) { return fault; }
bool Motor_Protect_IsFaultLatched(void) { return fault != FAULT_CODE_NONE; }

#include "../../App/motor/motor_snapshot.c"

int main(void)
{
    MOTOR_PARAM_S motor = {0};
    motor_status_t status;
    motor.ol_current_rpm = 72.0f;
    motor.v_bus = 24.0f;
    motor.currentAdc[0] = 2037.0f;
    motor.ol_theta = 0.7f;
    speed_frame.target_rpm = 100.0f;
    speed_frame.measured_rpm = 123.0f;
    speed_frame.error = -23.0f;
    speed_frame.iq_target = 0.45f;
    assert(Motor_GetStatus(NULL, &status) == MOTOR_ERR_NULL);
    assert(Motor_GetStatus(&motor, NULL) == MOTOR_ERR_NULL);
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.current_rpm == 72.0f && status.measured_rpm == 123.0f);
    assert(feedback_reads == 1U && speed_reads == 1U && critical_depth == 0U);

    fault = FAULT_CODE_ABS_OVER_CURRENT;
    in_isr = true;
    Motor_StatusSnapshotPublishFromIsr(&motor);
    in_isr = false;
    assert(speed_reads == 1U && feedback_reads == 2U);
    motor.v_bus = 20.0f;
    motor.currentAdc[0] = 1000.0f;
    motor.ol_current_rpm = 300.0f;
    fault = FAULT_CODE_NONE;
    speed_frame.measured_rpm = 456.0f;
    speed_frame.error = -356.0f;
    speed_frame.iq_target = 0.8f;
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.v_bus == 24.0f && status.current_adc[0] == 2037.0f && status.current_rpm == 72.0f);
    assert(status.feedback.elec_angle_rad == 0.7f && status.fault_latched);
    assert(status.fault == FAULT_CODE_ABS_OVER_CURRENT);
    assert(status.measured_rpm == 456.0f && status.speed_error == -356.0f && status.speed_iq == 0.8f);
    assert(speed_reads == 2U && feedback_reads == 2U && critical_depth == 0U);

    speed_available = false;
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.measured_rpm == 0.0f && status.speed_error == 0.0f && status.speed_iq == 0.0f);
    in_isr = true;
    Motor_StatusSnapshotPublishFromIsr(&motor);
    in_isr = false;
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.v_bus == 20.0f && status.current_rpm == 300.0f && !status.fault_latched);
    speed_available = true;
    motor.speed_enable = true;
    in_isr = true;
    Motor_StatusSnapshotPublishFromIsr(&motor);
    in_isr = false;
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.current_rpm == 100.0f && status.speed_enable);
    assert(critical_depth == 0U);
    puts("Motor status split snapshot tests passed");
    return 0;
}
