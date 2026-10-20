/*
 * Host stubs for motor_sensorless default wrappers (speed/protect side effects).
 */
#include <stdbool.h>
#include "motor_param.h"
#include "motor_types.h"
#include "motor_timing.h"

bool Motor_Timing_ProfileIsActive(void) { return false; }
bool host_observer_step_active;
motor_observer_step_sample_t host_observer_step;
unsigned host_observer_step_count;
bool Motor_Timing_ObserverStepActive(void) { return host_observer_step_active; }
bool Motor_Timing_ObserverStepDue(void) { return host_observer_step_active; }
void Motor_Timing_ObserverStepPush(const motor_observer_step_sample_t *sample)
{
    host_observer_step = *sample;
    host_observer_step_count++;
}
bool Motor_Timing_PhaseMode(void) { return false; }
bool Motor_Timing_PhaseDue(bool completed) { (void)completed; return false; }
void Motor_Timing_PhasePush(const motor_phase_sample_t *sample) { (void)sample; }
uint32_t Motor_Timing_ProfileTimeUs(void) { return 0U; }
void Motor_Timing_ProfileRecord(motor_timing_stage_t stage, uint32_t start)
{
    (void)stage;
    (void)start;
}

float host_speed_feedback_rpm;
bool host_speed_feedback_valid;
unsigned host_speed_feedback_calls;
mc_fault_code host_protect_fault;
unsigned host_protect_fault_calls;

void Motor_SpeedFeedback_Set(float rpm, bool valid)
{
    host_speed_feedback_rpm = rpm;
    host_speed_feedback_valid = valid;
    host_speed_feedback_calls++;
}

void Motor_SpeedLoop_PrimeIq(float iq)
{
    (void)iq;
}

void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor)
{
    (void)motor;
    host_protect_fault = fault;
    host_protect_fault_calls++;
}
