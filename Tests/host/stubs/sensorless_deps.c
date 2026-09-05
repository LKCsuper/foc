/*
 * Host stubs for motor_sensorless default wrappers (speed/protect side effects).
 */
#include <stdbool.h>
#include "motor_param.h"
#include "motor_types.h"

void Motor_SpeedFeedback_Set(float rpm, bool valid)
{
    (void)rpm;
    (void)valid;
}

void Motor_SpeedLoop_PrimeIq(float iq)
{
    (void)iq;
}

void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor)
{
    (void)fault;
    (void)motor;
}
