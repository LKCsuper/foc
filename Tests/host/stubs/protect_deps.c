/*
 * Host stubs for motor_protect default wrappers (sensorless/speed deps).
 * State-API tests do not call these paths; symbols exist for link completeness.
 */
#include <string.h>
#include <stdbool.h>
#include "motor_sensorless.h"
#include "motor_speed.h"

void Motor_Sensorless_GetDiagnostics(motor_sensorless_diagnostics_t *diagnostics)
{
    if (diagnostics != NULL) {
        memset(diagnostics, 0, sizeof(*diagnostics));
    }
}

float Motor_Sensorless_GetMechanicalRpm(void)
{
    return 0.0f;
}

float Motor_Sensorless_GetFluxMagnitude(void)
{
    return 0.0f;
}

float Motor_Sensorless_GetPhaseError(void)
{
    return 0.0f;
}

motor_sensorless_state_t Motor_Sensorless_GetState(void)
{
    return MOTOR_SENSORLESS_OPEN_LOOP;
}

float Motor_SpeedLoop_GetIqTarget(void)
{
    return 0.0f;
}

bool Motor_SpeedLoop_IsFeedbackValid(void)
{
    return false;
}
