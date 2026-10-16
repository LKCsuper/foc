#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "motor_detect.h"
#include "hal_motor.h"

MOTOR_PARAM_S motor;
static motor_detect_result_t detected;
static bool active, running, faulted;
static float sample_hz = 30000.0f;
int Cmd_Autotune(int argc, char *argv[]);

motor_result_t Motor_GetCommand(const MOTOR_PARAM_S *m, motor_command_t *cmd)
{
    (void)m; memset(cmd, 0, sizeof(*cmd)); cmd->enable = running; return MOTOR_OK;
}
motor_result_t Motor_GetStatus(const MOTOR_PARAM_S *m, motor_status_t *status)
{
    (void)m; memset(status, 0, sizeof(*status)); status->fault_latched = faulted; return MOTOR_OK;
}
bool Motor_Detect_IsActive(void) { return active; }
void DETECT_GetResult(motor_detect_result_t *result) { *result = detected; }
float Hal_Motor_AdcSampleRateHz(void) { return sample_hz; }

int main(void)
{
    MOTOR_PARAM_S before;
    char *args[] = {"pitune", "150", "100", "60"};
    memset(&motor, 0, sizeof(motor));
    motor.ol_current_kp = 0.12f; motor.ol_current_ki = 18.0f;
    before = motor;
    assert(Cmd_Autotune(3, args) == -1);
    detected.version = MOTOR_DETECT_RESULT_VERSION;
    detected.resistance_valid = detected.inductance_valid = true;
    detected.resistance_ohm = 0.5f;
    detected.inductance_d_h = 0.001f; detected.inductance_q_h = 0.002f;
    assert(Cmd_Autotune(3, args) == 0);
    assert(Cmd_Autotune(4, args) == 0);
    active = true; assert(Cmd_Autotune(3, args) == -1); active = false;
    running = true; assert(Cmd_Autotune(3, args) == -1); running = false;
    faulted = true; assert(Cmd_Autotune(3, args) == -1); faulted = false;
    sample_hz = NAN; assert(Cmd_Autotune(3, args) == -1); sample_hz = 30000.0f;
    detected.error = MOTOR_DETECT_ERROR_USER_ABORT;
    assert(Cmd_Autotune(3, args) == -1); detected.error = MOTOR_DETECT_ERROR_NONE;
    args[1] = "150junk"; assert(Cmd_Autotune(3, args) == -1);
    args[1] = "nan"; assert(Cmd_Autotune(3, args) == -1);
    args[1] = "1000"; assert(Cmd_Autotune(3, args) == -1);
    args[1] = "150"; args[2] = "1000"; assert(Cmd_Autotune(3, args) == -1);
    args[2] = "1"; assert(Cmd_Autotune(3, args) == -1);
    assert(Cmd_Autotune(0, NULL) == -1);
    assert(memcmp(&motor, &before, sizeof(motor)) == 0);
    puts("autotune command: validation, rejection and read-only behavior passed");
    return 0;
}
