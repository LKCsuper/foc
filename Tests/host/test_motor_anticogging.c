#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_anticogging.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void)
{
    motor_anticogging_status_t status;
    unsigned pass;
    unsigned bin;

    Motor_Anticogging_Init();
    assert(Motor_Anticogging_Start(2U));
    for (pass = 0U; pass < 2U; ++pass) {
        for (bin = 0U; bin < MOTOR_ANTICOGGING_BINS; ++bin) {
            float angle = ((float)bin + 0.25f) * 2.0f * (float)M_PI /
                          (float)MOTOR_ANTICOGGING_BINS;
            float correction = 0.1f + 0.2f * sinf(angle);
            Motor_Anticogging_LearnStep(angle, correction);
        }
    }
    Motor_Anticogging_GetStatus(&status);
    assert(!status.learning && status.calibrated && status.enabled);
    assert(status.min_samples == 2U);
    assert(fabsf(Motor_Anticogging_GetVoltage(0.5f * (float)M_PI) - 0.2f) < 0.02f);
    assert(Motor_Anticogging_SetEnabled(false));
    assert(fabsf(Motor_Anticogging_GetVoltage(0.5f * (float)M_PI)) < 1e-6f);
    assert(Motor_Anticogging_SetEnabled(true));
    Motor_Anticogging_Clear();
    assert(!Motor_Anticogging_SetEnabled(true));
    puts("test_motor_anticogging: pass");
    return 0;
}
