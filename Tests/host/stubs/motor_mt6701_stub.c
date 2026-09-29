#include <stdbool.h>
#include "motor_mt6701.h"

void Motor_MT6701_Init(void)
{
}

bool Motor_MT6701_Poll(void)
{
    return true;
}

void Motor_MT6701_GetStatus(motor_mt6701_status_t *status)
{
    (void)status;
}

bool Motor_MT6701_IsValid(void)
{
    return true;
}

uint32_t Motor_MT6701_GetSampleSequence(void)
{
    static uint32_t sequence;
    return ++sequence;
}

const char *Motor_MT6701_InterfaceName(void)
{
    return "HOST";
}
