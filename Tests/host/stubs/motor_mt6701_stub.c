#include <stdbool.h>
#include "motor_mt6701.h"
static motor_mt6701_status_t s_status;
void Test_MT6701_SetSample(uint32_t sequence, uint32_t timestamp)
{
    s_status.valid = true;
    s_status.sample_sequence = sequence;
    s_status.sample_time_us = timestamp;
}

void Motor_MT6701_Init(void)
{
}

bool Motor_MT6701_Poll(void)
{
    return true;
}

void Motor_MT6701_GetStatus(motor_mt6701_status_t *status)
{
    *status = s_status;
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
