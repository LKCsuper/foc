#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "motor_mt6701.h"

static uint8_t s_registers[256];
static uint8_t s_read_order[4];
static uint32_t s_read_count;
static uint16_t s_published_angle;
static bool s_fail_reads;
static uint32_t s_time_us;

uint32_t Hal_Motor_TimeUs(void)
{
    s_time_us += 100U;
    return s_time_us;
}

void Hal_Encoder_Init(void)
{
}

bool Hal_Encoder_I2cReadRegister(uint8_t address_7bit, uint8_t reg, uint8_t *value)
{
    assert(address_7bit == 0x06U);
    if (s_fail_reads) {
        return false;
    }
    s_read_order[s_read_count++] = reg;
    *value = s_registers[reg];
    return true;
}

void Hal_Encoder_SsiInit(void)
{
}

bool Hal_Encoder_SsiReadFrame(uint32_t *frame)
{
    (void)frame;
    return false;
}

void Hal_Encoder_SetCount(uint16_t count)
{
    s_published_angle = count;
}

int main(void)
{
    motor_mt6701_status_t status;

    Motor_MT6701_Init();
    assert(!Motor_MT6701_IsValid());

    s_registers[0x03U] = 0xABU;
    s_registers[0x04U] = 0xC0U;
    assert(Motor_MT6701_Poll());
    assert(s_read_count == 2U);
    assert(s_read_order[0] == 0x03U);
    assert(s_read_order[1] == 0x04U);
    assert(s_published_angle == (uint16_t)((0xABU << 6U) | (0xC0U >> 2U)));
    assert(Motor_MT6701_IsValid());
    assert(Motor_MT6701_GetSampleSequence() == 1U);

    s_fail_reads = true;
    assert(!Motor_MT6701_Poll());
    assert(Motor_MT6701_IsValid());
    assert(!Motor_MT6701_Poll());
    assert(Motor_MT6701_IsValid());
    assert(!Motor_MT6701_Poll());
    assert(!Motor_MT6701_IsValid());

    Motor_MT6701_GetStatus(&status);
    assert(status.success_count == 1U);
    assert(status.error_count == 3U);
    assert(status.consecutive_errors == 3U);
    assert(status.sample_sequence == 1U);
    assert(status.sample_time_us == 100U);
    puts("test_motor_mt6701: pass");
    return 0;
}
