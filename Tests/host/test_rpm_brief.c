#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
typedef unsigned char UCHAR;
typedef unsigned long ULONG;
typedef unsigned mc_fault_code;
typedef struct { unsigned state; float rpm; } motor_sensorless_light_snapshot_t;
static struct { float ol_target_rpm, Iq; bool sensorless_takeover; } motor;
static motor_sensorless_light_snapshot_t light;
static char output[192];
static unsigned crit, writes, flushes;
static mc_fault_code fault;
static uint32_t Motor_Crit_Enter(void) { assert(!crit); crit = 1U; return 0U; }
static void Motor_Crit_Exit(uint32_t key) { assert(key == 0U && crit); crit = 0U; }
static void Motor_Sensorless_GetLightSnapshot(motor_sensorless_light_snapshot_t *out) { assert(crit); *out = light; }
static float Motor_SpeedLoop_GetMeasuredRpm(void) { assert(crit); return 0.0f; }
static bool Motor_SpeedLoop_IsFeedbackValid(void) { assert(crit); return false; }
static mc_fault_code Motor_Protect_GetFault(void) { assert(crit); return fault; }
static const char *ol_sensorless_state_name(unsigned state) { assert(!crit && state == 0U); return "OPEN_LOOP"; }
static ULONG Bsp_Usart_Write(const UCHAR *data, ULONG size)
{
    assert(!crit && size < sizeof(output));
    memcpy(output, data, size);
    output[size] = '\0';
    writes++;
    return size;
}
static void Bsp_Usart_Flush(void) { assert(!crit); flushes++; }
#include "rpm_brief_under_test.inc"
int main(void)
{
    motor.ol_target_rpm = 300.0f;
    motor.Iq = 1.025f;
    light.rpm = 299.5f;
    assert(ol_print_rpm_brief() == 0);
    assert(strstr(output, "rpm obs=299.5 meas=0.0 tgt=300 iq=1.025 OPEN_LOOP tk=0 fb=invalid fault=0\r\n"));
    assert(strchr(output, '\n') == output + strlen(output) - 1U);
    assert(!strstr(output, "prof") && !strstr(output, "enc"));
    fault = 4U;
    motor.sensorless_takeover = true;
    assert(ol_print_rpm_brief() == 0);
    assert(strstr(output, "tk=1 fb=invalid fault=4"));
    assert(writes == 2U && flushes == 2U && crit == 0U);
    puts("test_rpm_brief: pass");
    return 0;
}
