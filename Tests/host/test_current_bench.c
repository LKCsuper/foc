#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "motor_param.h"
#include "motor_current.h"
#include "motor_pid.h"
#include "motor_control.h"
#include "motor_feedback.h"
#include "motor_foc.h"

typedef struct { bool enable; float target_rpm; } motor_command_t;
#define MOTOR_OK 0
MOTOR_PARAM_S motor;
static MOTOR_PARAM_S test_motor;
static motor_command_t command;
static unsigned blocked, stopped, calls, lines, depth;
static int get_result;
static unsigned resets, begins, ends;
static unsigned probe_calls, capture_calls;
static bool detailed;
static bool arm_fail;
static unsigned math_calls;
static int Motor_GetCommand(MOTOR_PARAM_S *instance, motor_command_t *out)
{ assert(instance == &motor); *out = command; return get_result; }
static bool Motor_Position_IsEnabled(void) { return blocked == 1U; }
static bool Motor_Joint_IsEnabled(void) { return blocked == 2U; }
static bool Motor_Bldc_IsSelected(void) { return blocked == 3U; }
static bool Motor_Detect_IsActive(void) { return blocked == 4U; }
static bool Motor_Protect_IsFaultLatched(void) { return blocked == 5U; }
motor_feedback_source_t Motor_Feedback_GetSourceDefault(void) { return blocked == 6U ? MOTOR_FEEDBACK_SRC_ENCODER : MOTOR_FEEDBACK_SRC_SENSORLESS; }
void Motor_Control_Init(motor_control_t *ctrl) { memset(ctrl, 0, sizeof(*ctrl)); }
motor_control_result_t Motor_Control_IfDiagnosticStep(motor_control_t *ctrl, MOTOR_PARAM_S *instance,
                                                     motor_current_target_ctx_t *ctx, unsigned mode)
{ motor_control_result_t result = {0}; assert(ctrl && instance != &motor && ctx && mode <= 8U); calls++; return result; }
static void Motor_Timing_CadenceStop(void) { stopped++; }
static void Motor_Timing_HandoffStop(void) { stopped++; }
static void Motor_Timing_CadenceReset(void) { resets++; }
static void Motor_Timing_CallbackStop(void) { }
static void Motor_Timing_ProfileSetDetailed(bool enabled) { detailed = enabled; }
static bool Motor_Timing_ProfileBegin(void) { assert(depth == 1U && detailed); begins++; return !arm_fail; }
static void Motor_Timing_ProfileEnd(void) { assert(depth == 1U); ends++; }
static bool Motor_Timing_ProfileDetailedIsActive(void) { probe_calls++; return false; }
static bool Motor_Timing_HandoffDue(void) { capture_calls++; return false; }
static uint32_t Motor_Crit_Enter(void) { assert(depth == 0U); depth++; return 0U; }
static void Motor_Crit_Exit(uint32_t key) { assert(key == 0U && depth == 1U); depth--; }
static uint32_t Hal_Motor_TimeUs(void) { static uint32_t now; return now++; }
static void print_line(const char *format, ...) { (void)format; assert(depth == 0U); lines++; }
#define PRINTF print_line
void sincos_f32(float angle, float *sine, float *cosine)
{ assert(depth == 1U); math_calls++; *sine = angle; *cosine = 1.0f; }
void sincos_taylor_f32(float angle, float *sine, float *cosine)
{ sincos_f32(angle, sine, cosine); }
void Motor_ReferenceTrigRaw(float angle, int terms, float *sine, float *cosine)
{ assert(terms == 3 || terms == 5 || terms == 7 || terms == 9 || terms == 12); sincos_f32(angle, sine, cosine); }
void Motor_Current_FeedbackUpdateDq(MOTOR_PARAM_S *instance, float theta)
{ assert(instance != &motor && theta == 0.6f); instance->Id = 0.1f; instance->Iq = 0.2f; calls++; }
void Motor_Current_GetTargets(const MOTOR_PARAM_S *instance, const motor_current_target_ctx_t *ctx, float *id, float *iq)
{ assert(instance != &motor && ctx->soft_current_scale == 1.0f); *id = 0.0f; *iq = 1.0f; calls++; }
float Motor_Current_PI_Run(float target, float measured, float *integral, float kp, float ki, float dt, float limit)
{ assert(kp == 0.12f && ki == 18.0f && dt > 0.0f && limit == 4.0f); calls++; return *integral + target - measured; }
void Motor_Current_LimitVoltage(MOTOR_PARAM_S *instance, motor_dq_voltage_t *voltage, float id, float iq, float limit)
{ assert(instance != &motor && voltage && limit == 4.0f); (void)id; (void)iq; calls++; }
motor_dq_voltage_t Motor_Current_RunPi(MOTOR_PARAM_S *instance, float id, float iq, float dt, float limit)
{ motor_dq_voltage_t voltage = {id, iq}; assert(instance != &motor && dt > 0.0f && limit == 4.0f); calls++; return voltage; }
motor_dq_voltage_t Motor_Current_Step(MOTOR_PARAM_S *instance, float theta, const motor_current_target_ctx_t *ctx,
                                     float dt, float limit, float *id, float *iq)
{ assert(theta == 0.6f && ctx && ((id && iq) || (!id && !iq))); return Motor_Current_RunPi(instance, id ? *id : 0.0f, iq ? *iq : 1.0f, dt, limit); }
#include "current_bench_under_test.inc"
#include "trig_bench_under_test.inc"

int main(void)
{
    MOTOR_PARAM_S before;
    unsigned index;
    for (index = 1U; index <= 6U; ++index) {
        blocked = index;
        assert(ol_current_bench() == -1 && calls == 0U && stopped == 0U);
    }
    blocked = 0U;
    command.enable = true; assert(ol_current_bench() == -1);
    command.enable = false; command.target_rpm = 300.0f; assert(ol_current_bench() == -1);
    command.target_rpm = 0.0f; motor.ol_enable = true; assert(ol_current_bench() == -1);
    motor.ol_enable = false; get_result = -1; assert(ol_current_bench() == -1);
    get_result = MOTOR_OK; before = motor; lines = 0U;
    assert(ol_current_bench() == 0);
    assert(calls == 16898U && stopped == 4U && lines == 49U && depth == 0U);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    for (index = 1U; index <= 5U; ++index) {
        blocked = index;
        assert(ol_trig_bench() == -1 && math_calls == 0U);
    }
    blocked = 0U; lines = 0U;
    assert(ol_trig_bench() == 0);
    assert(math_calls == 10752U && lines == 25U && depth == 0U);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    assert(probe_calls == 1024U && capture_calls == 1024U);
    assert(resets == 2U && begins == 128U && ends == 128U && !detailed);
    arm_fail = true;
    assert(ol_current_bench() == -1);
    assert(begins == 384U && ends == 128U && !detailed && depth == 0U);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    puts("Stopped current benchmark tests passed");
    return 0;
}
