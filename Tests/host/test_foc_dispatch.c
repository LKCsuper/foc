#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define COMMON_ASSERT(value) assert(value)
typedef struct { unsigned unused; } MOTOR_PARAM_S;
typedef unsigned motor_feedback_source_t;
static bool s_execute;
static unsigned s_runs, s_publishes, s_gates;
volatile uint8_t g_motor_timing_split_active;
static unsigned s_class_full, s_class_skip, s_publish_notes;
void Motor_Timing_CallbackClassify(bool full)
{
    if (full) s_class_full++;
    else s_class_skip++;
}
void Motor_Timing_CallbackPublish(uint32_t elapsed_us)
{
    (void)elapsed_us;
    s_publish_notes++;
}
static motor_feedback_source_t s_source;
static uint32_t Hal_Motor_TimeUs(void) { return 1234U; }
static motor_feedback_source_t Motor_Feedback_GetSourceDefault(void) { return s_source; }
static bool Motor_FocIsr_ShouldExecuteControl(motor_feedback_source_t source)
{
    assert(source == s_source);
    s_gates++;
    return s_execute;
}
static void Motor_FocIsr_Run(MOTOR_PARAM_S *motor, motor_feedback_source_t source, uint32_t start)
{
    assert(motor != NULL && source == s_source && start == 1234U);
    s_runs++;
}
static void Motor_FocIsr_PublishStatus(const MOTOR_PARAM_S *motor) { assert(motor != NULL); s_publishes++; }
#include "foc_dispatch_under_test.inc"

static unsigned s_status_frames;
static void Motor_StatusSnapshotPublishFromIsr(const MOTOR_PARAM_S *motor)
{
    assert(motor != NULL);
    s_status_frames++;
}
#define Motor_FocIsr_PublishStatus Motor_FocIsr_PublishStatusUnderTest
#include "status_publish_under_test.inc"
#undef Motor_FocIsr_PublishStatus

int main(void)
{
    MOTOR_PARAM_S motor = {0};
    unsigned source;
    for (source = 0U; source < 4U; source++) {
        s_source = source;
        s_execute = false;
        Motor_FocIsr_Step(&motor);
        assert(s_runs == source);
        s_execute = true;
        Motor_FocIsr_Step(&motor);
        assert(s_runs == source + 1U);
        assert(s_gates == 2U * (source + 1U) && s_publishes == s_gates);
    }
    for (source = 1U; source <= 900U; source++) {
        Motor_FocIsr_PublishStatusUnderTest(&motor);
        assert(s_status_frames == source / 300U);
    }
    g_motor_timing_split_active = 1U;
    s_execute = false;
    Motor_FocIsr_Step(&motor);
    s_execute = true;
    Motor_FocIsr_Step(&motor);
    assert(s_class_skip == 1U && s_class_full == 1U);
    for (source = 0U; source < 300U; source++) Motor_FocIsr_PublishStatusUnderTest(&motor);
    assert(s_status_frames == 4U && s_publish_notes == 1U);
    puts("test_foc_dispatch: pass");
    return 0;
}
