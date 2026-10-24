#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define COMMON_ASSERT(value) assert(value)
#define MOTOR_CFG_DIAG 1
typedef struct { bool ol_enable; } MOTOR_PARAM_S;
volatile uint32_t g_motor_phase_active;
static struct { int skip; int divider; } s_divider;
static struct { uint32_t CR1; uint32_t CNT; } phase_timer;
#define TIM1 (&phase_timer)
static unsigned phase_notes;
static uint32_t phase_flags;
static void Motor_Timing_PhaseEdgeNote(uint32_t time, uint32_t count, uint32_t cr1,
                                       uint32_t divider, uint32_t flags, bool running)
{
    assert(time == 1234U && count == 200U && cr1 == 16U);
    assert(divider == 0U && running);
    phase_notes++;
    phase_flags = flags;
}
typedef unsigned motor_feedback_source_t;
static bool s_execute;
static unsigned s_runs, s_publishes, s_gates;
volatile uint8_t g_motor_timing_split_active;

static bool Motor_Timing_SplitIsActive(void)
{
    return g_motor_timing_split_active != 0U;
}
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
static unsigned s_status_marks;
static void Motor_Timing_PhaseMarkStatus(void) { s_status_marks++; }
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
        assert(s_status_frames == source / 300U && s_status_marks == s_status_frames);
    }
    g_motor_timing_split_active = 1U;
    s_execute = false;
    Motor_FocIsr_Step(&motor);
    s_execute = true;
    Motor_FocIsr_Step(&motor);
    assert(s_class_skip == 1U && s_class_full == 1U);
    for (source = 0U; source < 300U; source++) Motor_FocIsr_PublishStatusUnderTest(&motor);
    assert(s_status_frames == 4U && s_publish_notes == 1U);
    assert(s_status_marks == 4U);
    g_motor_phase_active = 1U;
    phase_timer.CR1 = 16U;
    phase_timer.CNT = 200U;
    s_divider.divider = 2;
    motor.ol_enable = true;
    s_execute = false;
    Motor_FocIsr_Step(&motor);
    assert(phase_notes == 1U && phase_flags == ((s_source << 8) | (2U << 16)));
    s_execute = true;
    Motor_FocIsr_Step(&motor);
    assert(phase_notes == 2U && phase_flags == ((s_source << 8) | (2U << 16) | 1U));
    puts("test_foc_dispatch: pass");
    return 0;
}
