#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define COMMON_ASSERT(value) assert(value)
typedef struct { unsigned unused; } MOTOR_PARAM_S;
typedef unsigned motor_feedback_source_t;
static bool s_execute;
static unsigned s_runs, s_skips, s_publishes, s_gates;
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
static void Motor_Timing_RecordRuntime(uint32_t start) { assert(start == 1234U); s_skips++; }
static void Motor_FocIsr_PublishStatus(const MOTOR_PARAM_S *motor) { assert(motor != NULL); s_publishes++; }
#include "foc_dispatch_under_test.inc"

int main(void)
{
    MOTOR_PARAM_S motor = {0};
    unsigned source;
    for (source = 0U; source < 4U; source++) {
        s_source = source;
        s_execute = false;
        Motor_FocIsr_Step(&motor);
        assert(s_runs == source && s_skips == source + 1U);
        s_execute = true;
        Motor_FocIsr_Step(&motor);
        assert(s_runs == source + 1U && s_skips == source + 1U);
        assert(s_gates == 2U * (source + 1U) && s_publishes == s_gates);
    }
    puts("test_foc_dispatch: pass");
    return 0;
}
