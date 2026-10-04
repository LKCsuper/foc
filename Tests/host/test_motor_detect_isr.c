#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "motor_detect_isr.h"

static unsigned s_sample_calls;
static unsigned s_feed_calls;
static bool s_pwm_down;

void Motor_Sense_Update(MOTOR_PARAM_S *motor)
{
    assert(motor != NULL);
    s_sample_calls++;
}

void Motor_Detect_IsrFeed(bool pwm_down)
{
    assert(s_sample_calls == s_feed_calls + 1U);
    s_feed_calls++;
    s_pwm_down = pwm_down;
}

int main(void)
{
    MOTOR_PARAM_S motor;

    memset(&motor, 0, sizeof(motor));
    Motor_DetectIsr_Step(NULL, true);
    assert(s_sample_calls == 0U && s_feed_calls == 0U);

    Motor_DetectIsr_Step(&motor, false);
    assert(s_sample_calls == 1U && s_feed_calls == 1U && !s_pwm_down);
    Motor_DetectIsr_Step(&motor, true);
    assert(s_sample_calls == 2U && s_feed_calls == 2U && s_pwm_down);
    puts("test_motor_detect_isr: pass");
    return 0;
}
