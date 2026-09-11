#include <assert.h>
#include <stdio.h>

#include "motor_bldc_sensorless.h"

static void init_pair(motor_bldc_six_step_t *commutator,
                      motor_bldc_sensorless_t *sensorless,
                      motor_bldc_sensorless_config_t *sensorless_config)
{
    motor_bldc_six_step_config_t six_config;

    Motor_BldcSixStep_DefaultConfig(&six_config);
    assert(Motor_BldcSixStep_Init(commutator, &six_config));
    Motor_BldcSensorless_DefaultConfig(sensorless_config);
    sensorless_config->align_time_s = 0.001f;
    sensorless_config->startup_initial_period_s = 0.010f;
    sensorless_config->startup_final_period_s = 0.010f;
    sensorless_config->startup_ramp_time_s = 0.10f;
    sensorless_config->startup_timeout_s = 0.10f;
    sensorless_config->commutation_blanking_s = 0.0f;
    sensorless_config->zero_cross_threshold_v = 0.10f;
    sensorless_config->bemf_filter_alpha = 1.0f;
    sensorless_config->takeover_crossings = 1U;
    sensorless_config->closed_loop_timeout_s = 0.05f;
    assert(Motor_BldcSensorless_Init(sensorless, commutator, sensorless_config));
}

static void test_open_loop_to_closed_loop(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float neutral[3] = {0.0f, 0.0f, 0.0f};
    const float b_falling_side[3] = {1.0f, -1.0f, 0.0f};
    const float b_rising_side[3] = {0.0f, 1.0f, 0.0f};

    init_pair(&commutator, &sensorless, &config);
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.2f));
    assert(sensorless.step == 0U);
    assert(Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.001f));
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP);
    assert(sensorless.step == 1U);

    /* 正转 step1 的悬空 B 相应从负侧跨到正侧。 */
    assert(Motor_BldcSensorless_Step(&sensorless, b_falling_side, 24.0f, 0.001f));
    assert(Motor_BldcSensorless_Step(&sensorless, b_rising_side, 24.0f, 0.001f));
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP);
    assert(sensorless.commutation_scheduled);

    assert(Motor_BldcSensorless_Step(&sensorless, b_rising_side, 24.0f, 0.002f));
    assert(sensorless.step == 2U);
    assert(commutator.output.step == 2U);
}

static void test_closed_loop_lost_crossing(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float neutral[3] = {0.0f, 0.0f, 0.0f};
    const float negative_b[3] = {1.0f, -1.0f, 0.0f};
    const float positive_b[3] = {0.0f, 1.0f, 0.0f};

    init_pair(&commutator, &sensorless, &config);
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.2f));
    assert(Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.001f));
    assert(Motor_BldcSensorless_Step(&sensorless, negative_b, 24.0f, 0.001f));
    assert(Motor_BldcSensorless_Step(&sensorless, positive_b, 24.0f, 0.001f));
    assert(Motor_BldcSensorless_Step(&sensorless, positive_b, 24.0f, 0.002f));
    assert(!Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.05f));
    assert(sensorless.fault == MOTOR_BLDC_SENSORLESS_FAULT_ZERO_CROSS_LOST);
    assert(!Motor_BldcSixStep_IsRunning(&commutator));
}

static void test_invalid_sample_stops_bridge(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float phase[3] = {0.0f, 0.0f, 0.0f};

    init_pair(&commutator, &sensorless, &config);
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.2f));
    assert(!Motor_BldcSensorless_Step(&sensorless, phase, 0.0f, 0.001f));
    assert(sensorless.fault == MOTOR_BLDC_SENSORLESS_FAULT_INVALID_SAMPLE);
    assert(commutator.output.step == MOTOR_BLDC_SIX_STEP_INVALID);
}

int main(void)
{
    test_open_loop_to_closed_loop();
    test_closed_loop_lost_crossing();
    test_invalid_sample_stops_bridge();
    puts("motor_bldc_sensorless tests passed");
    return 0;
}
