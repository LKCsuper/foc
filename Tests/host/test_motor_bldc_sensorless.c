#include <assert.h>
#include <stdio.h>

#include "motor_bldc_sensorless.h"

static void init_pair(motor_bldc_six_step_t *commutator,
                      motor_bldc_sensorless_t *sensorless,
                      motor_bldc_sensorless_config_t *config)
{
    motor_bldc_six_step_config_t six_config;

    Motor_BldcSixStep_DefaultConfig(&six_config);
    assert(Motor_BldcSixStep_Init(commutator, &six_config));
    Motor_BldcSensorless_DefaultConfig(config);
    config->align_time_s = 0.001f;
    config->startup_initial_period_s = 0.010f;
    config->startup_final_period_s = 0.010f;
    config->startup_ramp_time_s = 0.010f;
    config->startup_timeout_s = 0.10f;
    config->commutation_blanking_s = 0.0f;
    config->bemf_filter_alpha = 1.0f;
    config->step0_rising = true;
    config->closed_loop_min_period_s = 0.001f;
    config->closed_loop_timeout_s = 0.05f;
    assert(Motor_BldcSensorless_Init(sensorless, commutator, config));
}

static void test_open_loop_to_integral_closed_loop(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float neutral[3] = {0.0f, 0.0f, 0.0f};

    init_pair(&commutator, &sensorless, &config);
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    assert(Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.001f));
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP);
    assert(Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.010f));
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP);
    assert(sensorless.step == 2U);
    assert(sensorless.filtered_commutation_period_s == 0.010f);
}

static void test_fixed_open_loop_ignores_bemf(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float waveform[3] = {1.0f, -1.0f, 0.0f};
    float startup_final_period_s;

    init_pair(&commutator, &sensorless, &config);
    sensorless.config.startup_ramp_time_s = 0.001f;
    startup_final_period_s = sensorless.config.startup_final_period_s;
    assert(Motor_BldcSensorless_StartOpenLoop(&sensorless, MOTOR_BLDC_DIRECTION_CW,
                                              0.08f, 0.003f));
    assert(Motor_BldcSensorless_Step(&sensorless, waveform, 24.0f, 0.001f));
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP_FIXED);
    assert(Motor_BldcSensorless_Step(&sensorless, waveform, 24.0f, 0.003f));
    assert(sensorless.step == 2U);
    assert(commutator.output.step == 2U);
    assert(sensorless.config.startup_final_period_s == startup_final_period_s);
}

static void test_invalid_sample_stops_bridge(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float phase[3] = {0.0f, 0.0f, 0.0f};

    init_pair(&commutator, &sensorless, &config);
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    assert(!Motor_BldcSensorless_Step(&sensorless, phase, 0.0f, 0.001f));
    assert(sensorless.fault == MOTOR_BLDC_SENSORLESS_FAULT_INVALID_SAMPLE);
    assert(commutator.output.step == MOTOR_BLDC_SIX_STEP_INVALID);
}

static void test_bemf_integrator_commutates(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float negative_c[3] = {0.0f, 0.0f, -1.0f};
    const float positive_c[3] = {0.0f, 0.0f, 1.0f};

    init_pair(&commutator, &sensorless, &config);
    config.bemf_integral_threshold_vs = 0.0010f;
    config.bemf_integral_noise_v = 0.05f;
    assert(Motor_BldcSensorless_Init(&sensorless, &commutator, &config));
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    sensorless.mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;

    assert(Motor_BldcSensorless_Step(&sensorless, negative_c, 24.0f, 0.001f));
    assert(sensorless.bemf_integral_vs == 0.0f);
    assert(Motor_BldcSensorless_Step(&sensorless, positive_c, 24.0f, 0.001f));
    assert(sensorless.step == 0U);
    assert(Motor_BldcSensorless_Step(&sensorless, positive_c, 24.0f, 0.001f));
    assert(sensorless.step == 1U);
    assert(sensorless.integral_commutations == 1U);
}

static void test_bemf_integrator_limits_period_reduction(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float positive_c[3] = {0.0f, 0.0f, 1.0f};
    uint8_t i;

    init_pair(&commutator, &sensorless, &config);
    config.bemf_integral_threshold_vs = 0.0001f;
    config.period_change_limit = 0.10f;
    config.period_filter_alpha = 0.25f;
    assert(Motor_BldcSensorless_Init(&sensorless, &commutator, &config));
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    sensorless.mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
    sensorless.filtered_commutation_period_s = 0.020f;

    for (i = 0U; i < 17U; i++) {
        assert(Motor_BldcSensorless_Step(&sensorless, positive_c, 24.0f, 0.001f));
        assert(sensorless.step == 0U);
    }
    assert(Motor_BldcSensorless_Step(&sensorless, positive_c, 24.0f, 0.001f));
    assert(sensorless.step == 1U);
    assert(sensorless.minimum_commutation_period_s > 0.0179f);
    assert(sensorless.filtered_commutation_period_s > 0.0194f);
    assert(sensorless.filtered_commutation_period_s < 0.0196f);
}

int main(void)
{
    test_open_loop_to_integral_closed_loop();
    test_fixed_open_loop_ignores_bemf();
    test_invalid_sample_stops_bridge();
    test_bemf_integrator_commutates();
    test_bemf_integrator_limits_period_reduction();
    puts("motor_bldc_sensorless tests passed");
    return 0;
}
