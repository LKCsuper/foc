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
    static const uint8_t floating_phase[6] = {2U, 1U, 0U, 2U, 1U, 0U};
    float phase[3];
    uint8_t i;

    init_pair(&commutator, &sensorless, &config);
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    phase[0] = 12.0f;
    phase[1] = 12.0f;
    phase[2] = 12.0f;
    assert(Motor_BldcSensorless_Step(&sensorless, phase, 24.0f, 0.001f));
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP);
    for (i = 0U; i < 6U; i++) {
        phase[0] = 12.0f;
        phase[1] = 12.0f;
        phase[2] = 12.0f;
        phase[floating_phase[sensorless.step]] +=
            (sensorless.step & 1U) == 0U ? 1.0f : -1.0f;
        assert(Motor_BldcSensorless_Step(&sensorless, phase, 24.0f, 0.010f));
    }
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP);
    assert(sensorless.startup_sync_sectors == 6U);
    assert(sensorless.startup_valid_sectors == 6U);
    assert(sensorless.startup_invalid_sectors == 0U);
    assert(sensorless.startup_takeover_time_s > 0.0f);
    assert(sensorless.startup_takeover_period_s == 0.010f);
    for (i = 0U; i < 6U; i++) {
        assert(sensorless.step_startup_integral_vs[i] > 0.0f);
        assert(sensorless.step_startup_reverse_integral_vs[i] == 0.0f);
    }
    assert(sensorless.step == 1U);
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
    assert(commutator.duty == 0.08f);
}

static void test_automatic_start_uses_startup_duty_then_slews_to_target(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float neutral[3] = {12.0f, 12.0f, 12.0f};

    init_pair(&commutator, &sensorless, &config);
    config.startup_duty = 0.08f;
    config.duty_slew_rate_per_s = 0.05f;
    assert(Motor_BldcSensorless_Init(&sensorless, &commutator, &config));
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.12f));
    assert(commutator.duty == 0.08f);
    assert(sensorless.target_duty == 0.12f);

    sensorless.mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
    assert(Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.10f));
    assert(commutator.duty > 0.0849f && commutator.duty < 0.0851f);
}

static void test_fixed_open_loop_uses_requested_duty(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;

    init_pair(&commutator, &sensorless, &config);
    assert(Motor_BldcSensorless_StartOpenLoop(&sensorless, MOTOR_BLDC_DIRECTION_CW,
                                              0.12f, 0.010f));
    assert(commutator.duty == 0.12f);
    assert(sensorless.target_duty == 0.12f);
}

static void test_open_loop_rejects_unsynchronized_bemf(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float neutral[3] = {12.0f, 12.0f, 12.0f};
    uint8_t i;

    init_pair(&commutator, &sensorless, &config);
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    for (i = 0U; i < 20U && sensorless.mode != MOTOR_BLDC_SENSORLESS_FAULT; i++) {
        Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.010f);
        assert(sensorless.mode != MOTOR_BLDC_SENSORLESS_CLOSED_LOOP);
    }
    assert(sensorless.fault == MOTOR_BLDC_SENSORLESS_FAULT_STARTUP_TIMEOUT);
    assert(sensorless.startup_sync_sectors == 0U);
    assert(sensorless.startup_valid_sectors == 0U);
    assert(sensorless.startup_invalid_sectors > 0U);
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
    const float negative_c[3] = {12.0f, 12.0f, 11.0f};
    const float positive_c[3] = {12.0f, 12.0f, 13.0f};

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
    const float positive_c[3] = {12.0f, 12.0f, 13.0f};
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

static void test_bemf_integrator_blocks_rail_sample_in_first_half(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float rail_phase[3] = {0.0f, 0.0f, 24.0f};

    init_pair(&commutator, &sensorless, &config);
    config.bemf_integral_threshold_vs = 0.010f;
    assert(Motor_BldcSensorless_Init(&sensorless, &commutator, &config));
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    sensorless.mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
    sensorless.filtered_commutation_period_s = 0.010f;

    assert(Motor_BldcSensorless_Step(&sensorless, rail_phase, 24.0f, 0.001f));
    assert(sensorless.bemf_integral_vs == 0.0f);
    assert(sensorless.phase_blocked_samples == 1U);
    assert(sensorless.phase_late_samples == 0U);

    assert(Motor_BldcSensorless_Step(&sensorless, rail_phase, 24.0f, 0.008f));
    assert(sensorless.phase_late_samples == 1U);
    assert(sensorless.integral_commutations == 1U);
}

static void test_duty_period_selects_startup_capture_period(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float neutral[3] = {12.0f, 12.0f, 12.0f};

    init_pair(&commutator, &sensorless, &config);
    config.duty_period_enabled = true;
    config.duty_period_low_duty = 0.08f;
    config.duty_period_high_duty = 0.10f;
    config.duty_period_low_s = 0.012f;
    config.duty_period_high_s = 0.011f;
    config.startup_ramp_time_s = 0.001f;
    assert(Motor_BldcSensorless_Init(&sensorless, &commutator, &config));
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.09f));
    commutator.duty = 0.09f;

    assert(Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.001f));
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP);
    assert(Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.001f));
    assert(sensorless.filtered_commutation_period_s > 0.01149f);
    assert(sensorless.filtered_commutation_period_s < 0.01151f);
}

static void test_closed_loop_detects_per_step_integral_thresholds(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    static const uint8_t floating_phase[6] = {2U, 1U, 0U, 2U, 1U, 0U};
    float phase[3];
    uint8_t i;

    init_pair(&commutator, &sensorless, &config);
    config.integral_auto_detect_enabled = true;
    config.integral_detect_scale = 0.80f;
    config.bemf_integral_threshold_vs = 0.001f;
    config.duty_period_enabled = true;
    config.duty_period_low_s = 0.010f;
    config.duty_period_high_s = 0.010f;
    assert(Motor_BldcSensorless_Init(&sensorless, &commutator, &config));
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    sensorless.mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
    sensorless.filtered_commutation_period_s = 0.010f;

    for (i = 0U; i < 84U; i++) {
        phase[0] = 12.0f;
        phase[1] = 12.0f;
        phase[2] = 12.0f;
        phase[floating_phase[sensorless.step]] +=
            (sensorless.step & 1U) == 0U ? 1.0f : -1.0f;
        assert(Motor_BldcSensorless_Step(&sensorless, phase, 24.0f, 0.010f));
    }
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP);
    assert(sensorless.detected_integral_mask == 0x3FU);
    assert(sensorless.integral_detect_settle_sectors == 12U);
    for (i = 0U; i < 6U; i++) {
        assert(sensorless.step_integral_detect_samples[i] == 12U);
        assert(sensorless.step_detected_integral_vs[i] > 0.0045f);
        assert(sensorless.step_detected_integral_vs[i] < 0.0055f);
    }
}

static void test_detected_threshold_replaces_duty_period_floor(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float positive_c[3] = {12.0f, 12.0f, 13.0f};
    uint8_t i;

    init_pair(&commutator, &sensorless, &config);
    config.integral_auto_detect_enabled = true;
    config.duty_period_enabled = true;
    config.closed_loop_min_period_s = 0.001f;
    config.bemf_integral_threshold_vs = 0.001f;
    assert(Motor_BldcSensorless_Init(&sensorless, &commutator, &config));
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    sensorless.mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
    sensorless.filtered_commutation_period_s = 0.010f;
    sensorless.detected_integral_mask = 0x3FU;
    sensorless.step_detected_integral_vs[0] = 0.005f;

    for (i = 0U; i < 9U; i++) {
        assert(Motor_BldcSensorless_Step(&sensorless, positive_c, 24.0f, 0.001f));
    }
    assert(sensorless.step == 1U);
    assert(sensorless.active_minimum_period_s == 0.001f);
    assert(sensorless.minimum_commutation_period_s > 0.0089f);
    assert(sensorless.minimum_commutation_period_s < 0.0091f);
}

static void test_startup_preserves_integral_threshold(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    static const uint8_t floating_phase[6] = {2U, 1U, 0U, 2U, 1U, 0U};
    float phase[3];
    uint8_t i;

    init_pair(&commutator, &sensorless, &config);
    config.startup_ramp_time_s = 0.030f;
    config.bemf_integral_threshold_vs = 0.009f;
    assert(Motor_BldcSensorless_Init(&sensorless, &commutator, &config));
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    phase[0] = 12.0f;
    phase[1] = 12.0f;
    phase[2] = 12.0f;
    phase[floating_phase[sensorless.step]] +=
        (sensorless.step & 1U) == 0U ? 2.0f : -2.0f;
    assert(Motor_BldcSensorless_Step(&sensorless, phase, 24.0f, 0.001f));

    for (i = 0U; i < 100U && sensorless.mode != MOTOR_BLDC_SENSORLESS_CLOSED_LOOP; i++) {
        phase[0] = 12.0f;
        phase[1] = 12.0f;
        phase[2] = 12.0f;
        phase[floating_phase[sensorless.step]] +=
            (sensorless.step & 1U) == 0U ? 2.0f : -2.0f;
        assert(Motor_BldcSensorless_Step(&sensorless, phase, 24.0f, 0.001f));
    }
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP);
    assert(sensorless.config.bemf_integral_threshold_vs == 0.009f);
}

static void test_closed_loop_forces_late_step_without_corrupting_period(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float positive_c[3] = {12.0f, 12.0f, 13.0f};
    uint8_t i;

    init_pair(&commutator, &sensorless, &config);
    config.bemf_integral_threshold_vs = 0.0001f;
    config.period_change_limit = 0.10f;
    config.period_filter_alpha = 0.25f;
    config.duty_period_enabled = true;
    assert(Motor_BldcSensorless_Init(&sensorless, &commutator, &config));
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    sensorless.mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
    sensorless.filtered_commutation_period_s = 0.010f;
    sensorless.config.bemf_integral_threshold_vs = 0.100f;

    for (i = 0U; i < 11U; i++) {
        assert(Motor_BldcSensorless_Step(&sensorless, positive_c, 24.0f, 0.001f));
    }
    assert(sensorless.step == 1U);
    assert(sensorless.forced_commutations == 1U);
    assert(sensorless.config.bemf_integral_threshold_vs == 0.100f);
    assert(sensorless.filtered_commutation_period_s > 0.0099f);
    assert(sensorless.filtered_commutation_period_s < 0.0101f);
}

static void test_closed_loop_trips_after_one_forced_electrical_cycle(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float neutral[3] = {12.0f, 12.0f, 12.0f};
    uint8_t i;

    init_pair(&commutator, &sensorless, &config);
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    sensorless.mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
    sensorless.filtered_commutation_period_s = 0.010f;

    for (i = 0U; i < 5U; i++) {
        assert(Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.013f));
        assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP);
    }
    assert(!Motor_BldcSensorless_Step(&sensorless, neutral, 24.0f, 0.013f));
    assert(sensorless.mode == MOTOR_BLDC_SENSORLESS_FAULT);
    assert(sensorless.fault == MOTOR_BLDC_SENSORLESS_FAULT_BEMF_LOST);
    assert(sensorless.forced_commutations == 6U);
    assert(sensorless.consecutive_forced_commutations == 6U);
    assert(!Motor_BldcSixStep_IsRunning(&commutator));
}

static void test_passive_zero_candidate_is_not_blocked_by_integral_gate(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;
    motor_bldc_sensorless_config_t config;
    const float rail_sample[3] = {0.0f, 0.0f, 24.0f};
    const float near_zero_sample[3] = {12.0f, 12.0f, 12.03f};

    init_pair(&commutator, &sensorless, &config);
    config.commutation_blanking_s = 0.0f;
    config.bemf_integral_threshold_vs = 1.0f;
    config.duty_period_enabled = false;
    assert(Motor_BldcSensorless_Init(&sensorless, &commutator, &config));
    assert(Motor_BldcSensorless_Start(&sensorless, MOTOR_BLDC_DIRECTION_CW, 0.08f));
    sensorless.mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
    sensorless.filtered_commutation_period_s = 0.010f;

    assert(Motor_BldcSensorless_Step(&sensorless, rail_sample, 24.0f, 0.001f));
    assert(sensorless.phase_blocked_samples == 1U);
    assert(Motor_BldcSensorless_Step(&sensorless, near_zero_sample, 24.0f, 0.012f));

    assert(sensorless.step == 1U);
    assert(sensorless.step_candidate_min_time_s[0][2] > 0.0129f);
    assert(sensorless.step_candidate_min_time_s[0][2] < 0.0131f);
    assert(sensorless.step_candidate_min_abs_v[0][2] > 0.019f);
    assert(sensorless.step_candidate_min_abs_v[0][2] < 0.021f);
    assert(sensorless.step_bus_candidate_min_time_s[0][2] > 0.0129f);
    assert(sensorless.step_bus_candidate_min_time_s[0][2] < 0.0131f);
    assert(sensorless.step_bus_candidate_min_abs_v[0][2] > 0.029f);
    assert(sensorless.step_bus_candidate_min_abs_v[0][2] < 0.031f);
    assert(sensorless.step_floating_waveform_v[0][7] > 12.029f);
    assert(sensorless.step_floating_waveform_v[0][7] < 12.031f);
    assert(sensorless.step_neutral_waveform_v[0][7] > 12.009f);
    assert(sensorless.step_neutral_waveform_v[0][7] < 12.011f);
}

int main(void)
{
    test_open_loop_to_integral_closed_loop();
    test_fixed_open_loop_ignores_bemf();
    test_automatic_start_uses_startup_duty_then_slews_to_target();
    test_fixed_open_loop_uses_requested_duty();
    test_open_loop_rejects_unsynchronized_bemf();
    test_invalid_sample_stops_bridge();
    test_bemf_integrator_commutates();
    test_bemf_integrator_limits_period_reduction();
    test_bemf_integrator_blocks_rail_sample_in_first_half();
    test_duty_period_selects_startup_capture_period();
    test_closed_loop_detects_per_step_integral_thresholds();
    test_detected_threshold_replaces_duty_period_floor();
    test_startup_preserves_integral_threshold();
    test_closed_loop_forces_late_step_without_corrupting_period();
    test_closed_loop_trips_after_one_forced_electrical_cycle();
    test_passive_zero_candidate_is_not_blocked_by_integral_gate();
    puts("motor_bldc_sensorless tests passed");
    return 0;
}
