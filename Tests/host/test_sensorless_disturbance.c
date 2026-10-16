#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_sensorless.h"
#include "motor_foc.h"

typedef struct {
    const char *name;
    float common_noise, differential_noise, quantization;
    int current_delay, voltage_delay;
    float resistance_ratio, inductance_ratio, voltage_bias, gain;
} scenario_t;

typedef struct {
    double angle_square;
    float angle_max, flux_min, flux_max, rpm_min, rpm_max;
    double phase_square, phase_sum, flux_sum, flux_square, flux_phase_sum;
    float phase_min, phase_max;
    unsigned sign_mismatches, sign_reversals, sign_zeros, sign_comparisons;
    double low_current_phase_square, high_current_phase_square;
    unsigned low_current_samples, high_current_samples;
} result_t;

static float noise(uint32_t *state)
{
    *state = *state * 1664525U + 1013904223U;
    return ((float)(*state >> 8) / 16777216.0f * 2.0f - 1.0f) * 1.732050808f;
}

static void deadtime_error(const float current[3], float amplitude, float *alpha, float *beta)
{
    float phase_error[3];
    for (unsigned phase = 0; phase < 3; ++phase)
        phase_error[phase] = current[phase] > 0.0f ? amplitude :
                             (current[phase] < 0.0f ? -amplitude : 0.0f);
    clarke_transform(phase_error[0], phase_error[1], phase_error[2], alpha, beta);
}

static result_t simulate_dynamic(const scenario_t *scenario, float voltage_ratio, float deadtime_voltage,
                                 float current_amplitude, float sign_compensation,
                                 float modulation_depth, float modulation_hz)
{
    MOTOR_PARAM_S motor;
    motor_sensorless_runtime_t runtime;
    motor_sensorless_effects_t effects;
    const float dt = 1.0f / 15000.0f;
    const float resistance = 0.646111f, inductance = 0.000133819f, flux = 0.0023358f;
    const double tau = 6.283185307179586, omega = 600.0 * 7.0 * tau / 60.0;
    float previous_alpha = 0.0f, previous_beta = current_amplitude;
    float previous_flux_alpha = flux, previous_flux_beta = 0.0f;
    float delayed_current[3] = {0.0f, 0.866025404f * current_amplitude, -0.866025404f * current_amplitude};
    float delayed_voltage[2] = {0.0f, 0.0f};
    uint32_t random_state = 12345U;
    result_t result = {.flux_min = 100.0f, .rpm_min = 1.0e9f, .rpm_max = -1.0e9f,
                       .phase_min = 1.0e9f, .phase_max = -1.0e9f};
    memset(&motor, 0, sizeof(motor));
    Motor_Sensorless_InitState(&runtime);
    motor.ol_enable = true;
    motor.ol_mode = MOTOR_OL_MODE_IF;
    motor.ol_pole_pairs = 7.0f;
    motor.ol_current_rpm = motor.ol_target_rpm = 600.0f;
    motor.ol_iq_set = current_amplitude;
    motor.ol_max_rpm = 5920.0f;
    motor.sensorless_enable = true;
    motor.sensorless_motor_r = resistance * scenario->resistance_ratio;
    motor.sensorless_motor_l = inductance * scenario->inductance_ratio;
    motor.sensorless_flux_linkage = flux;
    motor.sensorless_observer_gain = scenario->gain;
    motor.sensorless_pll_kp = 2000.0f;
    motor.sensorless_pll_ki = 30000.0f;
    motor.sensorless_speed_filter = 0.02f;
    motor.sensorless_transition_rpm = 500.0f;
    motor.sensorless_phase_tolerance = 1.4f;
    motor.sensorless_lock_time = 0.1f;
    runtime.initialized = true;
    runtime.x_alpha = flux;
    runtime.x_beta = motor.sensorless_motor_l * previous_beta;
    runtime.pll_speed_electrical = (float)omega;
    for (unsigned step = 1; step <= 45000; ++step) {
        float angle = (float)remainder(step * (double)dt * omega, tau);
        float amplitude = current_amplitude + modulation_depth *
                          (float)sin(step * (double)dt * tau * modulation_hz);
        float alpha = -amplitude * sinf(angle), beta = amplitude * cosf(angle);
        float flux_alpha = flux * cosf(angle), flux_beta = flux * sinf(angle);
        float voltage[2] = {
            resistance * alpha + inductance * (alpha - previous_alpha) / dt +
                (flux_alpha - previous_flux_alpha) / dt,
            resistance * beta + inductance * (beta - previous_beta) / dt +
                (flux_beta - previous_flux_beta) / dt};
        float current[3] = {alpha, -0.5f * alpha + 0.866025404f * beta,
                           -0.5f * alpha - 0.866025404f * beta};
        float measured[3];
        float deadtime_alpha, deadtime_beta;
        deadtime_error(current, deadtime_voltage, &deadtime_alpha, &deadtime_beta);
        float common = scenario->common_noise * noise(&random_state);
        for (unsigned phase = 0; phase < 3; ++phase) {
            measured[phase] = (scenario->current_delay ? delayed_current[phase] : current[phase]) +
                              common + scenario->differential_noise * noise(&random_state);
            if (scenario->quantization > 0.0f)
                measured[phase] = roundf(measured[phase] / scenario->quantization) * scenario->quantization;
            delayed_current[phase] = current[phase];
            if (step > 15000) {
                int true_sign = (current[phase] > 0) - (current[phase] < 0);
                int measured_sign = (measured[phase] > 0) - (measured[phase] < 0);
                result.sign_mismatches += true_sign != measured_sign;
                result.sign_reversals += true_sign * measured_sign < 0;
                result.sign_zeros += measured_sign == 0 && true_sign != 0;
                ++result.sign_comparisons;
            }
        }
        float true_comp_alpha, true_comp_beta, measured_comp_alpha, measured_comp_beta;
        deadtime_error(current, sign_compensation, &true_comp_alpha, &true_comp_beta);
        deadtime_error(measured, sign_compensation, &measured_comp_alpha, &measured_comp_beta);
        deadtime_alpha += true_comp_alpha - measured_comp_alpha;
        deadtime_beta += true_comp_beta - measured_comp_beta;
        motor.ia = measured[0];
        motor.ib = measured[1];
        motor.ic = measured[2];
        motor.ol_theta = angle;
        float previous_pll = runtime.pll_angle;
        Motor_Sensorless_SetAppliedVoltageState(&runtime,
            voltage_ratio * (scenario->voltage_delay ? delayed_voltage[0] : voltage[0]) +
                scenario->voltage_bias + deadtime_alpha,
            voltage_ratio * (scenario->voltage_delay ? delayed_voltage[1] : voltage[1]) + deadtime_beta);
        Motor_Sensorless_UpdateState(&runtime, &motor, dt, &effects);
        assert(isfinite(runtime.pll_angle) && isfinite(runtime.flux_magnitude));
        if (step > 15000) {
            float phase = remainderf(runtime.observer_angle - previous_pll, (float)tau);
            double flux_ratio = runtime.flux_magnitude / flux;
            result.phase_square += (double)phase * phase;
            if (amplitude < .25f) {
                result.low_current_phase_square += (double)phase * phase;
                ++result.low_current_samples;
            } else {
                result.high_current_phase_square += (double)phase * phase;
                ++result.high_current_samples;
            }
            result.phase_sum += phase;
            result.flux_sum += flux_ratio;
            result.flux_square += flux_ratio * flux_ratio;
            result.flux_phase_sum += flux_ratio * phase;
            result.phase_min = fminf(result.phase_min, phase);
            result.phase_max = fmaxf(result.phase_max, phase);
            float error = remainderf(runtime.pll_angle - angle, (float)tau);
            float rpm = remainderf(runtime.pll_angle - previous_pll, (float)tau) / dt * 60.0f /
                        (7.0f * (float)tau);
            result.angle_square += (double)error * error;
            result.angle_max = fmaxf(result.angle_max, fabsf(error));
            result.flux_min = fminf(result.flux_min, runtime.flux_magnitude / flux);
            result.flux_max = fmaxf(result.flux_max, runtime.flux_magnitude / flux);
            result.rpm_min = fminf(result.rpm_min, rpm);
            result.rpm_max = fmaxf(result.rpm_max, rpm);
        }
        memcpy(delayed_voltage, voltage, sizeof(voltage));
        previous_alpha = alpha;
        previous_beta = beta;
        previous_flux_alpha = flux_alpha;
        previous_flux_beta = flux_beta;
    }
    return result;
}

static result_t simulate_current(const scenario_t *scenario, float voltage_ratio, float deadtime_voltage,
                                 float current_amplitude, float sign_compensation)
{
    return simulate_dynamic(scenario, voltage_ratio, deadtime_voltage, current_amplitude,
                            sign_compensation, 0.0f, 0.0f);
}

static result_t simulate_voltage(const scenario_t *scenario, float voltage_ratio, float deadtime_voltage)
{
    return simulate_current(scenario, voltage_ratio, deadtime_voltage, .5f, 0.0f);
}

static result_t simulate(const scenario_t *scenario)
{
    return simulate_voltage(scenario, 1.0f, 0.0f);
}

static void print_result(const char *name, result_t result)
{
    double covariance = result.flux_phase_sum - result.flux_sum * result.phase_sum / 30000.0;
    double flux_variance = result.flux_square - result.flux_sum * result.flux_sum / 30000.0;
    double phase_variance = result.phase_square - result.phase_sum * result.phase_sum / 30000.0;
    double correlation = flux_variance > 1.0e-8 && phase_variance > 1.0e-8 ?
                         covariance / sqrt(flux_variance * phase_variance) : NAN;
    printf("%s,%.8f,%.8f,%.6f,%.6f,%.3f,%.3f,%.8f,%.8f,%.8f,%.6f,%.6f,%.6f,%.6f,%u,%u,%.8f,%.8f\n", name,
           sqrt(result.angle_square / 30000.0), result.angle_max, result.flux_min,
           result.flux_max, result.rpm_min, result.rpm_max,
           sqrt(result.phase_square / 30000.0), result.phase_min, result.phase_max, correlation,
           (double)result.sign_mismatches / result.sign_comparisons,
           (double)result.sign_reversals / result.sign_comparisons,
           (double)result.sign_zeros / result.sign_comparisons,
           result.low_current_samples, result.high_current_samples,
           result.low_current_samples ? sqrt(result.low_current_phase_square / result.low_current_samples) : NAN,
           result.high_current_samples ? sqrt(result.high_current_phase_square / result.high_current_samples) : NAN);
}

int main(void)
{
    const scenario_t scenarios[] = {
        {"baseline", 0, 0, 0, 0, 0, 1, 1, 0, 12000000},
        {"common_2counts_rms", .16117216f, 0, 0, 0, 0, 1, 1, 0, 12000000},
        {"independent_1count_rms", 0, .08058608f, 0, 0, 0, 1, 1, 0, 12000000},
        {"quantized", 0, 0, .08058608f, 0, 0, 1, 1, 0, 12000000},
        {"current_delay_1tick", 0, 0, 0, 1, 0, 1, 1, 0, 12000000},
        {"voltage_delay_1tick", 0, 0, 0, 0, 1, 1, 1, 0, 12000000},
        {"R_minus20pct", 0, 0, 0, 0, 0, .8f, 1, 0, 12000000},
        {"R_plus20pct", 0, 0, 0, 0, 0, 1.2f, 1, 0, 12000000},
        {"L_minus20pct", 0, 0, 0, 0, 0, 1, .8f, 0, 12000000},
        {"L_plus20pct", 0, 0, 0, 0, 0, 1, 1.2f, 0, 12000000},
        {"voltage_bias_30mV", 0, 0, 0, 0, 0, 1, 1, .03f, 12000000},
        {"noise_gain_3M", 0, .08058608f, 0, 0, 0, 1, 1, 0, 3000000},
        {"noise_gain_48M", 0, .08058608f, 0, 0, 0, 1, 1, 0, 48000000},
    };
    result_t baseline = simulate(&scenarios[0]);
    assert(baseline.angle_max < .08f);
    puts("scenario,angle_rms_rad,angle_max_rad,flux_min_ratio,flux_max_ratio,pll_advance_min_rpm,pll_advance_max_rpm,phase_rms_rad,phase_min_rad,phase_max_rad,flux_phase_correlation,sign_mismatch_fraction,sign_reversal_fraction,sign_zero_fraction,low_current_samples,high_current_samples,low_current_phase_rms_rad,high_current_phase_rms_rad");
    for (unsigned index = 0; index < sizeof(scenarios) / sizeof(scenarios[0]); ++index) {
        result_t result = simulate(&scenarios[index]);
        result_t repeat = simulate(&scenarios[index]);
        assert(result.angle_square == repeat.angle_square);
        if (index == 1) assert(fabs(result.angle_square - baseline.angle_square) < 1.0e-4);
        print_result(scenarios[index].name, result);
    }
    const float ratios[] = {.9f, .95f, .98f, 1.0f, 1.02f, 1.05f, 1.1f};
    const float residuals[] = {-.4f, -.2f, -.1f, -.05f, 0.0f, .05f, .1f, .2f, .4f};
    for (unsigned index = 0; index < sizeof(ratios) / sizeof(ratios[0]); ++index) {
        char name[64];
        snprintf(name, sizeof(name), "voltage_ratio_%.2f", ratios[index]);
        result_t result = simulate_voltage(&scenarios[0], ratios[index], 0.0f);
        result_t repeat = simulate_voltage(&scenarios[0], ratios[index], 0.0f);
        assert(result.angle_square == repeat.angle_square);
        if (ratios[index] == 1.0f) assert(result.angle_square == baseline.angle_square);
        print_result(name, result);
    }
    float positive_current[3] = {1.0f, -0.5f, -0.5f};
    float negative_current[3] = {-1.0f, 0.5f, 0.5f};
    float positive_alpha, positive_beta, negative_alpha, negative_beta;
    deadtime_error(positive_current, .1f, &positive_alpha, &positive_beta);
    deadtime_error(negative_current, .1f, &negative_alpha, &negative_beta);
    assert(fabsf(positive_alpha - .133333333f) < 1.0e-6f);
    assert(positive_alpha == -negative_alpha && positive_beta == -negative_beta);
    for (unsigned index = 0; index < sizeof(residuals) / sizeof(residuals[0]); ++index) {
        char name[64];
        snprintf(name, sizeof(name), "deadtime_residual_%+.2fV", residuals[index]);
        result_t result = simulate_voltage(&scenarios[0], 1.0f, residuals[index]);
        result_t repeat = simulate_voltage(&scenarios[0], 1.0f, residuals[index]);
        assert(result.angle_square == repeat.angle_square);
        if (residuals[index] == 0.0f) assert(result.angle_square == baseline.angle_square);
        print_result(name, result);
    }
    print_result("ratio_1.05_deadtime_+0.10V", simulate_voltage(&scenarios[0], 1.05f, .1f));
    print_result("ratio_0.95_deadtime_-0.10V", simulate_voltage(&scenarios[0], .95f, -.1f));
    const float amplitudes[] = {.05f, .1f, .2f, .3f, .5f, .75f, 1.0f};
    const char *modes[] = {"ideal", "quantized", "noise_quantized", "residual", "noise_residual", "noise_sign_comp"};
    for (unsigned amplitude = 0; amplitude < sizeof(amplitudes) / sizeof(amplitudes[0]); ++amplitude) {
        for (unsigned mode = 0; mode < sizeof(modes) / sizeof(modes[0]); ++mode) {
            scenario_t scenario = scenarios[0];
            bool noisy = mode == 2 || mode == 4 || mode == 5;
            scenario.quantization = mode == 1 || noisy ? .08058608f : 0.0f;
            scenario.differential_noise = noisy ? .08058608f : 0.0f;
            float residual = mode == 3 || mode == 4 ? -.1f : 0.0f;
            float compensation = mode == 5 ? .1f : 0.0f;
            result_t result = simulate_current(&scenario, 1.0f, residual, amplitudes[amplitude], compensation);
            result_t repeat = simulate_current(&scenario, 1.0f, residual, amplitudes[amplitude], compensation);
            assert(result.phase_square == repeat.phase_square);
            assert(result.sign_reversals + result.sign_zeros == result.sign_mismatches);
            if (mode == 1) assert(result.sign_reversals == 0);
            if (mode == 0) {
                assert(result.angle_max < .08f);
                assert(result.sign_mismatches == 0);
            }
            char name[80];
            snprintf(name, sizeof(name), "current_%.2fA_%s", amplitudes[amplitude], modes[mode]);
            print_result(name, result);
        }
    }
    result_t exact_compensation = simulate_current(&scenarios[0], 1, 0, .5f, .1f);
    assert(exact_compensation.phase_square == baseline.phase_square);
    result_t zero_modulation = simulate_dynamic(&scenarios[0], 1, 0, .5f, 0, 0, 70);
    assert(zero_modulation.phase_square == baseline.phase_square);
    const float frequencies[] = {5.0f, 70.0f, 500.0f};
    const char *delay_modes[] = {"aligned", "current_late", "voltage_late", "both_late", "noise_aligned", "noise_voltage_late"};
    for (unsigned frequency = 0; frequency < sizeof(frequencies) / sizeof(frequencies[0]); ++frequency) {
        for (unsigned mode = 0; mode < sizeof(delay_modes) / sizeof(delay_modes[0]); ++mode) {
            scenario_t scenario = scenarios[0];
            scenario.current_delay = mode == 1 || mode == 3;
            scenario.voltage_delay = mode == 2 || mode == 3 || mode == 5;
            if (mode >= 4) {
                scenario.quantization = .08058608f;
                scenario.differential_noise = .08058608f;
            }
            result_t result = simulate_dynamic(&scenario, 1, 0, .4f, 0, .35f, frequencies[frequency]);
            result_t repeat = simulate_dynamic(&scenario, 1, 0, .4f, 0, .35f, frequencies[frequency]);
            assert(result.phase_square == repeat.phase_square);
            assert(result.low_current_samples + result.high_current_samples == 30000);
            assert(result.low_current_samples > 0 && result.high_current_samples > 0);
            if (mode == 0) assert(result.angle_max < .08f);
            char name[96];
            snprintf(name, sizeof(name), "dynamic_%.0fHz_%s", frequencies[frequency], delay_modes[mode]);
            print_result(name, result);
        }
    }
    return 0;
}
