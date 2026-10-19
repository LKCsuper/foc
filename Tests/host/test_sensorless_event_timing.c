#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "motor_sensorless.h"

typedef struct {
    double phase_square, angle_square, voltage_error_square;
    double flux_min, flux_max;
    unsigned missed_boundary;
} event_result_t;

static event_result_t run_events(bool variable_write, bool average_voltage, bool service_dt, unsigned cells)
{
    const double tau = 6.283185307179586, omega = 70.0 * tau;
    const double sample_dt = 1.0 / 15000.0, resistance = .646111, inductance = .000133819;
    const double flux = .0023358, cell_dt = sample_dt / cells;
    const double rotation_cos = cos(omega * cell_dt), rotation_sin = sin(omega * cell_dt);
    double rotor_cos = 1, rotor_sin = 0, current_alpha = 0, current_beta = .4;
    double active[2] = {0, 0}, pending[2] = {0, 0}, software[2] = {0, 0};
    double previous_service = 8.0e-6;
    MOTOR_PARAM_S motor;
    motor_sensorless_runtime_t runtime;
    motor_sensorless_effects_t effects;
    event_result_t result = {.flux_min = 100};
    memset(&motor, 0, sizeof(motor));
    Motor_Sensorless_InitState(&runtime);
    motor.ol_enable = motor.sensorless_enable = motor.sensorless_takeover = true;
    motor.ol_mode = MOTOR_OL_MODE_IF;
    motor.ol_pole_pairs = 7;
    motor.ol_current_rpm = motor.ol_target_rpm = 600;
    motor.ol_iq_set = .4f;
    motor.ol_max_rpm = 5920;
    motor.sensorless_motor_r = (float)resistance;
    motor.sensorless_motor_l = (float)inductance;
    motor.sensorless_flux_linkage = (float)flux;
    motor.sensorless_observer_gain = 12000000;
    motor.sensorless_pll_kp = 2000;
    motor.sensorless_pll_ki = 30000;
    motor.sensorless_speed_filter = .02f;
    motor.sensorless_transition_rpm = 500;
    motor.sensorless_phase_tolerance = 1.4f;
    motor.sensorless_lock_time = .1f;
    runtime.initialized = true;
    runtime.x_alpha = (float)flux;
    runtime.x_beta = (float)(inductance * current_beta);
    runtime.pll_speed_electrical = (float)omega;
    for (unsigned interval = 0; interval < 45000; ++interval) {
        double time = interval * sample_dt;
        double amplitude = .4 + .35 * sin(tau * 70 * time);
        double derivative = .35 * tau * 70 * cos(tau * 70 * time);
        double command[2] = {
            -resistance * amplitude * rotor_sin + inductance *
                (-derivative * rotor_sin - amplitude * omega * rotor_cos) - flux * omega * rotor_sin,
            resistance * amplitude * rotor_cos + inductance *
                (derivative * rotor_cos - amplitude * omega * rotor_sin) + flux * omega * rotor_cos};
        double latency = variable_write && interval % 2 ? 40.0e-6 : 8.0e-6;
        unsigned write_cell = (unsigned)ceil(latency / cell_dt);
        double integrated_voltage[2] = {0, 0};
        if (write_cell >= cells / 2 && interval >= 15000) ++result.missed_boundary;
        for (unsigned cell = 0; cell < cells; ++cell) {
            if (cell == 0 || cell == cells / 2) {
                active[0] = pending[0];
                active[1] = pending[1];
            }
            if (cell == write_cell) {
                pending[0] = software[0] = command[0];
                pending[1] = software[1] = command[1];
            }
            double next_cos = rotor_cos * rotation_cos - rotor_sin * rotation_sin;
            double next_sin = rotor_sin * rotation_cos + rotor_cos * rotation_sin;
            current_alpha += ((active[0] - resistance * current_alpha) * cell_dt -
                              flux * (next_cos - rotor_cos)) / inductance;
            current_beta += ((active[1] - resistance * current_beta) * cell_dt -
                             flux * (next_sin - rotor_sin)) / inductance;
            integrated_voltage[0] += active[0] / cells;
            integrated_voltage[1] += active[1] / cells;
            rotor_cos = next_cos;
            rotor_sin = next_sin;
        }
        double next_service = variable_write && (interval + 1) % 2 ? 16.0e-6 : 8.0e-6;
        double dt = sample_dt + (service_dt ? next_service - previous_service : 0);
        previous_service = next_service;
        motor.ia = (float)current_alpha;
        motor.ib = (float)(-.5 * current_alpha + .866025404 * current_beta);
        motor.ic = (float)(-.5 * current_alpha - .866025404 * current_beta);
        motor.ol_theta = (float)atan2(rotor_sin, rotor_cos);
        double input[2] = {average_voltage ? integrated_voltage[0] : software[0],
                           average_voltage ? integrated_voltage[1] : software[1]};
        float previous_pll = runtime.pll_angle;
        Motor_Sensorless_SetAppliedVoltageState(&runtime, (float)input[0], (float)input[1]);
        Motor_Sensorless_UpdateState(&runtime, &motor, (float)dt, &effects);
        assert(isfinite(runtime.flux_magnitude) && isfinite(runtime.pll_angle));
        if (interval >= 15000) {
            double phase = remainder(runtime.observer_angle - previous_pll, tau);
            double angle = remainder(runtime.pll_angle - atan2(rotor_sin, rotor_cos), tau);
            result.phase_square += phase * phase;
            result.angle_square += angle * angle;
            result.flux_min = fmin(result.flux_min, runtime.flux_magnitude / flux);
            result.flux_max = fmax(result.flux_max, runtime.flux_magnitude / flux);
            for (unsigned axis = 0; axis < 2; ++axis) {
                double error = software[axis] - integrated_voltage[axis];
                result.voltage_error_square += error * error;
            }
        }
    }
    return result;
}

int main(void)
{
    puts("variable_write,average_voltage,service_dt,phase_rms_rad,angle_rms_rad,flux_min_ratio,flux_max_ratio,software_voltage_error_rms_V,missed_boundary");
    for (unsigned variable = 0; variable < 2; ++variable)
        for (unsigned average = 0; average < 2; ++average)
            for (unsigned service = 0; service < 2; ++service) {
                event_result_t result = run_events(variable != 0, average != 0, service != 0, 120);
                event_result_t repeat = run_events(variable != 0, average != 0, service != 0, 120);
                assert(result.phase_square == repeat.phase_square);
                assert(result.missed_boundary == (variable ? 15000U : 0U));
                printf("%u,%u,%u,%.8f,%.8f,%.6f,%.6f,%.6f,%u\n", variable, average, service,
                       sqrt(result.phase_square / 30000), sqrt(result.angle_square / 30000),
                       result.flux_min, result.flux_max, sqrt(result.voltage_error_square / 30000),
                       result.missed_boundary);
            }
    event_result_t coarse = run_events(false, true, false, 120);
    event_result_t fine = run_events(false, true, false, 240);
    assert(fabs(sqrt(coarse.phase_square / 30000) - sqrt(fine.phase_square / 30000)) < .001);
    event_result_t constant_service = run_events(false, true, true, 120);
    assert(coarse.phase_square == constant_service.phase_square);
    return 0;
}
