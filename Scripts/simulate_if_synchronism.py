"""Sampled electrical/mechanical I/F sensitivity model, with unmeasured J/load ranges."""
import argparse
import json
import math
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

from simulate_current_loop import Config as ElectricalConfig, phase_peak, pi_step


@dataclass(frozen=True)
class Config:
    rpm: float = 3500
    inertia: float = 2e-5
    friction: float = 2e-6
    equilibrium_angle: float = 1.3
    sample_delay: int = 0
    output_delay: int = 0
    voltage_limit: float = 8.5
    substeps: int = 8
    stationary_hold: bool = True
    output_fraction: float = 0.0


def equilibrium(config):
    if not all(math.isfinite(value) for value in
               (config.rpm, config.inertia, config.friction, config.equilibrium_angle, config.voltage_limit,
                config.output_fraction)):
        raise ValueError('Nonfinite model parameter')
    if config.inertia <= 0 or config.friction < 0 or config.rpm <= 0 or not 0 < config.equilibrium_angle < math.pi / 2:
        raise ValueError('Invalid mechanical equilibrium')
    if config.sample_delay not in (0, 1) or config.output_delay not in (0, 1) or not 1 <= config.substeps <= 32:
        raise ValueError('Unsupported delay/substep selection')
    if not 0 <= config.output_fraction < 1:
        raise ValueError('Fractional output delay must be in [0, 1)')
    electric = ElectricalConfig(rpm=config.rpm, voltage_limit=config.voltage_limit)
    mechanical_speed = config.rpm * math.tau / 60
    electrical_speed = mechanical_speed * electric.pole_pairs
    torque_constant = 1.5 * electric.pole_pairs * electric.flux
    load = torque_constant * math.cos(config.equilibrium_angle) - config.friction * mechanical_speed
    if load < 0:
        raise ValueError('Selected equilibrium requires a driving load')
    emf = electrical_speed * electric.flux
    voltage = (-emf * math.sin(config.equilibrium_angle) - electrical_speed * electric.inductance,
               emf * math.cos(config.equilibrium_angle) + electric.resistance)
    if math.hypot(*voltage) >= config.voltage_limit:
        raise ValueError('Equilibrium exceeds voltage headroom')
    if config.stationary_hold:
        lead = electrical_speed / electric.frequency * (config.output_delay + config.output_fraction + .5)
        voltage = (voltage[0] * math.cos(lead) - voltage[1] * math.sin(lead),
                   voltage[0] * math.sin(lead) + voltage[1] * math.cos(lead))
    state = (0.0, 1.0, config.equilibrium_angle, mechanical_speed, *voltage)
    if config.sample_delay:
        state += (0.0, 1.0)
    for _ in range(config.output_delay + int(config.output_fraction > 0)):
        state += voltage
    if config.stationary_hold or config.output_fraction:
        for _ in range(15):
            residual = np.asarray(advance(state, config, electric, load)) - state
            if max(abs(residual)) < 1e-10:
                break
            columns = []
            for index, value in enumerate(state):
                step = 1e-4 if index == 3 else 1e-6
                plus, minus = list(state), list(state)
                plus[index], minus[index] = value + step, value - step
                columns.append((np.asarray(advance(tuple(plus), config, electric, load)) -
                                np.asarray(advance(tuple(minus), config, electric, load))) / (2 * step))
            correction = np.linalg.solve(np.asarray(columns).T - np.eye(len(state)), -residual)
            state = tuple(np.asarray(state) + correction)
        else:
            raise ArithmeticError('Could not solve sampled equilibrium')
    return state, electric, load


def advance(state, config, electric, load):
    dt = 1 / electric.frequency
    forced_speed = config.rpm * math.tau / 60 * electric.pole_pairs
    torque_constant = 1.5 * electric.pole_pairs * electric.flux
    feedback = state[6:8] if config.sample_delay else state[:2]
    integral, requested = pi_step(state[4:6], feedback, electric, dt)
    queue_start = 6 + 2 * config.sample_delay
    voltage = state[queue_start:queue_start + 2] if config.output_delay else requested

    def derivative(plant, offset, applied_voltage, age_cycles):
        direct, quadrature, relative_angle, speed = plant
        rotor_speed = speed * electric.pole_pairs
        sine, cosine = math.sin(relative_angle), math.cos(relative_angle)
        emf_direct = -rotor_speed * electric.flux * sine
        emf_quadrature = rotor_speed * electric.flux * cosine
        torque = torque_constant * (-direct * sine + quadrature * cosine)
        rotation = -forced_speed * (age_cycles * dt + offset) if config.stationary_hold else 0.0
        applied_direct = applied_voltage[0] * math.cos(rotation) - applied_voltage[1] * math.sin(rotation)
        applied_quadrature = applied_voltage[0] * math.sin(rotation) + applied_voltage[1] * math.cos(rotation)
        return ((applied_direct - electric.resistance * direct + forced_speed * electric.inductance * quadrature - emf_direct) / electric.inductance,
                (applied_quadrature - electric.resistance * quadrature - forced_speed * electric.inductance * direct - emf_quadrature) / electric.inductance,
                rotor_speed - forced_speed,
                (torque - load - config.friction * speed) / config.inertia)

    def integrate(plant, begin, end, applied_voltage, age_cycles):
        steps = max(1, math.ceil(config.substeps * (end - begin) / dt))
        step = (end - begin) / steps
        for substep in range(steps):
            offset = begin + substep * step
            first = derivative(plant, offset, applied_voltage, age_cycles)
            second = derivative(tuple(value + step * slope / 2 for value, slope in zip(plant, first)),
                                offset + step / 2, applied_voltage, age_cycles)
            third = derivative(tuple(value + step * slope / 2 for value, slope in zip(plant, second)),
                               offset + step / 2, applied_voltage, age_cycles)
            fourth = derivative(tuple(value + step * slope for value, slope in zip(plant, third)),
                                offset + step, applied_voltage, age_cycles)
            plant = tuple(value + step * (first_slope + 2 * second_slope + 2 * third_slope + fourth_slope) / 6
                          for value, first_slope, second_slope, third_slope, fourth_slope
                          in zip(plant, first, second, third, fourth))
        return plant

    switch = config.output_fraction * dt
    plant = state[:4]
    if config.output_fraction:
        plant = integrate(plant, 0.0, switch, state[-2:], config.output_delay + 1)
    plant = integrate(plant, switch, dt, voltage, config.output_delay)
    result = (*plant, *integral)
    if config.sample_delay:
        result += state[:2]
    if config.output_delay or config.output_fraction:
        result += requested
    if config.output_delay and config.output_fraction:
        result += state[queue_start:queue_start + 2]
    return result


def stability(config, difference_scale=1.0):
    state, electric, load = equilibrium(config)
    columns = []
    for index, value in enumerate(state):
        step = difference_scale * (1e-4 if index == 3 else 1e-6)
        plus, minus = list(state), list(state)
        plus[index], minus[index] = value + step, value - step
        columns.append((np.asarray(advance(tuple(plus), config, electric, load)) -
                        np.asarray(advance(tuple(minus), config, electric, load))) / (2 * step))
    eigenvalues = np.linalg.eigvals(np.asarray(columns).T)
    dominant = max(eigenvalues, key=abs)
    return {'config': asdict(config), 'load_nm': load,
            'sampled_equilibrium_angle_rad': state[2],
            'dominant_magnitude': float(abs(dominant)),
            'growth_per_s': float(math.log(abs(dominant)) * electric.frequency),
            'oscillation_hz': float(abs(np.angle(dominant)) * electric.frequency / math.tau)}


def trajectory(config, duration=.5, angle_perturbation=.005, modal_perturbation=False):
    if not math.isfinite(duration) or duration <= 0 or not math.isfinite(angle_perturbation):
        raise ValueError('Invalid trajectory duration or perturbation')
    state, electric, load = equilibrium(config)
    reference_angle = state[2]
    initial_state = state
    if modal_perturbation and angle_perturbation:
        columns = []
        for index, value in enumerate(state):
            step = 1e-4 if index == 3 else 1e-6
            plus, minus = list(state), list(state)
            plus[index], minus[index] = value + step, value - step
            columns.append((np.asarray(advance(tuple(plus), config, electric, load)) -
                            np.asarray(advance(tuple(minus), config, electric, load))) / (2 * step))
        eigenvalues, eigenvectors = np.linalg.eig(np.asarray(columns).T)
        vector = eigenvectors[:, np.argmax(abs(eigenvalues))]
        vector = vector * np.exp(-1j * np.angle(vector[2]))
        if abs(vector[2]) < 1e-12:
            raise ArithmeticError('Dominant mode has no angle component')
        state = tuple(np.asarray(state) + vector.real * angle_perturbation / vector[2].real)
    else:
        state = (*state[:2], state[2] + angle_perturbation, *state[3:])
    initial_current_deviation = math.hypot(state[0] - initial_state[0], state[1] - initial_state[1])
    maximum_current = 0.0
    maximum_angle = abs(angle_perturbation)
    timer = 0.0
    fault_ms = None
    departure_ms = None
    stop_reason = 'duration'
    for index in range(round(duration * electric.frequency)):
        state = advance(state, config, electric, load)
        if not all(math.isfinite(value) for value in state):
            raise ArithmeticError('Nonfinite trajectory')
        time = (index + 1) / electric.frequency
        peak = phase_peak(state[:2], config.rpm * math.tau / 60 * electric.pole_pairs * time)
        maximum_current = max(maximum_current, peak)
        maximum_angle = max(maximum_angle, abs(state[2] - reference_angle))
        timer = timer + 1 / electric.frequency if peak >= 10 else 0.0
        if timer >= .001 - 1e-12:
            fault_ms = time * 1000
            stop_reason = 'simulated_overcurrent'
            break
        if abs(state[2] - reference_angle) >= math.pi / 2:
            departure_ms = time * 1000
            stop_reason = 'left_local_angle_range_not_a_hardware_fault'
            break
    return {'config': asdict(config), 'initial_angle_perturbation_rad': angle_perturbation,
            'modal_perturbation': modal_perturbation, 'initial_current_deviation_a': initial_current_deviation,
            'duration_ms': time * 1000, 'peak_phase_a': maximum_current,
            'max_angle_deviation_rad': maximum_angle, 'final_angle_deviation_rad': state[2] - reference_angle,
            'final_rpm': state[3] * 60 / math.tau, 'overcurrent_ms': fault_ms,
            'left_local_angle_range_ms': departure_ms, 'stop_reason': stop_reason}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--boundary-sweep', action='store_true')
    args = parser.parse_args()
    results = []
    if args.boundary_sweep:
        for rpm, limit in ((300, 4), (3000, 8), (3500, 8.5), (5000, 10)):
            for inertia in (1e-6, 5e-6, 2e-5, 1e-4):
                for friction in (0.0, 2e-6):
                    for angle in (1.0, 1.3, 1.5):
                        for sample_delay in (0, 1):
                            for output_delay, fraction in ((0, 0.0), (0, .45), (0, .5), (0, .55), (1, 0.0)):
                                results.append(stability(Config(rpm, inertia, friction, angle, sample_delay,
                                                                output_delay, limit, output_fraction=fraction)))
        output = {
            'scope': 'Boundary timing sensitivity, not a measured failure-speed prediction; J/load unmeasured',
            'timing_basis': '300RPM write finishes 9.92-10.71us before next half-cycle UEV. '
                            'The nominal sample-to-load delay is about half a 66.67us control cycle, '
                            'not the measured write-to-boundary margin. High-speed timing is unmeasured.',
            'electrical_parameters': asdict(ElectricalConfig()), 'linearized_cases': results}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(output, indent=2), encoding='utf-8')
        for rpm in (300, 3000, 3500, 5000):
            for delay in (0.0, .45, .5, .55, 1.0):
                group = [result for result in results if result['config']['rpm'] == rpm and
                         result['config']['output_delay'] + result['config']['output_fraction'] == delay]
                print('RPM', rpm, 'output cycles', delay, 'cases', len(group),
                      'positive', sum(result['growth_per_s'] > .01 for result in group),
                      'growth range', min(result['growth_per_s'] for result in group),
                      max(result['growth_per_s'] for result in group))
        return
    for rpm, limit in ((3000, 8), (3500, 8.5), (5000, 10)):
        for inertia in (1e-6, 5e-6, 2e-5, 1e-4):
            for friction in (0.0, 2e-6):
                for angle in (1.0, 1.3, 1.5):
                    for sample_delay, output_delay in ((0, 0), (1, 0), (0, 1), (1, 1)):
                        for stationary_hold in (False, True):
                            results.append(stability(Config(rpm, inertia, friction, angle, sample_delay, output_delay,
                                                            limit, stationary_hold=stationary_hold)))
    fastest = max(results, key=lambda result: result['growth_per_s'])
    slowest = min(results, key=lambda result: result['growth_per_s'])
    selected = max((result for result in results if result['config']['rpm'] == 3500),
                   key=lambda result: result['growth_per_s'])
    selected_config = Config(**selected['config'])
    traces = [trajectory(config, duration=2.0, modal_perturbation=True) for config in
              (selected_config, replace(selected_config, stationary_hold=False))]
    output = {'scope': 'Hypothetical J/load sensitivity; no measured mechanical parameters, not a fitted failure-speed prediction',
              'electrical_parameters': asdict(ElectricalConfig()), 'linearized_cases': results,
              'trajectories_selected_as': ['largest growth at 3500RPM', 'same configuration with ideal rotating dq hold'],
              'trajectories': traces}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2), encoding='utf-8')
    print('Cases:', len(results), 'positive growth:', sum(result['growth_per_s'] > .01 for result in results))
    print('Largest growth:', fastest)
    print('Smallest growth:', slowest)
    print('Small perturbation trajectories:', traces)


if __name__ == '__main__':
    main()
