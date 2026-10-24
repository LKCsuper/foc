"""Electrical-only dq sensitivity study; not a mechanical loss-of-sync reconstruction."""
import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Config:
    resistance: float = 0.474
    inductance: float = 0.000112
    flux: float = 0.002390
    pole_pairs: int = 7
    rpm: float = 3500
    kp: float = 0.12
    ki: float = 18
    frequency: float = 15000
    voltage_limit: float = 8.5
    duration: float = 0.16
    disturbance_start: float = 0.02
    disturbance_duration: float = 0.02
    angle_change: float = 0.0
    initial_id: float = 0.0
    sample_delay: int = 0
    output_delay: int = 0
    coupling: bool = True
    substeps: int = 8


def pi_step(integrals, currents, config, dt):
    errors = (-currents[0], 1.0 - currents[1])
    outputs = []
    updated = []
    for integral, error in zip(integrals, errors):
        integral += error * config.ki * dt
        output = integral + error * config.kp
        limited = max(-config.voltage_limit, min(config.voltage_limit, output))
        if limited != output:
            integral = limited - error * config.kp
        outputs.append(limited)
        updated.append(integral)
    norm = math.hypot(*outputs)
    if norm > config.voltage_limit:
        outputs = [value * config.voltage_limit / norm for value in outputs]
        updated = [value - error * config.kp for value, error in zip(outputs, errors)]
    return tuple(updated), tuple(outputs)


def phase_peak(currents, theta):
    direct, quadrature = currents
    alpha = direct * math.cos(theta) - quadrature * math.sin(theta)
    beta = direct * math.sin(theta) + quadrature * math.cos(theta)
    return max(abs(alpha), abs(-0.5 * alpha + math.sqrt(3) * 0.5 * beta),
               abs(-0.5 * alpha - math.sqrt(3) * 0.5 * beta))


def simulate(config):
    positive = (config.resistance, config.inductance, config.flux, config.frequency,
                config.voltage_limit, config.duration, config.disturbance_duration)
    finite = positive + (config.rpm, config.kp, config.ki, config.angle_change, config.disturbance_start, config.initial_id)
    if not all(math.isfinite(value) for value in finite) or min(positive) <= 0:
        raise ValueError('Invalid physical or timing configuration')
    if config.kp < 0 or config.ki < 0 or config.pole_pairs < 1 or config.disturbance_start < 0:
        raise ValueError('Invalid gains or disturbance')
    if any(not isinstance(value, int) or not 0 <= value <= 8 for value in (config.sample_delay, config.output_delay)):
        raise ValueError('Delays must be integer control periods, 0..8')
    if not isinstance(config.substeps, int) or not 1 <= config.substeps <= 64:
        raise ValueError('Invalid integration substeps')
    dt = 1 / config.frequency
    omega = config.rpm * config.pole_pairs * math.tau / 60
    coupling = omega * config.inductance if config.coupling else 0.0
    emf = omega * config.flux
    initial_voltage = (-emf - coupling, config.resistance)
    if math.hypot(*initial_voltage) >= config.voltage_limit:
        raise ValueError('Initial equilibrium is outside the voltage limit')
    currents = (config.initial_id, 1.0)
    integrals = initial_voltage
    sample_history = [currents] * config.sample_delay
    output_history = [initial_voltage] * config.output_delay
    trace = []
    overcurrent_time = 0.0
    fault_time = None

    def derivative(state, time, voltage):
        fraction = max(0.0, min(1.0, (time - config.disturbance_start) / config.disturbance_duration))
        angle = math.pi / 2 + config.angle_change * 0.5 * (1 - math.cos(math.pi * fraction))
        relative_speed = config.angle_change * 0.5 * math.pi * math.sin(math.pi * fraction) / config.disturbance_duration
        moving_emf = (omega + relative_speed) * config.flux
        emf_d, emf_q = -moving_emf * math.sin(angle), moving_emf * math.cos(angle)
        direct, quadrature = state
        return ((voltage[0] - config.resistance * direct + coupling * quadrature - emf_d) / config.inductance,
                (voltage[1] - config.resistance * quadrature - coupling * direct - emf_q) / config.inductance)

    for index in range(math.ceil(config.duration / dt)):
        time = index * dt
        sample_history.append(currents)
        feedback = sample_history.pop(0)
        integrals, requested = pi_step(integrals, feedback, config, dt)
        output_history.append(requested)
        voltage = output_history.pop(0)
        for substep in range(config.substeps):
            step = dt / config.substeps
            start = time + substep * step
            first = derivative(currents, start, voltage)
            second = derivative(tuple(value + step * slope / 2 for value, slope in zip(currents, first)), start + step / 2, voltage)
            third = derivative(tuple(value + step * slope / 2 for value, slope in zip(currents, second)), start + step / 2, voltage)
            fourth = derivative(tuple(value + step * slope for value, slope in zip(currents, third)), start + step, voltage)
            currents = tuple(value + step * (a + 2 * b + 2 * c + d) / 6
                             for value, a, b, c, d in zip(currents, first, second, third, fourth))
        peak = phase_peak(currents, omega * (time + dt))
        overcurrent_time = overcurrent_time + dt if peak >= 10 else 0.0
        trace.append((time + dt, *currents, peak, *voltage))
        if not all(math.isfinite(value) for value in trace[-1]):
            raise ArithmeticError('Nonfinite integration result')
        if overcurrent_time >= 0.001 - 1e-12:
            fault_time = time + dt
            break
    return {
        'rpm': config.rpm, 'coupling': config.coupling, 'angle_change_rad': config.angle_change,
        'initial_id_a': config.initial_id,
        'sample_delay_periods': config.sample_delay, 'output_delay_periods': config.output_delay,
        'phase_peak_a': max(row[3] for row in trace),
        'id_min_a': min(row[1] for row in trace), 'iq_min_a': min(row[2] for row in trace),
        'voltage_peak_v': max(math.hypot(row[4], row[5]) for row in trace),
        'final_id_a': trace[-1][1], 'final_iq_a': trace[-1][2],
        'overcurrent_ms': fault_time * 1000 if fault_time is not None else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    results = []
    for angle in (0.0, -0.25, -0.5, -1.0, -1.5, -2.0):
        for coupling in (False, True):
            for sample_delay, output_delay in ((0, 0), (1, 0), (0, 1), (1, 1), (2, 2)):
                result = simulate(Config(angle_change=angle, coupling=coupling,
                                         sample_delay=sample_delay, output_delay=output_delay))
                results.append(result)
                print(json.dumps(result))
    for delay in (0, 1, 2):
        result = simulate(Config(initial_id=0.05, sample_delay=delay, output_delay=delay))
        results.append(result)
        print(json.dumps(result))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({
            'scope': 'Electrical RL only, prescribed EMF-angle perturbation; not a measured rotor trajectory or mechanical stability model',
            'parameters': Config().__dict__, 'results': results,
        }, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
