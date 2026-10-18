"""Fixed-speed electrical PI model and prescribed slip sensitivity; no mechanical replay."""
import argparse
import cmath
import csv
import json
import math
from pathlib import Path

R = 0.474
L = 0.000112
KP = 0.12
KI = 18.0
OMEGA = 3300 * 7 * 2 * math.pi / 60
DT = 1 / 15000
LIMIT = 6.5


def pi_step(target, measured, integral, dt):
    error = target - measured
    integral += KI * error * dt
    axes = []
    accumulators = []
    for value, proportional in ((integral.real, KP * error.real), (integral.imag, KP * error.imag)):
        output = max(-LIMIT, min(LIMIT, value + proportional))
        axes.append(output)
        accumulators.append(output - proportional)
    voltage = complex(*axes)
    integral = complex(*accumulators)
    if abs(voltage) > LIMIT:
        voltage *= LIMIT / abs(voltage)
        integral = voltage - KP * error
    return voltage, integral


def poles(omega=OMEGA):
    coefficient = complex(R + KP, omega * L)
    discriminant = cmath.sqrt(coefficient * coefficient - 4 * L * KI)
    return [(-coefficient + discriminant) / (2 * L), (-coefficient - discriminant) / (2 * L)]


def simulate(emf, case='baseline', lag=0, substeps=8, duration=0.4):
    if lag not in (0, 1) or substeps < 1:
        raise ValueError('Lag must be zero/one and positive substeps required')
    target = 1j
    impedance = complex(R, OMEGA * L)
    current = target
    integral = impedance * target + emf
    if abs(integral) >= LIMIT:
        raise ValueError('Equilibrium outside voltage limit')
    old_feedback = current
    peak = abs(current)
    max_voltage = 0.0
    first_ten = None
    clamps = 0
    rows = []
    step = DT / substeps
    decay = cmath.exp(-impedance / L * step)
    for index in range(round(duration / DT)):
        time = index * DT
        if case == 'current_step' and index == round(0.02 / DT):
            current += 0.2
        feedback = current if lag == 0 else old_feedback
        old_feedback = current
        voltage, integral = pi_step(target, feedback, integral, DT)
        if abs(voltage) >= LIMIT - 1e-9:
            clamps += 1
        max_voltage = max(max_voltage, abs(voltage))
        for inner in range(substeps):
            stamp = time + (inner + 0.5) * step
            applied_emf = emf
            if case == 'phase_step' and stamp >= 0.02:
                applied_emf *= cmath.exp(0.1j)
            if case.startswith('slip_'):
                slip_rpm = float(case.split('_')[1])
                progress = max(0.0, min(0.04, stamp - 0.02))
                phase = -slip_rpm * 7 * 2 * math.pi / 60 * progress
                speed_ratio = 1 - slip_rpm / 3300 if 0.02 <= stamp < 0.06 else 1
                applied_emf *= speed_ratio * cmath.exp(1j * phase)
            current = current * decay + (voltage - applied_emf) / impedance * (1 - decay)
            peak = max(peak, abs(current))
            if abs(current) >= 10 and first_ten is None:
                first_ten = stamp
        if index % 4 == 0:
            rows.append({'time_s': time + DT, 'id_A': current.real, 'iq_A': current.imag,
                         'vd_V': voltage.real, 'vq_V': voltage.imag})
    return {'case': case, 'sample_delay_steps': lag, 'peak_current_vector_A': peak,
            'first_vector_above_10A_s': first_ten, 'max_voltage_V': max_voltage,
            'voltage_clamp_steps': clamps, 'final_target_error_A': abs(current - target),
            'samples': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fault_csv')
    args = parser.parse_args()
    from fault_trace_sensitivity import read_trace
    source = read_trace(args.fault_csv)[0]
    current = complex(source['prev_id'], source['prev_iq'])
    voltage = complex(source['prev_vd_int'] - KP * current.real,
                      source['prev_vq_int'] + KP * (1 - current.imag))
    emf = voltage - complex(R, OMEGA * L) * current
    runs = [simulate(emf, case, lag) for lag in (0, 1)
            for case in ('baseline', 'current_step', 'phase_step', 'slip_25', 'slip_50', 'slip_100')]
    print(json.dumps({
        'R_ohm': R, 'L_H': L, 'Kp': KP, 'Ki': KI, 'control_dt_s': DT,
        'omega_rad_s': OMEGA, 'cross_coupling_ohm': OMEGA * L,
        'fitted_constant_emf_dq_V': [emf.real, emf.imag],
        'continuous_poles_per_s': [[value.real, value.imag] for value in poles()],
        'slow_decay_time_s': -1 / max(value.real for value in poles()),
        'runs': [{key: value for key, value in run.items() if key != 'samples'} for run in runs],
        'limitations': [
            'Plant: L*dI/dt=V-(R+j*omega*L)*I-E, isotropic L, no mechanical equation.',
            'Constant E estimated from one saved operating point, not measured back EMF.',
            'Slip cases impose 25/50/100 mechanical RPM reduction for 40ms, then restore speed; not measured rotor motion.',
            'No sampling noise, deadtime, bus sag, observer feedback or real fault stopping.',
            '10A vector crossing is not max-phase 10A for 1ms fault reproduction.',
            'Stable nominal electrical model does not prove real motor is stable or identify gains as correct.'
        ]}, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
