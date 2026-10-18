"""Input error budgets from a validated decimated trace, not a motor/observer replay."""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path

from compact_trace import crc32_msb
from fault_trace import HEADER


def clarke(abc):
    return complex((2 * abc[0] - abc[1] - abc[2]) / 3,
                   (abc[1] - abc[2]) / math.sqrt(3))


def harmonic_fit(values, angles):
    if len(values) != len(angles) or len(values) < 3:
        raise ValueError('At least three matched samples required')
    basis = [[1, math.cos(angle), math.sin(angle)] for angle in angles]
    matrix = [[sum(row[first] * row[second] for row in basis) for second in range(3)] +
              [sum(row[first] * value for row, value in zip(basis, values))] for first in range(3)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda index: abs(matrix[index][column]))
        matrix[column], matrix[pivot] = matrix[pivot], matrix[column]
        scale = matrix[column][column]
        if abs(scale) < 1e-12:
            raise ValueError('Singular angle distribution')
        matrix[column] = [value / scale for value in matrix[column]]
        for index in range(3):
            if index != column:
                scale = matrix[index][column]
                matrix[index] = [value - scale * reference
                                 for value, reference in zip(matrix[index], matrix[column])]
    coefficients = [row[-1] for row in matrix]
    residual = sum((value - sum(coef * entry for coef, entry in zip(coefficients, row)))**2
                   for row, value in zip(basis, values))
    variance = sum((value - statistics.mean(values))**2 for value in values)
    return {'dc': coefficients[0], 'cos': coefficients[1], 'sin': coefficients[2],
            'amplitude': math.hypot(coefficients[1], coefficients[2]),
            'explained': 1 - residual / variance if variance > 1e-24 else None}


def read_trace(path):
    lines = Path(path).read_text(encoding='utf-8').splitlines()
    if not lines or lines[0] != HEADER or not 3 <= len(lines) - 1 <= 512:
        raise ValueError('Invalid fault CSV')
    for line in lines[1:]:
        payload, checksum = line.rsplit(',', 1)
        if crc32_msb(payload.encode('ascii')) != int(checksum, 16):
            raise ValueError('CRC mismatch')
    rows = [{key: float(value) for key, value in row.items() if key != 'crc32'}
            for row in csv.DictReader(lines)]
    if any(not math.isfinite(value) for row in rows for value in row.values()):
        raise ValueError('Nonfinite input')
    if [row['seq'] for row in rows] != list(range(len(rows))):
        raise ValueError('Invalid row order')
    return rows


def analyze(rows):
    resistance = 0.474
    count_scale = 0.080586076
    angles = [row['prev_theta'] for row in rows]
    elapsed = [0]
    for previous, current in zip(rows, rows[1:]):
        delta = (int(current['time_us']) - int(previous['time_us'])) & 0xffffffff
        if not 0 < delta < 0x80000000:
            raise ValueError('Invalid timestamp order')
        elapsed.append(elapsed[-1] + delta)
    span = elapsed[-1] * 1e-6
    rotation = sum(math.remainder(current - previous, 2 * math.pi)
                   for previous, current in zip(angles, angles[1:]))
    omega = abs(rotation / span)
    if omega <= 0:
        raise ValueError('Nonzero rotation required')
    current = [clarke([row[key] for key in ('ia', 'ib', 'ic')]) for row in rows]
    voltage = [complex(row['prev_v_alpha'], row['prev_v_beta']) for row in rows]
    current_dc = complex(harmonic_fit([value.real for value in current], angles)['dc'],
                         harmonic_fit([value.imag for value in current], angles)['dc'])
    voltage_dc = complex(harmonic_fit([value.real for value in voltage], angles)['dc'],
                         harmonic_fit([value.imag for value in voltage], angles)['dc'])
    emf_dc = voltage_dc - resistance * current_dc
    voltage_rms = math.sqrt(statistics.mean(abs(value)**2 for value in voltage))
    current_cases = []
    for name, counts in [('common_all', (1, 1, 1)), ('A_only', (1, 0, 0)),
                         ('B_only', (0, 1, 0)), ('C_only', (0, 0, 1))]:
        error = clarke([value * count_scale for value in counts])
        current_cases.append({'scenario': name, 'abc_count_error': counts,
                              'alpha_beta_error_A': abs(error),
                              'R_times_error_V': resistance * abs(error),
                              'uncorrected_flux_drift_Wb_over_window': resistance * abs(error) * span})
    return {
        'rows': len(rows), 'window_s': span, 'electrical_Hz': omega / (2 * math.pi),
        'resistance_ohm_assumed': resistance, 'scale_A_per_count_assumed': count_scale,
        'measured_flux_first_harmonic': harmonic_fit([row['prev_flux'] for row in rows], angles),
        'measured_phase_first_harmonic': harmonic_fit(
            [math.remainder(row['prev_pll'] - row['prev_theta'], 2 * math.pi) for row in rows], angles),
        'current_offset_cases': current_cases,
        'voltage_scale_cases': [
            {'fraction': fraction, 'input_error_rms_V': fraction * voltage_rms,
             'fundamental_flux_error_scale_Wb': fraction * voltage_rms / omega}
            for fraction in (0.01, 0.05, 0.10)],
        'voltage_time_shift_cases': [
            {'shift_us': shift, 'fundamental_voltage_difference_V':
             2 * voltage_rms * abs(math.sin(omega * shift * 1e-6 / 2)),
             'fundamental_flux_error_scale_Wb':
             2 * voltage_rms * abs(math.sin(omega * shift * 1e-6 / 2)) / omega}
            for shift in (33.333333, 66.666667, 266.666667)],
        'constant_voltage_cases': [
            {'bias_V': bias, 'uncorrected_flux_drift_Wb_over_window': bias * span}
            for bias in (0.001, 0.005, 0.010)],
        'fitted_stationary_components': {
            'current_dc_alpha_A': current_dc.real, 'current_dc_beta_A': current_dc.imag,
            'voltage_dc_alpha_V': voltage_dc.real, 'voltage_dc_beta_V': voltage_dc.imag,
            'V_minus_RI_dc_magnitude_V': abs(emf_dc),
            'uncorrected_flux_drift_Wb_over_window': abs(emf_dc) * span},
        'limitations': [
            'Decimated current and previous-control voltage are not an exact observer-step pair.',
            'No stator flux vector, initial state or intermediate control frames: no exact replay.',
            'DC drift numbers omit nonlinear flux correction and are not predicted observed flux.',
            'Time-shift and scale estimates assume a balanced fundamental, not inverter simulation.',
            'Uniform scale or fixed time shift alone changes a balanced rotating phasor, not its once-per-turn magnitude.',
            'First-harmonic similarity and fitted DC do not prove a physical bias or identify rotor motion.'
        ]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    args = parser.parse_args()
    print(json.dumps(analyze(read_trace(args.csv)), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
