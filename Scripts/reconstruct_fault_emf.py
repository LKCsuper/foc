"""Reconstruct model-based EMF from a stopped fault export; not independent rotor sensing."""
import argparse
import bisect
import cmath
import csv
import json
import math
import statistics
from pathlib import Path

from compact_trace import crc32_msb


def load_rows(path):
    lines = path.read_text(encoding='utf-8').splitlines()
    for line in lines[1:]:
        payload, checksum = line.rsplit(',', 1)
        if crc32_msb(payload.encode('ascii')) != int(checksum, 16):
            raise ValueError('CRC mismatch')
    rows = [{key: float(value) for key, value in row.items() if key != 'crc32'}
            for row in csv.DictReader(lines)]
    if len(rows) < 9 or any(row['seq'] != index for index, row in enumerate(rows)):
        raise ValueError('Incomplete or unordered export')
    return rows


def circular_mean(values):
    return cmath.phase(sum(cmath.exp(1j * value) for value in values))


def derivative(values, times, index):
    before, after = times[index] - times[index - 1], times[index + 1] - times[index]
    return (-after / (before * (before + after)) * values[index - 1] +
            (after - before) / (before * after) * values[index] +
            before / (after * (before + after)) * values[index + 1])


def reconstruct(rows, resistance=.474, inductance=.000112, method='demod', window=1, voltage_shift=0.0):
    if method not in ('direct', 'demod') or window not in (1, 3, 5) or (method == 'direct' and window != 1):
        raise ValueError('Unsupported derivative/filter combination')
    if not all(math.isfinite(value) for value in (resistance, inductance, voltage_shift)) or min(resistance, inductance) < 0:
        raise ValueError('Invalid model parameters')
    times = [(row['time_us'] - rows[0]['time_us']) * 1e-6 for row in rows]
    if any(after <= before for before, after in zip(times, times[1:])):
        raise ValueError('Non-increasing timestamps')
    angles = [rows[0]['prev_theta']]
    for row in rows[1:]:
        angles.append(angles[-1] + math.remainder(row['prev_theta'] - angles[-1], math.tau))
    current = [complex((2 * row['ia'] - row['ib'] - row['ic']) / 3,
                       (row['ib'] - row['ic']) / math.sqrt(3)) for row in rows]
    voltages = [complex(row['prev_v_alpha'], row['prev_v_beta']) * cmath.exp(-1j * angle)
                for row, angle in zip(rows, angles)]
    envelope = [value * cmath.exp(-1j * angle) for value, angle in zip(current, angles)]
    half = window // 2
    filtered = [sum(envelope[max(0, index - half):min(len(rows), index + half + 1)]) /
                len(envelope[max(0, index - half):min(len(rows), index + half + 1)]) for index in range(len(rows))]
    result = []
    for index in range(half + 1, len(rows) - half - 1):
        target = times[index] - voltage_shift
        right = bisect.bisect_left(times, target)
        if right == 0 or right >= len(times):
            continue
        left = right - 1
        fraction = (target - times[left]) / (times[right] - times[left])
        voltage = (voltages[left] * (1 - fraction) + voltages[right] * fraction) * cmath.exp(
            1j * (angles[left] * (1 - fraction) + angles[right] * fraction))
        if method == 'direct':
            measured = current[index]
            slope = derivative(current, times, index)
        else:
            rotation = cmath.exp(1j * angles[index])
            measured = filtered[index] * rotation
            slope = rotation * (derivative(filtered, times, index) +
                                1j * derivative(angles, times, index) * filtered[index])
        emf = voltage - resistance * measured - inductance * slope
        flux_angle = cmath.phase(emf) - math.pi / 2
        result.append({'ms': (times[index] - times[-1]) * 1000,
                       'emf_v': abs(emf),
                       'emf_relative': math.remainder(flux_angle - angles[index], math.tau),
                       'pll_relative': math.remainder(rows[index]['prev_pll'] - angles[index], math.tau)})
    baseline = [row for row in result if -137 <= row['ms'] < -100 and row['emf_v'] >= 1]
    if not baseline:
        raise ValueError('No usable baseline')
    emf_base = circular_mean(row['emf_relative'] for row in baseline)
    pll_base = circular_mean(row['pll_relative'] for row in baseline)
    for row in result:
        row['emf_drift'] = math.remainder(row['emf_relative'] - emf_base, math.tau)
        row['pll_drift'] = math.remainder(row['pll_relative'] - pll_base, math.tau)
    return result


def summarize(result):
    summary = []
    for lower, upper in ((-137, -100), (-100, -50), (-50, -20), (-20, -10), (-10, -5), (-5, 0)):
        group = [row for row in result if lower <= row['ms'] < upper and row['emf_v'] >= 1]
        summary.append({'window_ms': [lower, upper], 'count': len(group),
                        'emf_median_v': statistics.median(row['emf_v'] for row in group),
                        'emf_drift_rad': circular_mean(row['emf_drift'] for row in group),
                        'pll_drift_rad': circular_mean(row['pll_drift'] for row in group)})
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows = load_rows(args.csv)
    cases = []
    for method, window in (('direct', 1), ('demod', 1), ('demod', 3), ('demod', 5)):
        for shift in (-1 / 15000, 0.0, 1 / 15000):
            for resistance_scale, inductance_scale in ((1, 1), (.8, 1), (1.2, 1), (1, .8), (1, 1.2)):
                result = reconstruct(rows, .474 * resistance_scale, .000112 * inductance_scale,
                                     method, window, shift)
                cases.append({'method': method, 'window': window, 'voltage_shift_us': shift * 1e6,
                              'resistance_scale': resistance_scale, 'inductance_scale': inductance_scale,
                              'summary': summarize(result)})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'scope': 'Model-based, shared voltage/current inputs with PLL; not independent rotor truth',
                                      'cases': cases}, indent=2), encoding='utf-8')
    print('Wrote', len(cases), 'sensitivity cases to', args.output)


if __name__ == '__main__':
    main()
