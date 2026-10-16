"""Validate CRC-protected stopped handoff exports and inspect PI transport."""
import argparse
import json
import math
import re
from pathlib import Path
from compact_trace import crc32_msb

HEADER = ('seq,time_us,flags,theta,delta,id,iq,old_id,old_iq,id_cmd,iq_cmd,'
          'vd_before,vq_before,vd_transport,vq_transport,vd_after,vq_after,vd,vq,'
          'ia,ib,ic,vbus,kp,dt,adc0,adc1,adc2,crc32')


def read_frames(paths, header=HEADER, label='handoff'):
    expected = None
    frames = {}
    rejected = 0
    for path in paths:
        content = Path(path).read_text(encoding='utf-8')
        count = re.search(re.escape(label) + r' samples=(\d+)', content)
        if not count or header not in content:
            raise ValueError('Missing handoff count/header')
        size = int(count[1])
        if not 0 <= size <= 32 or (expected is not None and expected != size):
            raise ValueError('Handoff buffer count changed or invalid')
        expected = size
        for line in content.split(header, 1)[1].splitlines():
            if not re.match(r'^\d+,', line):
                continue
            try:
                payload, checksum = line.rsplit(',', 1)
                if not re.fullmatch(r'[0-9a-fA-F]{8}', checksum):
                    raise ValueError()
                if crc32_msb(payload.encode('ascii')) != int(checksum, 16):
                    raise ValueError()
                values = list(map(float, payload.split(',')))
                if len(values) != len(header.split(',')) - 1 or not all(map(math.isfinite, values)):
                    raise ValueError()
                row = dict(zip(header.split(',')[:-1], values))
                for key in ('seq', 'time_us', 'flags', 'adc0', 'adc1', 'adc2'):
                    if key not in row:
                        continue
                    if row[key] != int(row[key]):
                        raise ValueError()
                index = int(row['seq'])
                if not 0 <= index < size or not 0 <= row['time_us'] < 2**32 or row['flags'] not in (0, 1, 2):
                    raise ValueError()
                if any(not 0 <= row[key] <= 4095 for key in ('adc0', 'adc1', 'adc2') if key in row):
                    raise ValueError()
            except (ValueError, UnicodeError):
                rejected += 1
                continue
            if index in frames and frames[index] != row:
                raise ValueError('CRC-valid buffer contents changed')
            frames[index] = row
    if expected is None or len(frames) != expected:
        raise ValueError(f'Incomplete CRC-verified handoff: {len(frames)}/{expected}')
    rows = [frames[index] for index in range(expected)]
    for previous, current in zip(rows, rows[1:]):
        delta = (int(current['time_us']) - int(previous['time_us'])) % 2**32
        if not 0 < delta < 2**31:
            raise ValueError('Non-increasing handoff timestamps')
    return rows, rejected


def analyze(rows):
    events = [row for row in rows if row['flags'] == 1]
    if len(events) != 1:
        raise ValueError('Exactly one blend-completion event required')
    event = events[0]
    residuals = []
    for row in rows:
        cosine, sine = math.cos(row['delta']), math.sin(row['delta'])
        expected_d, expected_q = row['vd_before'], row['vq_before']
        if row['delta'] != 0:
            expected_d = (cosine * row['vd_before'] + sine * row['vq_before'] +
                          row['kp'] * (cosine * row['old_id'] + sine * row['old_iq'] - row['id_cmd']))
            expected_q = (-sine * row['vd_before'] + cosine * row['vq_before'] +
                          row['kp'] * (-sine * row['old_id'] + cosine * row['old_iq'] - row['iq_cmd']))
        residuals.append(max(abs(row['vd_transport'] - expected_d), abs(row['vq_transport'] - expected_q)))
    return {'rows': len(rows), 'trigger': event, 'max_transport_residual_V': max(residuals),
            'max_abs_delta_rad': max(abs(row['delta']) for row in rows),
            'max_abs_id_A': max(abs(row['id']) for row in rows),
            'max_sample_current_sum_A': max(abs(row['ia'] + row['ib'] + row['ic']) for row in rows),
            'note': 'Sampled short window, CRC verified; no independent rotor speed or physical current reference.'}


def analyze_currents(rows, offsets, scale):
    """Check this board's ACB mapping and full three-shunt Clarke/Park."""
    if not rows or len(offsets) != 3 or not all(map(math.isfinite, offsets)):
        raise ValueError('Finite three-channel offsets and nonempty rows required')
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('Positive finite current scale required')
    raw_errors, dq_errors = [], []
    for row in rows:
        expected = ((row['adc0'] - offsets[0]) * scale,
                    (row['adc2'] - offsets[2]) * scale,
                    (row['adc1'] - offsets[1]) * scale)
        raw_errors.append(max(abs(row[key] - value) for key, value in zip(('ia', 'ib', 'ic'), expected)))
        alpha = (2 * row['ia'] - row['ib'] - row['ic']) / 3
        beta = (row['ib'] - row['ic']) / math.sqrt(3)
        cosine, sine = math.cos(row['theta']), math.sin(row['theta'])
        dq_errors.append(max(abs(row['id'] - (alpha * cosine + beta * sine)),
                             abs(row['iq'] - (-alpha * sine + beta * cosine))))
    peak = max(rows, key=lambda row: abs(row['id']))
    event = next((row for row in rows if row['flags'] == 1), None)
    return {'offsets_adc': list(offsets), 'scale_A_per_count': scale,
            'raw_to_abc_max_error_A': max(raw_errors), 'abc_to_dq_max_error_A': max(dq_errors),
            'max_current_sum_A': max(abs(row['ia'] + row['ib'] + row['ic']) for row in rows),
            'peak_id': {'id_A': peak['id'], 'iq_A': peak['iq'],
                        'relative_ms': ((int(peak['time_us']) - int(event['time_us']) + 2**31) % 2**32 - 2**31) / 1000 if event else None,
                        'adc': [peak[key] for key in ('adc0', 'adc1', 'adc2')],
                        'abc_A': [peak[key] for key in ('ia', 'ib', 'ic')]},
            'note': 'Agreement verifies software consistency only, not ADC accuracy or true rotor phase; reconstruction can change raw-to-abc values.'}


def analyze_phase_motion(rows, pole_pairs):
    """Differentiate sampled post-blend control angle, not physical shaft speed."""
    if not math.isfinite(pole_pairs) or pole_pairs <= 0:
        raise ValueError('Positive finite pole-pair count required')
    speeds = []
    total_angle, total_us = 0.0, 0
    for previous, current in zip(rows, rows[1:]):
        if previous['flags'] not in (1, 2) or current['flags'] != 2:
            continue
        elapsed = (int(current['time_us']) - int(previous['time_us'])) % 2**32
        if not 0 < elapsed < 2**31:
            raise ValueError('Invalid phase sampling interval')
        angle = (current['theta'] - previous['theta'] + math.pi) % (2 * math.pi) - math.pi
        speeds.append(angle * 60e6 / (2 * math.pi * pole_pairs * elapsed))
        total_angle += angle
        total_us += elapsed
    if not speeds:
        raise ValueError('No adjacent post-blend samples')
    return {'intervals': len(speeds), 'span_ms': total_us / 1000,
            'equivalent_rpm_min': min(speeds), 'equivalent_rpm_max': max(speeds),
            'equivalent_rpm_time_mean': total_angle * 60e6 / (2 * math.pi * pole_pairs * total_us),
            'note': 'Control/PLL angle derivative only, not physical shaft RPM or independent observer validation; assumes less than pi rotation per sample interval.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--offsets', nargs=3, type=float)
    parser.add_argument('--scale', type=float)
    parser.add_argument('--pole-pairs', type=float)
    args = parser.parse_args()
    if (args.offsets is None) != (args.scale is None):
        parser.error('--offsets and --scale must be supplied together')
    samples, rejected = read_frames(args.logs)
    result = analyze(samples)
    result['rejected_frames'] = rejected
    if args.offsets is not None:
        result['current_consistency'] = analyze_currents(samples, args.offsets, args.scale)
    if args.pole_pairs is not None:
        result['phase_motion'] = analyze_phase_motion(samples, args.pole_pairs)
    print(json.dumps(result, ensure_ascii=False, indent=2))
