"""Bound callback metadata under explicit timer assumptions, not hardware sampling time."""
import argparse
from collections import Counter
import json
import math

from analyze_adc_pair import HEADER
from analyze_handoff import read_frames


def analyze(rows, timer_hz):
    if not rows or not math.isfinite(timer_hz) or timer_hz <= 0:
        raise ValueError('Nonempty capture and positive finite timer clock required')
    frames, intervals, residuals = [], [], []
    direction_changes = 0
    for row in rows:
        for prefix in ('prev_', ''):
            count, period, cr1 = (row[prefix + key] for key in ('count', 'period', 'cr1'))
            if any(value != int(value) for value in (count, period, cr1)):
                raise ValueError('Integer timer metadata required')
            if not 0 < period <= 65535 or not 0 <= count <= period or not 0 <= cr1 <= 65535:
                raise ValueError('Invalid timer metadata')
            down = bool(int(cr1) & 16)
            frames.append({'count': count, 'period': period, 'down': down,
                           'nominal_next_boundary_us': (count if down else period - count) / timer_hz * 1e6})
        elapsed = (int(row['entry_us']) - int(row['prev_entry_us'])) % 2**32
        if not 0 < elapsed < 2**31 or row['period'] != row['prev_period']:
            raise ValueError('Invalid interval or changing period')
        intervals.append(elapsed)
        residuals.append(elapsed - 2 * row['period'] / timer_hz * 1e6)
        direction_changes += frames[-1]['down'] != frames[-2]['down']
    return {
        'pairs': len(rows), 'callback_frames': len(frames), 'assumed_timer_hz': timer_hz,
        'entry_gap_histogram_us': dict(sorted(Counter(intervals).items())),
        'callback_count_range': [min(frame['count'] for frame in frames), max(frame['count'] for frame in frames)],
        'callback_down_frames': sum(frame['down'] for frame in frames),
        'paired_callback_direction_changes': direction_changes,
        'nominal_next_boundary_range_us': [min(frame['nominal_next_boundary_us'] for frame in frames),
                                            max(frame['nominal_next_boundary_us'] for frame in frames)],
        'conditional_gap_minus_full_cycle_range_us': [min(residuals), max(residuals)],
        'limits': [
            'CNT and DIR are sequential metadata reads after current preparation, not a coherent trigger snapshot.',
            'Nominal boundary distance assumes normal counting and given clock; it is not output deadline margin.',
            'Entry gap minus full cycle bounds latency differences only IF samples truly repeat every full cycle.',
            'Timestamp quantization and nominal clock uncertainty remain; absolute service latency is not identified.',
            'Missing hardware sample IDs and write/UEV timestamps prevent proving no lost sample or late latch.',
            'Sparse pairs cannot establish behavior in the unsampled intervals or in another PLL capture run.',
        ]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='+')
    parser.add_argument('--timer-hz', required=True, type=float)
    args = parser.parse_args()
    rows, rejected = read_frames(args.paths, HEADER, 'pair')
    result = analyze(rows, args.timer_hz)
    result['rejected_frames'] = rejected
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
