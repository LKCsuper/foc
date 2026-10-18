"""Screen early-stop candidates on decimated logs; not a real-time protection validation."""
import argparse
import json
import math

from fault_trace_sensitivity import read_trace


def replay(rows, threshold=2.0, hold_us=2000, max_gap_us=400):
    if threshold <= 0 or not math.isfinite(threshold) or hold_us < 0 or max_gap_us <= 0:
        raise ValueError('Invalid guard parameters')
    start = None
    previous = None
    first = None
    maximum = 0.0
    crossings = 0
    for row in rows:
        stamp = int(row['time_us'])
        values = [row[key] for key in ('prev_id', 'prev_iq', 'prev_id_cmd', 'prev_iq_cmd')]
        if not all(map(math.isfinite, values)):
            raise ValueError('Nonfinite current input')
        if previous is not None:
            gap = (stamp - previous) & 0xffffffff
            if not 0 < gap < 0x80000000:
                raise ValueError('Invalid timestamp order')
            if gap > max_gap_us:
                start = None
        error = math.hypot(values[0] - values[2], values[1] - values[3])
        maximum = max(maximum, error)
        if error >= threshold:
            crossings += 1
            if start is None:
                start = stamp
            if first is None and ((stamp - start) & 0xffffffff) >= hold_us:
                first = {'time_us': stamp, 'seq': int(row['seq']), 'error_A': error,
                         'observed_span_us': (stamp - start) & 0xffffffff}
        else:
            start = None
        previous = stamp
    if first is not None:
        first['lead_to_last_sample_ms'] = ((int(rows[-1]['time_us']) - first['time_us']) & 0xffffffff) / 1000
    return {'threshold_A': threshold, 'hold_us': hold_us, 'max_gap_us': max_gap_us,
            'max_error_A': maximum, 'above_threshold_rows': crossings, 'first_candidate': first}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('normal_csv')
    parser.add_argument('fault_csv')
    args = parser.parse_args()
    normal = read_trace(args.normal_csv)
    fault = read_trace(args.fault_csv)
    if not all(int(row['flags']) >> 16 == 0 for row in normal):
        raise ValueError('Normal reference has fault flags')
    if int(fault[-1]['flags']) >> 16 != 4:
        raise ValueError('Expected final overcurrent trigger marker')
    print(json.dumps({
        'metric': 'hypot(previous_Id-Id_target, previous_Iq-Iq_target)',
        'normal_rows': len(normal), 'fault_rows': len(fault),
        'candidates': [{'normal': replay(normal, threshold, hold),
                        'fault': replay(fault, threshold, hold)}
                       for threshold in (1.0, 1.5, 2.0, 3.0) for hold in (1000, 2000, 5000)],
        'limitations': [
            'Only one 300RPM normal window and one 3300RPM fault window, about 136ms each.',
            'Hold means successive recorded points, not proof of uninterrupted between-sample exceedance.',
            'No state change, gate shutoff, current decay or real-time execution has been simulated.',
            'A new stop could change the trajectory; lead time is historical screening, not guaranteed safety margin.',
            'No startup/reverse/load-change/high-speed-normal coverage, so no false-positive-rate claim.'
        ]}, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
