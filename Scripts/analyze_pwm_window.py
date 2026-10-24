"""Decode a stopped 32-frame PWM write window; timestamps may be sparse or wrap."""
import argparse
import csv
import json
import math
import struct
from pathlib import Path


FRAME = struct.Struct('<7I')
CAPACITY = 32


def decode_window(raw, timer_hz=168_000_000):
    if not math.isfinite(timer_hz) or timer_hz <= 0:
        raise ValueError('Invalid timer frequency')
    if len(raw) != 8 + CAPACITY * FRAME.size:
        raise ValueError('Wrong window size; verify firmware/map pairing')
    enabled, count = struct.unpack_from('<II', raw)
    if enabled or not 1 <= count <= CAPACITY:
        raise ValueError('Window must be stopped and nonempty')
    rows = []
    for index in range(count):
        before_us, after_us, before_cr1, before_count, after_count, after_cr1, period = FRAME.unpack_from(raw, 8 + index * FRAME.size)
        if not period or max(before_count, after_count) > period:
            raise ValueError('Invalid timer counts')
        elapsed = (after_us - before_us) & 0xffffffff
        interval = None if not rows else (before_us - rows[-1]['before_us']) & 0xffffffff
        if interval is not None and not 0 < interval < 0x80000000:
            raise ValueError('Invalid timestamp sequence')
        down = bool(after_cr1 & 16)
        same_direction = bool(before_cr1 & 16) == down
        monotonic = after_count <= before_count if down else after_count >= before_count
        ambiguous = not same_direction or not monotonic or elapsed >= period * 1e6 / timer_hz
        remaining = after_count if down else period - after_count
        rows.append(dict(index=index, before_us=before_us, after_us=after_us, before_cr1=before_cr1,
                         before_count=before_count, after_count=after_count, after_cr1=after_cr1,
                         period=period, elapsed_us=elapsed, interval_us=interval, down=down,
                         ambiguous=ambiguous, after_to_boundary_us=None if ambiguous else remaining * 1e6 / timer_hz))
    margins = [row['after_to_boundary_us'] for row in rows if row['after_to_boundary_us'] is not None]
    intervals = [row['interval_us'] for row in rows[1:]]
    report = dict(count=count, span_us=(rows[-1]['after_us'] - rows[0]['before_us']) & 0xffffffff,
                  interval_us_range=[min(intervals), max(intervals)] if intervals else None,
                  up_frames=sum(not row['down'] for row in rows),
                  direction_change_rows=[current['index'] for previous, current in zip(rows, rows[1:])
                                         if previous['down'] != current['down']],
                  ambiguous_frames=sum(row['ambiguous'] for row in rows),
                  after_to_boundary_us_range=[min(margins), max(margins)] if margins else None,
                  timer_hz=timer_hz,
                  limitations='Sequential register reads, not hardware-latched ADC/CCR transfer timestamps. '
                              'Sparse rows cannot exclude anomalies between records; verify image/map and frozen dump.')
    return rows, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timer-hz', type=float, default=168_000_000)
    args = parser.parse_args()
    rows, report = decode_window(args.input.read_bytes(), args.timer_hz)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.with_suffix('.csv').open('w', newline='', encoding='utf-8') as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
