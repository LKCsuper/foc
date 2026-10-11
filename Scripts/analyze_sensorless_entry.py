"""Compare sensorless phase with time-aligned encoder angles in entry traces.

Offline only. Linear interpolation uses two distinct, valid encoder samples
bracketing each trace timestamp. No encoder RPM is used for this reference.
"""
import argparse
import bisect
import csv
import json
import math
import re
import statistics
from pathlib import Path


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def analyze(path):
    lines = path.read_text(encoding='utf-8').splitlines()
    header = next(line.split(',') for line in lines if line.startswith('time_us,flags,'))
    rows = []
    rejected = 0
    for line in lines:
        if not re.match(r'^\d{6,},\d+,', line):
            continue
        fields = line.split(',')
        try:
            values = list(map(float, fields))
        except ValueError:
            rejected += 1
            continue
        if len(values) != len(header) or not all(map(math.isfinite, values)):
            rejected += 1
            continue
        rows.append(dict(zip(header, values)))
    # Repeated angles represent the same sample. Use median reconstructed time
    # to reduce microsecond differences between trace and age timestamp reads.
    samples = []
    for row in rows:
        stamp = row['time_us'] - row['age'] * 1e6
        valid = bool(int(row['flags']) & 16)
        if samples and row['enc'] == samples[-1]['angle']:
            samples[-1]['times'].append(stamp)
            samples[-1]['valid'] = samples[-1]['valid'] and valid
        else:
            samples.append({'angle': row['enc'], 'times': [stamp], 'valid': valid})
    for sample in samples:
        sample['time'] = statistics.median(sample['times'])
    samples = [s for i, s in enumerate(samples) if i == 0 or s['time'] > samples[i-1]['time']]
    times = [s['time'] for s in samples]
    entry = next((r['time_us'] for r in rows if int(r['flags']) & 256), None)
    output = []
    for row in rows:
        if not (int(row['flags']) & 16) or not 0 <= row['age'] <= .004:
            continue
        i = bisect.bisect_right(times, row['time_us'])
        if i == 0 or i >= len(samples):
            continue
        a, b = samples[i-1:i+1]
        gap = b['time'] - a['time']
        if not a['valid'] or not b['valid'] or not 0 < gap <= 4000:
            continue
        advance = wrap(b['angle'] - a['angle'])
        rpm = advance / (gap / 1e6) * 60 / (2 * math.pi * 7)
        if not 400 < rpm < 850:
            continue
        ref = a['angle'] + advance * (row['time_us'] - a['time']) / gap
        out = dict(row)
        out.update(entry_ms=(row['time_us']-entry)/1000 if entry is not None else None,
                   bracket_us=gap, bracket_rpm=rpm,
                   raw_error=wrap(row['pll']-row['enc']),
                   aligned_error=wrap(row['pll']-ref))
        output.append(out)
    dest = path.with_suffix('.phase.csv')
    if output:
        with dest.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(output[0]))
            writer.writeheader(); writer.writerows(output)
    summary = {'rows': len(rows), 'malformed': rejected, 'entry_us': entry,
               'loss_marked': sum(bool(int(r['flags']) & 128) for r in rows)}
    for name, lo, hi in [('pre', -300, 0), ('blend', 0, 50), ('early', 50, 200), ('later', 200, 1000)]:
        group = [r for r in output if r['entry_ms'] is not None and lo <= r['entry_ms'] < hi]
        if not group:
            summary[name] = {'n': 0}; continue
        metrics = {'n': len(group), 'max_age_ms': max(r['age']*1000 for r in group)}
        for key in ('raw_error', 'aligned_error'):
            errors = [r[key] for r in group]
            absolute = sorted(map(abs, errors))
            metrics[key] = {'median': statistics.median(errors), 'abs_p95': absolute[int(.95*(len(absolute)-1))]}
        summary[name] = metrics
    summary['capture_ms'] = [(r['time_us']-entry)/1000 for r in (rows[0], rows[-1])] if entry is not None else None
    path.with_suffix('.phase.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    print(json.dumps(analyze(parser.parse_args().log), ensure_ascii=False, indent=2))
