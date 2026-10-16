"""Check paired raw ADC/current and software PWM submission tags, not hardware latch timing."""
import argparse
import json
import math
from analyze_handoff import read_frames

FRAME = 'sample,pwm,entry_us,ia,ib,ic,adc0,adc1,adc2,ccr0,ccr1,ccr2,count,cr1,recon,period'
HEADER = 'seq,time_us,flags,' + ','.join('prev_' + key for key in FRAME.split(',')) + ',' + FRAME + ',v_alpha,v_beta,crc32'


def clarke(currents):
    return ((2 * currents[0] - currents[1] - currents[2]) / 3,
            (currents[1] - currents[2]) / math.sqrt(3))


def analyze(rows, offsets, scale):
    if not rows or len(offsets) != 3 or not all(map(math.isfinite, offsets)) or not math.isfinite(scale) or scale <= 0:
        raise ValueError('Nonempty capture, finite offsets and positive scale required')
    events = [row for row in rows if row['flags'] == 1]
    if len(events) != 1:
        raise ValueError('Exactly one event required')
    samples = []
    for row in rows:
        conversions, currents, raw = [], [], []
        for prefix in ('prev_', ''):
            for key in FRAME.split(','):
                if key not in ('ia', 'ib', 'ic') and row[prefix + key] != int(row[prefix + key]):
                    raise ValueError('Noninteger metadata')
            for key in ('sample', 'pwm', 'entry_us'):
                if not 0 <= row[prefix + key] < 2**32:
                    raise ValueError('Invalid counter')
            period = row[prefix + 'period']
            if not 0 < period <= 65535 or not 0 <= row[prefix + 'count'] <= period or not 0 <= row[prefix + 'cr1'] <= 65535:
                raise ValueError('Invalid callback metadata')
            if any(not 0 <= row[prefix + 'ccr' + str(index)] <= period for index in range(3)):
                raise ValueError('Invalid CCR')
            adc = [row[prefix + 'adc' + str(index)] for index in range(3)]
            if any(not 0 <= value <= 4095 for value in adc) or row[prefix + 'recon'] not in (0, 1, 2, 3):
                raise ValueError('Invalid ADC/reconstruction metadata')
            expected = [(adc[index] - offsets[index]) * scale for index in (0, 2, 1)]
            reconstruction = int(row[prefix + 'recon'])
            if reconstruction:
                expected[reconstruction - 1] = -sum(value for index, value in enumerate(expected) if index != reconstruction - 1)
            actual = [row[prefix + key] for key in ('ia', 'ib', 'ic')]
            conversions.append(max(abs(first - second) for first, second in zip(expected, actual)))
            currents.append(actual)
            raw.append(adc)
        elapsed = (int(row['entry_us']) - int(row['prev_entry_us'])) % 2**32
        if not 0 < elapsed < 2**31:
            raise ValueError('Invalid paired interval')
        delta_abc = [current - previous for current, previous in zip(currents[1], currents[0])]
        delta_alpha, delta_beta = clarke(delta_abc)
        samples.append({
            'seq': int(row['seq']),
            'relative_ms': ((int(row['time_us']) - int(events[0]['time_us']) + 2**31) % 2**32 - 2**31) / 1000,
            'paired_interval_us': elapsed,
            'sample_sequence_delta': (int(row['sample']) - int(row['prev_sample'])) % 2**32,
            'software_pwm_sequence_delta': (int(row['pwm']) - int(row['prev_pwm'])) % 2**32,
            'conversion_max_error_A': max(conversions),
            'delta_adc_counts': [current - previous for current, previous in zip(raw[1], raw[0])],
            'delta_alpha_A': delta_alpha, 'delta_beta_A': delta_beta,
            'delta_current_vector_A': math.hypot(delta_alpha, delta_beta),
            'delta_zero_sequence_A': sum(delta_abc) / 3,
            'callback_down': bool(int(row['cr1']) & 16),
            'previous_callback_down': bool(int(row['prev_cr1']) & 16),
            'reconstructed': int(row['recon']), 'previous_reconstructed': int(row['prev_recon'])})
    return {'rows': len(rows), 'offsets_adc': offsets, 'scale_A_per_count': scale,
            'paired_interval_range_us': [min(sample['paired_interval_us'] for sample in samples), max(sample['paired_interval_us'] for sample in samples)],
            'sample_sequence_deltas': sorted(set(sample['sample_sequence_delta'] for sample in samples)),
            'software_pwm_sequence_deltas': sorted(set(sample['software_pwm_sequence_delta'] for sample in samples)),
            'conversion_max_error_A': max(sample['conversion_max_error_A'] for sample in samples),
            'callback_direction_changes': sum(sample['callback_down'] != sample['previous_callback_down'] for sample in samples),
            'down_callbacks_in_pairs': sum(sample['callback_down'] + sample['previous_callback_down'] for sample in samples),
            'reconstructed_frames_in_pairs': sum(bool(sample['reconstructed']) + bool(sample['previous_reconstructed']) for sample in samples),
            'largest_delta': max(samples, key=lambda sample: sample['delta_current_vector_A']), 'samples': samples,
            'limitations': ['Raw order CURR1/2/3 maps to logical A/C/B. Current is post reconstruction.',
                            'PWM counter counts software COM submissions, NOT timer update/latch events.',
                            'CR1/CNT/CCR are callback-time sequential reads, NOT ADC trigger-time snapshots or active-CCR proof.',
                            'Software sequence is assigned to selected FOC frames, NOT an ADC DMA hardware sequence.',
                            'Current differences may contain real ripple, noise or timing error; software agreement cannot distinguish them.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='+')
    parser.add_argument('--offsets', required=True, nargs=3, type=float)
    parser.add_argument('--scale', required=True, type=float)
    args = parser.parse_args()
    rows, rejected = read_frames(args.paths, HEADER, 'pair')
    result = analyze(rows, args.offsets, args.scale)
    result['rejected_frames'] = rejected
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
