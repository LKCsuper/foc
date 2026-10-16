"""Compare adjacent ADC DMA service callbacks without asserting hardware trigger timing."""
import argparse
import json
import math
from analyze_adc_pair import HEADER as PAIR_HEADER, analyze as analyze_pair, clarke
from analyze_handoff import read_frames
from analyze_phase import correlation

HEADER = PAIR_HEADER.replace('prev_recon', 'prev_capture_flags').replace(',recon,', ',capture_flags,')


def difference_statistics(signed, scale):
    if not signed:
        return {'count': 0, 'note': 'No stable alternating pairs'}
    means = [sum(values) / len(values) for values in zip(*signed)]
    common = [sum(values) / 3 for values in signed]
    residuals = [[value - average for value in values] for values, average in zip(signed, common)]
    total_energy = sum(value * value for values in signed for value in values)
    common_energy = 3 * sum(value * value for value in common)
    differential_energy = sum(value * value for values in residuals for value in values)
    bias_energy = len(signed) * sum(value * value for value in means)
    vectors = [clarke([values[index] * scale for index in (0, 2, 1)]) for values in signed]
    vector_mean = [sum(values) / len(values) for values in zip(*vectors)]
    return {
        'count': len(signed),
        'mean_raw_delta_counts': means,
        'common_mean_counts': sum(common) / len(common),
        'common_rms_counts': math.sqrt(sum(value * value for value in common) / len(common)),
        'differential_rms_per_channel_counts': math.sqrt(differential_energy / (3 * len(signed))),
        'common_energy_fraction': common_energy / total_energy if total_energy else None,
        'differential_energy_fraction': differential_energy / total_energy if total_energy else None,
        'fixed_mean_energy_fraction': bias_energy / total_energy if total_energy else None,
        'centered_rms_channel_counts': [math.sqrt(sum((values[index] - means[index])**2 for values in signed) / len(signed)) for index in range(3)],
        'raw_delta_correlations': {f'{first}_{second}': correlation([values[first] for values in signed],
                                                                   [values[second] for values in signed])
                                   for first, second in ((0, 1), (0, 2), (1, 2))},
        'mean_clarke_delta_A': vector_mean,
        'rms_clarke_delta_vector_A': math.sqrt(sum(sum(value * value for value in vector) for vector in vectors) / len(vectors)),
        'centered_rms_clarke_delta_vector_A': math.sqrt(sum(sum((vector[index] - vector_mean[index])**2 for index in range(2)) for vector in vectors) / len(vectors)),
        'note': 'Common/differential split is orthogonal across the three raw channels. Fixed-mean fraction is a different decomposition and must not be added to those fractions. Centered RMS is not an independent noise estimate; adjacent samples may be correlated.'}


def analyze(rows, offsets, scale, allow_untriggered=False):
    if len(offsets) != 3 or not all(map(math.isfinite, offsets)) or not math.isfinite(scale) or scale <= 0:
        raise ValueError('Finite offsets and positive current scale required')
    normalized = []
    for row in rows:
        copy = row.copy()
        for prefix in ('prev_', ''):
            if any(row[prefix + key] != 0 for key in ('ia', 'ib', 'ic')):
                raise ValueError('Half capture current slots must be zero placeholders')
            status = row[prefix + 'capture_flags']
            if status != int(status) or not 0 <= status <= 15:
                raise ValueError('Invalid capture flags')
            copy[prefix + 'recon'] = 0
            for key, channel in zip(('ia', 'ib', 'ic'), (0, 2, 1)):
                copy[prefix + key] = (row[prefix + 'adc' + str(channel)] - offsets[channel]) * scale
        normalized.append(copy)
    result = analyze_pair(normalized, offsets, scale, require_event=not allow_untriggered)
    if result['time_reference'] == 'completion_event':
        result['time_reference'] = 'first_DMA_service_after_completion_mark'
    del result['conversion_max_error_A']
    del result['reconstructed_frames_in_pairs']
    for row, sample in zip(rows, result['samples']):
        sample['capture_flags'] = int(row['capture_flags'])
        sample['previous_capture_flags'] = int(row['prev_capture_flags'])
        del sample['conversion_max_error_A']
        del sample['reconstructed']
        del sample['previous_reconstructed']
    usable = [sample for sample in result['samples']
              if not sample['capture_flags'] and not sample['previous_capture_flags']
              and sample['sample_sequence_delta'] == 1 and sample['callback_down'] != sample['previous_callback_down']]
    groups = {}
    for name, subset in [('all_stable_alternating', usable),
                         ('same_software_pwm', [sample for sample in usable if sample['software_pwm_sequence_delta'] == 0]),
                         ('one_software_pwm_between', [sample for sample in usable if sample['software_pwm_sequence_delta'] == 1])]:
        signed = [[value * (1 if sample['callback_down'] else -1) for value in sample['delta_adc_counts']] for sample in subset]
        groups[name] = {'count': len(subset),
                        'mean_down_minus_up_counts': [sum(values) / len(values) for values in zip(*signed)] if signed else None,
                        'rms_channel_delta_counts': [math.sqrt(sum(value * value for value in values) / len(values)) for values in zip(*signed)] if signed else None,
                        'max_delta_vector_A': max((sample['delta_current_vector_A'] for sample in subset), default=None),
                        'difference_statistics': difference_statistics(signed, scale)}
    result['groups'] = groups
    result['unstable_pairs'] = sum(bool(sample['capture_flags'] or sample['previous_capture_flags']) for sample in result['samples'])
    result['limitations'] = [
        'Raw ADC only. ia/ib/ic CSV slots are zero, NOT measured phase currents; currents here are offline ACB conversions.',
        'Capture flags: bit0 raw values changed on reread; bit1 DIR changed; bit2 DMA NDTR changed; bit3 software PWM sequence changed.',
        'Zero flags do NOT prove atomic DMA coherence or absence of full circular-buffer overwrite.',
        'DIR/CNT/CCR are DMA service-time sequential reads, not hardware ADC-trigger or active-CCR snapshots.',
        'Software PWM sequence counts COM calls, not physical preload transfer. Same tag does not prove same applied waveform.',
        'Adjacent opposite-direction callbacks are candidate half cycles, not certified same-period trigger pairs.',
        'Differences combine real current evolution, switching ripple, measurement noise and possible timing error; no causal attribution.']
    result['limitations'].append('32 decimated pairs do not support spectral reconstruction, a hardware fault diagnosis, or dividing difference RMS by sqrt(2) to infer independent sample noise.')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='+')
    parser.add_argument('--offsets', nargs=3, required=True, type=float)
    parser.add_argument('--scale', required=True, type=float)
    parser.add_argument('--allow-untriggered', action='store_true', help='Explicit stopped baseline with no completion event')
    args = parser.parse_args()
    rows, rejected = read_frames(args.paths, HEADER, 'half')
    result = analyze(rows, args.offsets, args.scale, args.allow_untriggered)
    result['rejected_frames'] = rejected
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
