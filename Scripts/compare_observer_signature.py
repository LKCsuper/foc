"""Compare sparse captured PLL inputs with synthetic signatures, not rotor truth."""
import argparse
import csv
import json
import math

from analyze_handoff import read_frames
from analyze_phase import HEADER, correlation


def signature(rows):
    if len(rows) < 2:
        raise ValueError('At least two samples required')
    if any(not math.isfinite(row[key]) for row in rows
           for key in ('flux', 'flux_nominal', 'phase_error')):
        raise ValueError('Nonfinite input')
    if any(row['flux_nominal'] <= 0 for row in rows):
        raise ValueError('Positive nominal flux required')
    flux = [row['flux'] / row['flux_nominal'] for row in rows]
    phase = [row['phase_error'] for row in rows]
    return {
        'count': len(rows), 'flux_min_ratio': min(flux), 'flux_max_ratio': max(flux),
        'phase_rms_rad': math.sqrt(sum(value * value for value in phase) / len(phase)),
        'phase_min_rad': min(phase), 'phase_max_rad': max(phase),
        'flux_phase_correlation': correlation(flux, phase),
    }


def compare(rows, simulations):
    groups = {'all': rows, 'post_completion': [row for row in rows if row['flags'] in (1, 2)]}
    return {
        'captured': {name: signature(group) for name, group in groups.items()},
        'simulated': simulations,
        'limitations': [
            'Captured samples are decimated and not independent rotor truth.',
            'Simulation uses fixed speed/current; captured closed-loop feedback differs.',
            'Different durations, sampling and initial states prevent causal ranking or parameter fitting.',
            'Matching ranges or correlation does not identify a physical voltage error.',
        ],
    }


def current_relationships(rows, count_amperes=0.0805860805860806):
    if not math.isfinite(count_amperes) or count_amperes <= 0:
        raise ValueError('Positive finite current count required')
    samples = []
    for row in rows:
        if any(not math.isfinite(row[key]) for key in
               ('i_alpha', 'i_beta', 'x_alpha', 'x_beta', 'inductance', 'flux', 'flux_nominal', 'phase_error')):
            raise ValueError('Finite current and observer inputs required')
        alpha, beta = row['i_alpha'], row['i_beta']
        magnitude = math.hypot(alpha, beta)
        phase_currents = [alpha, -.5 * alpha + math.sqrt(3) / 2 * beta,
                         -.5 * alpha - math.sqrt(3) / 2 * beta]
        flux_alpha = row['x_alpha'] - row['inductance'] * alpha
        flux_beta = row['x_beta'] - row['inductance'] * beta
        flux_magnitude = math.hypot(flux_alpha, flux_beta)
        if flux_magnitude <= 0 or row['flux_nominal'] <= 0:
            raise ValueError('Positive flux required')
        signs = ['+' if value > 0 else '-' if value < 0 else '0' for value in phase_currents]
        samples.append({
            'seq': row['seq'], 'flags': row['flags'],
            'current_magnitude_A': magnitude,
            'min_abs_reconstructed_phase_A': min(map(abs, phase_currents)),
            'near_phase_zero': min(map(abs, phase_currents)) <= count_amperes,
            'low_current_vector': magnitude < .25,
            'sign_sector': ''.join(signs),
            'current_radial_A': (alpha * flux_alpha + beta * flux_beta) / flux_magnitude,
            'current_tangent_A': (-alpha * flux_beta + beta * flux_alpha) / flux_magnitude,
            'flux_ratio': row['flux'] / row['flux_nominal'],
            'phase_error_rad': row['phase_error'],
            'abs_phase_error_rad': abs(row['phase_error']),
        })
    groups = {}
    for name, subset in [('all', samples),
                         ('post_completion', [sample for sample in samples if sample['flags'] in (1, 2)])]:
        if len(subset) < 2:
            raise ValueError('At least two samples per group required')
        correlations = {}
        for feature in ('current_magnitude_A', 'min_abs_reconstructed_phase_A',
                        'current_radial_A', 'current_tangent_A'):
            correlations[feature] = {
                target: correlation([sample[feature] for sample in subset],
                                    [sample[target] for sample in subset])
                for target in ('flux_ratio', 'phase_error_rad', 'abs_phase_error_rad')}
        bins = {}
        for category in ('near_phase_zero', 'low_current_vector', 'sign_sector'):
            bins[category] = {}
            for value in sorted(set(sample[category] for sample in subset)):
                members = [sample for sample in subset if sample[category] == value]
                bins[category][str(value)] = {
                    'count': len(members),
                    'phase_rms_rad': math.sqrt(sum(sample['phase_error_rad'] ** 2 for sample in members) / len(members)),
                    'flux_range': [min(sample['flux_ratio'] for sample in members),
                                   max(sample['flux_ratio'] for sample in members)],
                }
        bins['near_zero_at_least_0.25A'] = {}
        for value in (False, True):
            members = [sample for sample in subset if not sample['low_current_vector']
                       and sample['near_phase_zero'] == value]
            bins['near_zero_at_least_0.25A'][str(value)] = {
                'count': len(members),
                'phase_rms_rad': math.sqrt(sum(sample['phase_error_rad'] ** 2 for sample in members) /
                                          len(members)) if members else None}
        groups[name] = {'count': len(subset), 'correlations': correlations, 'bins': bins,
                        'current_magnitude_range_A': [min(sample['current_magnitude_A'] for sample in subset),
                                                      max(sample['current_magnitude_A'] for sample in subset)]}
    return {'groups': groups, 'samples': samples, 'near_zero_threshold_A': count_amperes,
            'limitations': 'Reconstructed balanced phase currents omit common mode, not raw ADC phases. '
                           'One-count proximity is not a physical zero-crossing time. '
                           'Flux coordinates use the same observer estimate, not independent rotor angle. '
                           'Small unequal decimated groups and correlated closed-loop signals do not establish causality.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='+')
    parser.add_argument('--simulation', required=True)
    args = parser.parse_args()
    rows, rejected = read_frames(args.paths, HEADER, 'phase')
    with open(args.simulation, encoding='utf-8-sig') as source:
        simulations = []
        for row in csv.DictReader(source):
            sample = {'scenario': row['scenario']}
            for key in ('flux_min_ratio', 'flux_max_ratio', 'phase_rms_rad', 'phase_min_rad',
                        'phase_max_rad', 'flux_phase_correlation'):
                value = float(row[key])
                sample[key] = value if math.isfinite(value) else None
            simulations.append(sample)
    result = compare(rows, simulations)
    result['current_relationships'] = current_relationships(rows)
    result['rejected_frames'] = rejected
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
