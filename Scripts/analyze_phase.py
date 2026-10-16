"""Decompose CRC-verified PLL updates; equivalent RPM is not shaft speed."""
import argparse
import json
import math
from analyze_handoff import read_frames

HEADER = ('seq,time_us,flags,observer,pll_before,pll_after,phase_error,speed_before,'
          'speed_after,rpm,flux,x_alpha,x_beta,v_alpha,v_beta,i_alpha,i_beta,dt,kp,ki,'
          'resistance,inductance,flux_nominal,phase_offset,blend,control_angle,crc32')


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def correlation(first, second):
    if len(first) < 2:
        return None
    mean_first, mean_second = sum(first) / len(first), sum(second) / len(second)
    centered_first = [value - mean_first for value in first]
    centered_second = [value - mean_second for value in second]
    denominator = math.sqrt(sum(value * value for value in centered_first) *
                            sum(value * value for value in centered_second))
    return sum(left * right for left, right in zip(centered_first, centered_second)) / denominator if denominator else None


def analyze_vectors(rows, pole_pairs):
    if not rows or not math.isfinite(pole_pairs) or pole_pairs <= 0:
        raise ValueError('Nonempty samples and positive finite pole pairs required')
    samples = []
    events = [row for row in rows if row['flags'] == 1]
    if len(events) != 1:
        raise ValueError('Exactly one completion event required')
    event = events[0]
    for row in rows:
        resistance = max(row['resistance'], 0)
        inductance = max(row['inductance'], 0)
        nominal = max(row['flux_nominal'], 1e-6)
        rotor_alpha = row['x_alpha'] - inductance * row['i_alpha']
        rotor_beta = row['x_beta'] - inductance * row['i_beta']
        magnitude = math.hypot(rotor_alpha, rotor_beta)
        angle = math.atan2(rotor_beta, rotor_alpha)
        cosine, sine = math.cos(angle), math.sin(angle)
        voltage_radial = row['v_alpha'] * cosine + row['v_beta'] * sine
        voltage_tangent = -row['v_alpha'] * sine + row['v_beta'] * cosine
        current_radial = row['i_alpha'] * cosine + row['i_beta'] * sine
        current_tangent = -row['i_alpha'] * sine + row['i_beta'] * cosine
        emf_radial = voltage_radial - resistance * current_radial
        emf_tangent = voltage_tangent - resistance * current_tangent
        exact_error = wrap(angle + row['phase_offset'] - row['pll_before']) if magnitude else None
        samples.append({
            'seq': int(row['seq']), 'flags': int(row['flags']),
            'relative_ms': ((int(row['time_us']) - int(event['time_us']) + 2**31) % 2**32 - 2**31) / 1000,
            'flux_ratio': magnitude / nominal,
            'flux_magnitude_residual_Wb': row['flux'] - magnitude,
            'voltage_magnitude_V': math.hypot(row['v_alpha'], row['v_beta']),
            'current_magnitude_A': math.hypot(row['i_alpha'], row['i_beta']),
            'inductive_flux_fraction': inductance * math.hypot(row['i_alpha'], row['i_beta']) / magnitude if magnitude else None,
            'voltage_radial_V': voltage_radial if magnitude else None,
            'voltage_tangent_V': voltage_tangent if magnitude else None,
            'current_radial_A': current_radial if magnitude else None,
            'current_tangent_A': current_tangent if magnitude else None,
            'v_minus_ri_radial_V': emf_radial if magnitude else None,
            'v_minus_ri_tangent_V': emf_tangent if magnitude else None,
            'voltage_only_angular_proxy_rad_s': emf_tangent / magnitude if magnitude else None,
            'math_atan_phase_error_rad': exact_error,
            'fast_atan_error_rad': wrap(row['observer'] - angle - row['phase_offset']) if magnitude else None,
            'math_atan_proportional_equivalent_RPM': row['kp'] * exact_error * 60 / (2 * math.pi * pole_pairs) if magnitude else None,
            'control_minus_flux_angle_rad': wrap(row['control_angle'] - angle - row['phase_offset']) if magnitude else None,
            'phase_error_rad': row['phase_error']})
    groups = {}
    for name, subset in [('all', samples), ('post_completion', [sample for sample in samples if sample['flags'] in (1, 2)])]:
        valid = [sample for sample in subset if sample['math_atan_phase_error_rad'] is not None]
        groups[name] = {
            'count': len(subset),
            'ranges': {key: [min(values), max(values)] for key in samples[0]
                       if key not in ('seq', 'flags')
                       if (values := [sample[key] for sample in subset if sample[key] is not None])},
            'flux_vs_phase_error_correlation': correlation([sample['flux_ratio'] for sample in valid],
                                                          [sample['phase_error_rad'] for sample in valid]),
            'flux_vs_math_atan_error_correlation': correlation([sample['flux_ratio'] for sample in valid],
                                                              [sample['math_atan_phase_error_rad'] for sample in valid])}
    return {'groups': groups, 'samples': samples,
            'limitations': [
                'Voltage is commanded/applied-model voltage, not measured motor terminal voltage.',
                'All projections use the estimated flux direction, not an independent rotor angle.',
                'x is post-Euler state; pre-Euler state and observer gain are not in this capture.',
                'Angular proxy omits L*di/dt and pre-state correction; it is NOT rotor or flux angular speed.',
                '30ms decimation cannot recover derivatives, nonlinear correction history, offsets, ellipse, or causality.',
                'Zero flux has undefined angle and projections; correlations are sampled associations only.']}


def analyze(rows, pole_pairs):
    if not rows or not math.isfinite(pole_pairs) or pole_pairs <= 0:
        raise ValueError('Nonempty samples and positive finite pole pairs required')
    if sum(row['flags'] == 1 for row in rows) != 1:
        raise ValueError('Exactly one completion event required')
    factor = 60 / (2 * math.pi * pole_pairs)
    details = []
    event = next(row for row in rows if row['flags'] == 1)
    for row in rows:
        if row['dt'] <= 0:
            raise ValueError('Positive control dt required')
        proportional = row['kp'] * row['phase_error']
        predicted = row['pll_before'] + (row['speed_before'] + proportional) * row['dt']
        rotor_alpha = row['x_alpha'] - max(row['inductance'], 0) * row['i_alpha']
        rotor_beta = row['x_beta'] - max(row['inductance'], 0) * row['i_beta']
        details.append({
            'seq': int(row['seq']), 'flags': int(row['flags']),
            'relative_ms': ((int(row['time_us']) - int(event['time_us']) + 2**31) % 2**32 - 2**31) / 1000,
            'phase_error_rad': row['phase_error'], 'flux_Wb': row['flux'],
            'integral_equivalent_RPM': row['speed_before'] * factor,
            'proportional_equivalent_RPM': proportional * factor,
            'total_equivalent_RPM': (row['speed_before'] + proportional) * factor,
            'phase_error_residual_rad': wrap(row['phase_error'] - wrap(row['observer'] - row['pll_before'])),
            'angle_update_residual_rad': wrap(row['pll_after'] - predicted),
            'speed_update_residual_rad_s': row['speed_after'] - row['speed_before'] - row['ki'] * row['phase_error'] * row['dt'],
            'flux_vector_magnitude_Wb': math.hypot(rotor_alpha, rotor_beta),
            'atan2_difference_rad': wrap(row['observer'] - math.atan2(rotor_beta, rotor_alpha) - row['phase_offset'])})
    ranges = {key: [min(item[key] for item in details), max(item[key] for item in details)]
              for key in details[0] if key not in ('seq', 'flags')}
    intervals = [(int(current['time_us']) - int(previous['time_us'])) % 2**32
                 for previous, current in zip(rows, rows[1:])]
    if any(not 0 < elapsed < 2**31 for elapsed in intervals):
        raise ValueError('Non-increasing capture timestamps')
    return {'rows': len(rows), 'ranges': ranges, 'samples': details,
            'largest_phase_error_input': max(rows, key=lambda row: abs(row['phase_error'])),
            'lowest_flux_input': min(rows, key=lambda row: row['flux']),
            'span_ms': sum(intervals) / 1000,
            'max_sample_gap_ms': max(intervals, default=0) / 1000,
            'note': 'Decimated samples can miss between-sample spikes and cannot reconstruct continuous angle motion. Equivalent RPM is not independent shaft speed. Speed residual assumes no clamp/reset; atan2 difference includes fast-atan approximation. Voltage is the observer input from preceding PWM output.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='+')
    parser.add_argument('--pole-pairs', required=True, type=float)
    args = parser.parse_args()
    rows, rejected = read_frames(args.paths, HEADER, 'phase')
    result = analyze(rows, args.pole_pairs)
    result['vector_analysis'] = analyze_vectors(rows, args.pole_pairs)
    result['rejected_frames'] = rejected
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
