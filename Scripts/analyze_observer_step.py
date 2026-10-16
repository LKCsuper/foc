"""Verify sampled observer Euler steps; decompose estimated flux, not true rotor motion."""
import argparse
import json
import math
from analyze_handoff import read_frames
from analyze_phase import wrap

HEADER = ('seq,time_us,flags,x_before_alpha,x_before_beta,x_after_alpha,x_after_beta,'
          'i_before_alpha,i_before_beta,i_alpha,i_beta,v_alpha,v_beta,correction_alpha,'
          'correction_beta,dt,resistance,inductance,flux_nominal,gain,pll_before,observer,'
          'phase_error,blend,control_angle,previous_dt,phase_offset,crc32')


def analyze(rows):
    events = [row for row in rows if row['flags'] == 1]
    if not rows or len(events) != 1:
        raise ValueError('Nonempty capture with exactly one event required')
    event = events[0]
    details = []
    for row in rows:
        if row['dt'] <= 0 or row['previous_dt'] < 0 or row['inductance'] < 0 or row['resistance'] < 0 or row['flux_nominal'] <= 0:
            raise ValueError('Invalid effective model/dt')
        current_flux = [row['x_before_' + axis] - row['inductance'] * row['i_' + axis] for axis in ('alpha', 'beta')]
        previous_flux = [row['x_before_' + axis] - row['inductance'] * row['i_before_' + axis] for axis in ('alpha', 'beta')]
        after_flux = [row['x_after_' + axis] - row['inductance'] * row['i_' + axis] for axis in ('alpha', 'beta')]
        magnitude_error = min(0, row['flux_nominal']**2 - sum(value**2 for value in current_flux))
        expected_correction = [0.5 * row['gain'] * value * magnitude_error for value in current_flux]
        emf = [row['v_' + axis] - row['resistance'] * row['i_' + axis] for axis in ('alpha', 'beta')]
        correction = [row['correction_' + axis] for axis in ('alpha', 'beta')]
        delta_current = [-row['inductance'] * (row['i_' + axis] - row['i_before_' + axis]) for axis in ('alpha', 'beta')]
        increments = {'emf': [value * row['dt'] for value in emf],
                      'correction': [value * row['dt'] for value in correction], 'current_change': delta_current}
        previous_square = sum(value**2 for value in previous_flux)
        current_magnitude = math.hypot(*current_flux)
        valid = row['previous_dt'] > 0 and previous_square > 0 and math.hypot(*after_flux) > 0
        projections = {name: (previous_flux[0] * values[1] - previous_flux[1] * values[0]) / previous_square
                       if valid else None for name, values in increments.items()}
        angle_step = wrap(math.atan2(after_flux[1], after_flux[0]) - math.atan2(previous_flux[1], previous_flux[0])) if valid else None
        details.append({
            'seq': int(row['seq']),
            'relative_ms': ((int(row['time_us']) - int(event['time_us']) + 2**31) % 2**32 - 2**31) / 1000,
            'blend': row['blend'], 'phase_error_rad': row['phase_error'],
            'flux_before_Wb': current_magnitude, 'flux_after_Wb': math.hypot(*after_flux),
            'correction_magnitude_V': math.hypot(*correction),
            'correction_formula_residual_V': max(abs(actual - expected) for actual, expected in zip(correction, expected_correction)),
            'euler_residual_Wb': max(abs(row['x_after_' + axis] - row['x_before_' + axis] -
                                      (emf[index] + correction[index]) * row['dt']) for index, axis in enumerate(('alpha', 'beta'))),
            'flux_increment_residual_Wb': max(abs(after_flux[index] - previous_flux[index] - sum(values[index] for values in increments.values())) for index in range(2)) if valid else None,
            'correction_tangent_to_current_flux_V': (current_flux[0] * correction[1] - current_flux[1] * correction[0]) / current_magnitude if current_magnitude else None,
            'estimated_flux_angle_step_rad': angle_step,
            'linear_emf_angle_rad': projections['emf'],
            'linear_correction_angle_rad': projections['correction'],
            'linear_current_change_angle_rad': projections['current_change'],
            'angle_linearization_residual_rad': wrap(angle_step - sum(projections.values())) if valid else None,
            'current_change_flux_magnitude_Wb': math.hypot(*delta_current) if valid else None,
            'observer_atan_residual_rad': wrap(row['observer'] - math.atan2(after_flux[1], after_flux[0]) - row['phase_offset']) if math.hypot(*after_flux) else None})
    ranges = {key: [min(values), max(values)] for key in details[0] if key != 'seq'
              if (values := [sample[key] for sample in details if sample[key] is not None])}
    return {'rows': len(rows), 'ranges': ranges, 'samples': details,
            'note': 'Independent sampled single steps, not consecutive capture rows. Event follows blend completion by one control update. Previous_dt=0 means no valid previous-current history. Linear angle projections are first order and use previous estimated flux, not independent rotor angle. Correction is radial to pre-integration flux with current sample. Voltage is a PWM model; matching software formulas does not validate physical R/L/ADC/voltage accuracy.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='+')
    args = parser.parse_args()
    rows, rejected = read_frames(args.paths, HEADER, 'step')
    result = analyze(rows)
    result['rejected_frames'] = rejected
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
