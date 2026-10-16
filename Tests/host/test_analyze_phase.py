import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from analyze_phase import HEADER, analyze, analyze_vectors, wrap
from analyze_handoff import read_frames
from compact_trace import crc32_msb


class PhaseTest(unittest.TestCase):
    def row(self):
        row = dict.fromkeys(HEADER.split(',')[:-1], 0.0)
        row.update(seq=0, flags=1, time_us=1000, pll_before=3.14, observer=-3.12,
                   speed_before=440, dt=0.000067, kp=2000, ki=30000)
        row['phase_error'] = wrap(row['observer'] - row['pll_before'])
        row['pll_after'] = wrap(row['pll_before'] + (440 + 2000 * row['phase_error']) * row['dt'])
        row['speed_after'] = 440 + 30000 * row['phase_error'] * row['dt']
        return row

    def test_decomposition_wrap(self):
        result = analyze([self.row()], 7)
        for key in ('angle_update_residual_rad', 'phase_error_residual_rad', 'speed_update_residual_rad_s'):
            self.assertLess(abs(result['ranges'][key][1]), 1e-12)
        self.assertGreater(result['ranges']['proportional_equivalent_RPM'][0], 0)
        with self.assertRaises(ValueError):
            analyze([self.row()], 0)

    def test_crc_phase_schema(self):
        row = self.row()
        payload = ','.join(str(row[key]) for key in HEADER.split(',')[:-1])
        frame = payload + f',{crc32_msb(payload.encode()):08x}'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'phase.log'
            path.write_text('phase samples=1\n' + HEADER + '\n' + frame, encoding='utf-8')
            rows, rejected = read_frames([path], HEADER, 'phase')
            self.assertEqual((len(rows), rejected), (1, 0))
            path.write_text('phase samples=1\n' + HEADER + '\n' + frame[:-1] + 'x', encoding='utf-8')
            with self.assertRaises(ValueError):
                read_frames([path], HEADER, 'phase')

    def test_extended_window_wrap(self):
        first, event, last = self.row(), self.row(), self.row()
        first.update(seq=0, flags=0, time_us=2**32 - 30000)
        event.update(seq=1, flags=1, time_us=0)
        last.update(seq=2, flags=2, time_us=810000)
        result = analyze([first, event, last], 7)
        self.assertEqual(result['largest_phase_error_input'], first)
        self.assertEqual(result['lowest_flux_input'], first)
        self.assertEqual(result['span_ms'], 840)
        self.assertEqual(result['max_sample_gap_ms'], 810)
        self.assertEqual([row['relative_ms'] for row in result['samples']], [-30, 0, 810])
        with self.assertRaises(ValueError):
            analyze([event, first, last], 7)

    def test_vector_projections_and_rotation(self):
        row = self.row()
        row.update(resistance=0.5, inductance=0.001, flux_nominal=0.002,
                   i_alpha=0.3, i_beta=0.4, x_alpha=0.0023, x_beta=0.0004,
                   v_alpha=0.15, v_beta=1.2, flux=0.002, observer=0.005,
                   pll_before=0.025, phase_error=-0.02, phase_offset=0, control_angle=0)
        result = analyze_vectors([row], 7)['samples'][0]
        self.assertAlmostEqual(result['flux_ratio'], 1)
        self.assertAlmostEqual(result['inductive_flux_fraction'], 0.25)
        self.assertAlmostEqual(result['v_minus_ri_radial_V'], 0)
        self.assertAlmostEqual(result['v_minus_ri_tangent_V'], 1)
        self.assertAlmostEqual(result['voltage_only_angular_proxy_rad_s'], 500)
        self.assertAlmostEqual(result['math_atan_phase_error_rad'], -0.025)
        self.assertAlmostEqual(result['fast_atan_error_rad'], 0.005)
        rotated = row.copy()
        rotation = 1.2
        for first, second in [('i_alpha', 'i_beta'), ('x_alpha', 'x_beta'), ('v_alpha', 'v_beta')]:
            rotated[first] = row[first] * math.cos(rotation) - row[second] * math.sin(rotation)
            rotated[second] = row[first] * math.sin(rotation) + row[second] * math.cos(rotation)
        for key in ('observer', 'pll_before', 'control_angle'):
            rotated[key] += rotation
        actual = analyze_vectors([rotated], 7)['samples'][0]
        for key in result:
            if result[key] is not None:
                self.assertAlmostEqual(actual[key], result[key], places=10)

    def test_zero_flux_and_constant_correlation(self):
        result = analyze_vectors([self.row()], 7)
        self.assertIsNone(result['samples'][0]['math_atan_phase_error_rad'])
        self.assertIsNone(result['samples'][0]['voltage_only_angular_proxy_rad_s'])
        self.assertIsNone(result['groups']['all']['flux_vs_phase_error_correlation'])
        with self.assertRaises(ValueError):
            analyze_vectors([self.row()], float('nan'))
        duplicate = self.row()
        with self.assertRaises(ValueError):
            analyze_vectors([duplicate, duplicate], 7)

    def test_post_completion_correlation(self):
        rows = []
        for index, (flux, error) in enumerate([(0.004, -0.1), (0.002, 0.1), (0.003, -0.1)]):
            row = self.row()
            row.update(seq=index, flags=index, time_us=1000 + index * 30000,
                       x_alpha=flux, x_beta=0, flux=flux, flux_nominal=0.002,
                       observer=0, pll_before=-error, phase_error=error)
            rows.append(row)
        result = analyze_vectors(rows, 7)
        self.assertEqual(result['groups']['post_completion']['count'], 2)
        self.assertAlmostEqual(result['groups']['post_completion']['flux_vs_math_atan_error_correlation'], -1)


if __name__ == '__main__':
    unittest.main()
