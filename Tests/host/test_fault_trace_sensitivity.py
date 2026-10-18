import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from fault_trace_sensitivity import analyze, clarke, harmonic_fit


class SensitivityTests(unittest.TestCase):
    def test_common_mode_rejected(self):
        self.assertEqual(clarke([1, 1, 1]), 0j)
        self.assertAlmostEqual(abs(clarke([1, 0, 0])), 2 / 3)

    def test_harmonic_fit_noninteger_cycles(self):
        angles = [index * 0.06 for index in range(130)]
        result = harmonic_fit([2 + 3 * math.cos(angle) - 4 * math.sin(angle) for angle in angles], angles)
        self.assertAlmostEqual(result['dc'], 2)
        self.assertAlmostEqual(result['amplitude'], 5)
        self.assertAlmostEqual(result['explained'], 1)
        with self.assertRaises(ValueError):
            harmonic_fit([1, 2, 3], [0, 0, 0])

    def test_sensitivity_scales(self):
        rows = []
        for index in range(300):
            angle = index * 2 * math.pi * 35 * 0.000267
            rows.append(dict(seq=index, time_us=index * 267, prev_theta=math.remainder(angle, 2 * math.pi),
                             ia=math.cos(angle), ib=math.cos(angle - 2 * math.pi / 3),
                             ic=math.cos(angle + 2 * math.pi / 3), prev_v_alpha=math.cos(angle),
                             prev_v_beta=math.sin(angle), prev_flux=0.0024,
                             prev_pll=math.remainder(angle + 0.5, 2 * math.pi)))
        report = analyze(rows)
        self.assertAlmostEqual(report['electrical_Hz'], 35)
        self.assertAlmostEqual(report['voltage_scale_cases'][0]['fundamental_flux_error_scale_Wb'],
                               0.01 / (2 * math.pi * 35))
        self.assertEqual(report['current_offset_cases'][0]['R_times_error_V'], 0)
        self.assertAlmostEqual(report['fitted_stationary_components']['V_minus_RI_dc_magnitude_V'], 0)


if __name__ == '__main__':
    unittest.main()
