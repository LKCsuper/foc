import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from compare_observer_signature import compare, current_relationships, signature


class SignatureTest(unittest.TestCase):
    def rows(self):
        return [dict(flux=flux, flux_nominal=2, phase_error=phase, flags=flag)
                for flux, phase, flag in [(1, .2, 0), (2, .1, 1), (3, 0, 2)]]

    def test_metrics(self):
        result = signature(self.rows())
        self.assertEqual(result['flux_min_ratio'], .5)
        self.assertEqual(result['flux_max_ratio'], 1.5)
        self.assertAlmostEqual(result['flux_phase_correlation'], -1)
        self.assertAlmostEqual(result['phase_rms_rad'], math.sqrt(.05 / 3))

    def test_completion_filter(self):
        result = compare(self.rows(), [])
        self.assertEqual(result['captured']['post_completion']['count'], 2)

    def test_constant_flux_has_no_correlation(self):
        rows = self.rows()
        for row in rows:
            row['flux'] = 2
        self.assertIsNone(signature(rows)['flux_phase_correlation'])

    def test_invalid_input(self):
        for value in (0, -1, float('nan')):
            rows = self.rows()
            rows[0]['flux_nominal'] = value
            with self.assertRaises(ValueError):
                signature(rows)
        with self.assertRaises(ValueError):
            signature([])

    def test_current_projection_and_bins(self):
        rows = self.rows()
        for row in rows:
            row.update(seq=0, i_alpha=0, i_beta=1, x_alpha=2, x_beta=.1, inductance=.1)
        result = current_relationships(rows)
        sample = result['samples'][0]
        self.assertEqual(sample['current_radial_A'], 0)
        self.assertEqual(sample['current_tangent_A'], 1)
        self.assertTrue(sample['near_phase_zero'])
        self.assertEqual(sample['sign_sector'], '0+-')
        self.assertEqual(result['groups']['post_completion']['count'], 2)
        self.assertEqual(result['groups']['all']['bins']['near_phase_zero']['True']['count'], 3)
        self.assertEqual(result['groups']['all']['bins']['near_zero_at_least_0.25A']['False']['count'], 0)
        self.assertIsNone(result['groups']['all']['bins']['near_zero_at_least_0.25A']['False']['phase_rms_rad'])

    def test_current_threshold(self):
        with self.assertRaises(ValueError):
            current_relationships(self.rows(), 0)

    def test_nonfinite_current(self):
        rows = self.rows()
        for row in rows:
            row.update(seq=0, i_alpha=float('nan'), i_beta=1, x_alpha=2, x_beta=0, inductance=.1)
        with self.assertRaises(ValueError):
            current_relationships(rows)


if __name__ == '__main__':
    unittest.main()
