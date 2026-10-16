import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from analyze_observer_step import HEADER, analyze
from analyze_handoff import read_frames
from compact_trace import crc32_msb


class ObserverStepTest(unittest.TestCase):
    def row(self):
        row = dict.fromkeys(HEADER.split(',')[:-1], 0.0)
        row.update(seq=0, flags=1, dt=0.001, previous_dt=0.001, flux_nominal=0.002,
                   gain=12000000, resistance=0.5, inductance=0.001,
                   x_before_alpha=0.003, i_alpha=0.5, i_before_alpha=0.4, v_alpha=0.25, v_beta=1)
        row['correction_alpha'] = 0.5 * row['gain'] * 0.0025 * (0.002**2 - 0.0025**2)
        row['x_after_alpha'] = row['x_before_alpha'] + row['correction_alpha'] * row['dt']
        row['x_after_beta'] = 0.001
        return row

    def test_single_step_and_current_difference(self):
        result = analyze([self.row()])['samples'][0]
        for key in ('euler_residual_Wb', 'flux_increment_residual_Wb', 'correction_formula_residual_V', 'correction_tangent_to_current_flux_V'):
            self.assertAlmostEqual(result[key], 0)
        self.assertAlmostEqual(result['current_change_flux_magnitude_Wb'], 0.0001)
        self.assertGreater(result['linear_emf_angle_rad'], 0)

    def test_zero_correction_and_invalid_previous(self):
        row = self.row()
        row.update(flux_nominal=0.003, correction_alpha=0, x_after_alpha=0.003, previous_dt=0)
        result = analyze([row])['samples'][0]
        self.assertEqual(result['correction_formula_residual_V'], 0)
        self.assertIsNone(result['estimated_flux_angle_step_rad'])
        self.assertIsNone(result['linear_current_change_angle_rad'])
        row['dt'] = 0
        with self.assertRaises(ValueError):
            analyze([row])

    def test_crc_schema(self):
        row = self.row()
        payload = ','.join(str(row[key]) for key in HEADER.split(',')[:-1])
        text = f'step samples=1\n{HEADER}\n{payload},{crc32_msb(payload.encode()):08x}'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'step.log'
            path.write_text(text, encoding='utf-8')
            rows, rejected = read_frames([path], HEADER, 'step')
            self.assertEqual((len(rows), rejected), (1, 0))
            path.write_text(text[:-1] + 'x', encoding='utf-8')
            with self.assertRaises(ValueError):
                read_frames([path], HEADER, 'step')


if __name__ == '__main__':
    unittest.main()
