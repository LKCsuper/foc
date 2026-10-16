import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from analyze_adc_half import HEADER, analyze, difference_statistics
from analyze_handoff import read_frames
from compact_trace import crc32_msb


class HalfTest(unittest.TestCase):
    def row(self):
        row = dict.fromkeys(HEADER.split(',')[:-1], 0)
        row.update(flags=1, time_us=1000, prev_sample=1, sample=2, prev_pwm=1, pwm=1,
                   prev_entry_us=100, entry_us=133, prev_period=5600, period=5600, cr1=16,
                   adc0=2049, adc1=2050, adc2=2051, prev_adc0=2048, prev_adc1=2048, prev_adc2=2048)
        return row

    def test_direction_and_software_grouping(self):
        row = self.row()
        result = analyze([row], [2048] * 3, 0.1)
        self.assertEqual(result['time_reference'], 'first_DMA_service_after_completion_mark')
        self.assertEqual(result['groups']['same_software_pwm']['count'], 1)
        self.assertEqual(result['groups']['all_stable_alternating']['mean_down_minus_up_counts'], [1, 2, 3])
        row.update(prev_cr1=16, cr1=0, pwm=2)
        result = analyze([row], [2048] * 3, 0.1)
        self.assertEqual(result['groups']['one_software_pwm_between']['mean_down_minus_up_counts'], [-1, -2, -3])

    def test_unstable_excluded_and_metadata_rejected(self):
        row = self.row()
        row['capture_flags'] = 4
        result = analyze([row], [2048] * 3, 0.1)
        self.assertEqual(result['unstable_pairs'], 1)
        self.assertEqual(result['groups']['all_stable_alternating']['count'], 0)
        row['capture_flags'] = 16
        with self.assertRaises(ValueError):
            analyze([row], [2048] * 3, 0.1)
        row['capture_flags'] = 0
        row['prev_ia'] = 0.1
        with self.assertRaises(ValueError):
            analyze([row], [2048] * 3, 0.1)

    def test_crc_header(self):
        row = self.row()
        payload = ','.join(str(row[key]) for key in HEADER.split(',')[:-1])
        text = f'half samples=1\n{HEADER}\n{payload},{crc32_msb(payload.encode()):08x}'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'half.log'
            path.write_text(text, encoding='utf-8')
            rows, rejected = read_frames([path], HEADER, 'half')
            self.assertEqual((len(rows), rejected), (1, 0))
            with self.assertRaises(ValueError):
                read_frames([path], HEADER, 'pair')

    def test_untriggered_requires_explicit_baseline(self):
        row = self.row()
        row['flags'] = 0
        with self.assertRaises(ValueError):
            analyze([row], [2048] * 3, 0.1)
        result = analyze([row], [2048] * 3, 0.1, allow_untriggered=True)
        self.assertEqual(result['time_reference'], 'first_sample_no_event')
        self.assertEqual(result['samples'][0]['relative_ms'], 0)

    def test_common_mode_is_rejected_by_clarke(self):
        result = difference_statistics([[1, 1, 1], [-1, -1, -1]], 0.1)
        self.assertAlmostEqual(result['common_energy_fraction'], 1)
        self.assertAlmostEqual(result['differential_energy_fraction'], 0)
        self.assertAlmostEqual(result['rms_clarke_delta_vector_A'], 0)
        self.assertEqual(result['fixed_mean_energy_fraction'], 0)

    def test_zero_sum_and_fixed_bias(self):
        result = difference_statistics([[1, -1, 0], [1, -1, 0]], 0.1)
        self.assertAlmostEqual(result['common_energy_fraction'], 0)
        self.assertAlmostEqual(result['differential_energy_fraction'], 1)
        self.assertAlmostEqual(result['fixed_mean_energy_fraction'], 1)
        self.assertAlmostEqual(result['centered_rms_clarke_delta_vector_A'], 0)
        self.assertIsNone(result['raw_delta_correlations']['0_1'])

    def test_energy_identity_and_degenerate(self):
        result = difference_statistics([[1, 2, 3], [-2, 1, -1], [0, 1, 2]], 0.1)
        self.assertAlmostEqual(result['common_energy_fraction'] + result['differential_energy_fraction'], 1)
        self.assertLess(result['fixed_mean_energy_fraction'], 1)
        zero = difference_statistics([[0, 0, 0]], 0.1)
        self.assertIsNone(zero['common_energy_fraction'])
        self.assertEqual(difference_statistics([], 0.1)['count'], 0)


if __name__ == '__main__':
    unittest.main()
