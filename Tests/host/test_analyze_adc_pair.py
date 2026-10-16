import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from analyze_adc_pair import FRAME, HEADER, analyze
from analyze_handoff import read_frames
from compact_trace import crc32_msb


class PairTest(unittest.TestCase):
    def row(self):
        row = dict.fromkeys(HEADER.split(',')[:-1], 0)
        row.update(flags=1, time_us=100)
        for prefix in ('prev_', ''):
            row.update({prefix + key: value for key, value in zip(FRAME.split(','),
                [2**32 - 1, 2**32 - 1, 2**32 - 30, 0, 0, 0, 2048, 2048, 2048, 2800, 2800, 2800, 3000, 16, 0, 5600])})
        row.update(sample=0, pwm=0, entry_us=37, adc1=2049, adc2=2050, ib=0.2, ic=0.1)
        return row

    def test_mapping_wrap_and_difference(self):
        result = analyze([self.row()], [2048] * 3, 0.1)
        self.assertAlmostEqual(result['conversion_max_error_A'], 0)
        self.assertEqual(result['paired_interval_range_us'], [67, 67])
        self.assertEqual(result['software_pwm_sequence_deltas'], [1])
        self.assertEqual(result['largest_delta']['delta_adc_counts'], [0, 1, 2])
        self.assertAlmostEqual(result['largest_delta']['delta_zero_sequence_A'], 0.1)

    def test_reconstruction_and_invalid_frame(self):
        row = self.row()
        row.update(recon=1, ia=-0.3)
        self.assertAlmostEqual(analyze([row], [2048] * 3, 0.1)['conversion_max_error_A'], 0)
        row['prev_adc0'] = 4096
        with self.assertRaises(ValueError):
            analyze([row], [2048] * 3, 0.1)

    def test_crc(self):
        row = self.row()
        payload = ','.join(str(row[key]) for key in HEADER.split(',')[:-1])
        text = f'pair samples=1\n{HEADER}\n{payload},{crc32_msb(payload.encode()):08x}'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pair.log'
            path.write_text(text, encoding='utf-8')
            rows, rejected = read_frames([path], HEADER, 'pair')
            self.assertEqual((len(rows), rejected), (1, 0))
            path.write_text(text[:-1] + 'x', encoding='utf-8')
            with self.assertRaises(ValueError):
                read_frames([path], HEADER, 'pair')


if __name__ == '__main__':
    unittest.main()
