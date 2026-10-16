from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from analyze_timing_evidence import analyze


class TimingEvidenceTest(unittest.TestCase):
    def row(self):
        return dict(prev_count=3000, count=3500, prev_period=5600, period=5600,
                    prev_cr1=161, cr1=161, prev_entry_us=2**32-30, entry_us=37)

    def test_up_and_wrap(self):
        result = analyze([self.row()], 168e6)
        self.assertEqual(result['entry_gap_histogram_us'], {67: 1})
        self.assertEqual(result['callback_down_frames'], 0)
        self.assertAlmostEqual(result['nominal_next_boundary_range_us'][0], 12.5)
        self.assertAlmostEqual(result['conditional_gap_minus_full_cycle_range_us'][0], 1/3)

    def test_down_direction(self):
        row = self.row()
        row['cr1'] |= 16
        result = analyze([row], 168e6)
        self.assertEqual(result['paired_callback_direction_changes'], 1)
        self.assertEqual(result['callback_down_frames'], 1)

    def test_invalid(self):
        for clock in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                analyze([self.row()], clock)
        row = self.row()
        row['count'] = 5601
        with self.assertRaises(ValueError):
            analyze([row], 168e6)


if __name__ == '__main__':
    unittest.main()
