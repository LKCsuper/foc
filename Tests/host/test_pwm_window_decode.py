import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from analyze_pwm_window import decode_window


def window(stamps, enabled=0, direction=0):
    raw = bytearray(struct.pack('<II', enabled, len(stamps)) + bytes(32 * 28))
    for index, stamp in enumerate(stamps):
        struct.pack_into('<7I', raw, 8 + 28 * index, stamp, (stamp + 1) & 0xffffffff,
                         direction, 3800, 3840, direction, 5600)
    return raw


class PwmWindowDecodeTests(unittest.TestCase):
    def test_dense_and_sparse(self):
        for interval in (67, 66467):
            rows, report = decode_window(window([100 + interval * index for index in range(32)]))
            self.assertEqual(report['interval_us_range'], [interval, interval])
            self.assertEqual(report['span_us'], interval * 31 + 1)
            self.assertEqual(report['ambiguous_frames'], 0)
            self.assertAlmostEqual(rows[0]['after_to_boundary_us'], 1760 / 168)

    def test_wrap_and_partial(self):
        rows, report = decode_window(window([0xfffffff0, 0x32]))
        self.assertEqual(report['interval_us_range'], [66, 66])
        self.assertEqual(report['span_us'], 67)
        _, report = decode_window(window([100]))
        self.assertIsNone(report['interval_us_range'])

    def test_reject_invalid_or_active(self):
        for raw in (b'', window([]), window([100], enabled=1), window([100], enabled=2), window([100, 100])):
            with self.assertRaises(ValueError):
                decode_window(raw)
        for frequency in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                decode_window(window([100]), frequency)

    def test_ambiguous_direction_or_long_elapsed(self):
        raw = window([100])
        struct.pack_into('<I', raw, 8 + 5 * 4, 16)
        rows, report = decode_window(raw)
        self.assertEqual(report['ambiguous_frames'], 1)
        self.assertIsNone(rows[0]['after_to_boundary_us'])
        raw = window([100])
        struct.pack_into('<I', raw, 8 + 4, 200)
        self.assertEqual(decode_window(raw)[1]['ambiguous_frames'], 1)

    def test_between_frame_direction_change_is_not_hidden(self):
        raw = window([100, 167])
        struct.pack_into('<7I', raw, 8 + 28, 167, 168, 16, 1800, 1760, 16, 5600)
        _, report = decode_window(raw)
        self.assertEqual(report['up_frames'], 1)
        self.assertEqual(report['direction_change_rows'], [1])
        self.assertEqual(report['ambiguous_frames'], 0)


if __name__ == '__main__':
    unittest.main()
