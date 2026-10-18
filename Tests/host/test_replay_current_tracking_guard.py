import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from replay_current_tracking_guard import replay


def rows(errors, stamps=None):
    if stamps is None:
        stamps = [index * 267 for index in range(len(errors))]
    return [dict(seq=index, time_us=stamp, prev_id=error, prev_iq=1,
                 prev_id_cmd=0, prev_iq_cmd=1) for index, (stamp, error) in enumerate(zip(stamps, errors))]


class ReplayGuardTests(unittest.TestCase):
    def test_hold_and_boundary(self):
        result = replay(rows([2] * 10))
        self.assertEqual(result['first_candidate']['seq'], 8)
        self.assertEqual(result['first_candidate']['observed_span_us'], 2136)

    def test_reset_on_recovery(self):
        self.assertIsNone(replay(rows([3] * 7 + [0] + [3] * 7))['first_candidate'])

    def test_missing_samples_do_not_bridge(self):
        self.assertIsNone(replay(rows([3] * 8, [0, 267, 534, 801, 5000, 5267, 5534, 5801]))['first_candidate'])

    def test_wrap_and_invalid(self):
        wrapped = rows([3] * 10, [(0xffffff00 + index * 267) & 0xffffffff for index in range(10)])
        self.assertIsNotNone(replay(wrapped)['first_candidate'])
        with self.assertRaises(ValueError):
            replay(rows([1, 1], [100, 100]))
        with self.assertRaises(ValueError):
            replay(rows([math.nan]))
        with self.assertRaises(ValueError):
            replay(rows([1]), threshold=-1)


if __name__ == '__main__':
    unittest.main()
