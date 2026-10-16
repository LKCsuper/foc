import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from sensorless_observe_test import observation_collapsed, validate_plan, voltage_near_limit


class ObservePlanTest(unittest.TestCase):
    def test_staged_plan(self):
        validate_plan([300, 800, 1200, 1800, 2400, 3000], 3000, 10)
        validate_plan([300, 600], 800, 8)

    def test_reject_unsafe_plan(self):
        for rpms, maximum, samples in [([3000], 3000, 10), ([300, 3000], 3000, 10),
                                        ([300, 800], 600, 10), ([300], 3001, 10),
                                        ([300, 200], 3000, 10), ([], 800, 10), ([300], 800, 3)]:
            with self.assertRaises(ValueError):
                validate_plan(rpms, maximum, samples)

    def test_collapse_during_ramp(self):
        self.assertTrue(observation_collapsed(2275.3, 12.2))
        self.assertTrue(observation_collapsed(1000, -1))
        self.assertFalse(observation_collapsed(None, 0))
        self.assertFalse(observation_collapsed(300, 110))
        self.assertFalse(observation_collapsed(2400, 2200))

    def test_voltage_limit(self):
        self.assertFalse(voltage_near_limit('Vd, Vq out: (|V|=3.600 / 4.000 V)'))
        self.assertTrue(voltage_near_limit('Vd, Vq out: (|V|=3.920 / 4.000 V)'))
        self.assertTrue(voltage_near_limit('Vd, Vq out: (|V|=4.000 / 4.000 V)'))
        for reply in ('missing', '(|V|=0.000 / 0.000 V)'):
            with self.assertRaises(RuntimeError):
                voltage_near_limit(reply)


if __name__ == '__main__':
    unittest.main()
