import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from sensorless_observe_test import observation_collapsed, validate_plan, validate_voltage_limit, validate_voltage_schedule, voltage_near_limit


class ObservePlanTest(unittest.TestCase):
    def test_staged_plan(self):
        validate_plan([300, 800, 1200, 1800, 2400, 3000], 3000, 10)
        validate_plan([300, 600], 800, 8)

    def test_reject_unsafe_plan(self):
        for rpms, maximum, samples in [([3000], 3000, 10), ([300, 3000], 3000, 10),
                                        ([300, 800], 600, 10), ([300], 5001, 10),
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
        self.assertFalse(voltage_near_limit('Fault : 0 NONE\nVd, Vq out: (|V|=3.600 / 4.000 V)'))
        self.assertTrue(voltage_near_limit('Fault : 0 NONE\nVd, Vq out: (|V|=3.920 / 4.000 V)'))
        self.assertTrue(voltage_near_limit('Fault : 0 NONE\nVd, Vq out: (|V|=4.000 / 4.000 V)'))
        for reply in ('missing', 'Fault : 0 NONE\n(|V|=0.000 / 0.000 V)',
                      'Fault : 4 ABS_OVER_CURRENT\n(|V|=0.000 / 6.500 V)',
                      '(|V|=0.000 / 6.500 V)'):
            with self.assertRaises(RuntimeError):
                voltage_near_limit(reply)

    def test_temporary_voltage_bounds(self):
        for value in (None, 4, 4.5):
            validate_voltage_limit(value)
        for value in (0, -1, 4.6, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                validate_voltage_limit(value)

    def test_voltage_schedule(self):
        rpms = [300, 800, 1200, 1800, 2300, 2800, 3300, 3800, 4300, 4700, 5000]
        validate_plan(rpms, 5000, 8)
        validate_voltage_schedule(rpms, [4.5, 4.5, 4.5, 4.5, 5, 5.5, 6.5, 7.5, 8.5, 9.5, 10])
        for limits in ([4.5], [4.5, float('nan')], [4.5, 10.1], [5, 6], [4.5, 3], [4.5, 7]):
            with self.assertRaises(ValueError):
                validate_voltage_schedule([300, 800], limits)


if __name__ == '__main__':
    unittest.main()
