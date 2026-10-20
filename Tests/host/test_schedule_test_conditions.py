import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from schedule_test_conditions import validate_blocked_if_reply


class ScheduleConditionTests(unittest.TestCase):
    def test_stopped_observer_is_expected(self):
        result = validate_blocked_if_reply('rpm obs=0.0 meas=0.0 tgt=300 iq=1.021 OPEN_LOOP tk=0 fb=invalid fault=0')
        self.assertFalse(result['shaft_speed_verified'])
        self.assertEqual(result['observer_rpm'], 0)

    def test_reject_unsafe_or_incomplete(self):
        base = 'rpm obs=0.0 tgt=300 iq=1.0 tk=0 fault=0'
        for reply in ('', base.replace('tgt=300', 'tgt=3300'), base.replace('iq=1.0', 'iq=3.1'),
                      base.replace('tk=0', 'tk=1'), base.replace('fault=0', 'fault=10'),
                      base.replace('obs=0.0', 'obs=1201.0'), base.replace('iq=1.0', 'iq=nan')):
            with self.subTest(reply=reply), self.assertRaises(ValueError):
                validate_blocked_if_reply(reply)


if __name__ == '__main__':
    unittest.main()
