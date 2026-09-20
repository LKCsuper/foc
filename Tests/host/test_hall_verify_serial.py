"""Hall 串口验证脚本的停机判定。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "Scripts"))

from hall_verify_serial import HallRunner, Step, UnsafeSample, parse_current_diag  # noqa: E402
from ol_verify_serial import Abort  # noqa: E402


class HallSafetyTest(unittest.TestCase):
    def setUp(self):
        self.runner = HallRunner(None, 1, True)
        self.snap = {
            "filtered": 150.0, "id": 0.0, "iq": 0.3, "id_target": 0.0,
            "abc_sum": 0.0, "v_bus": 24.0, "valid": True, "state": 2,
            "age_ms": 1.0,
        }

    def test_abnormal_current_always_aborts_keep_going(self):
        for change in ({"iq": 3.0}, {"v_bus": 21.0}):
            with self.subTest(change=change):
                snap = {**self.snap, **change}
                with self.assertRaises(UnsafeSample) as caught:
                    self.runner.check_speed("150", snap, 150, 0.25)
                with self.assertRaises(UnsafeSample):
                    self.runner.handle_soft_fail(caught.exception, "150")

    def test_phase_sum_and_id_tracking_only_warn(self):
        self.runner.check_speed("150", {**self.snap, "abc_sum": 1.6, "id": 1.133}, 150, 0.25)
        self.assertFalse(self.runner.failed)
        warn_names = [name for name, _ in self.runner.sections if name.endswith("-CURRENT-WARN")]
        self.assertGreaterEqual(len(warn_names), 2)

    def test_speed_error_can_continue(self):
        self.runner.handle_soft_fail(Abort("speed outside tolerance"), "150")
        self.assertTrue(self.runner.failed)

    def test_idle_noise_range_only_warns(self):
        self.runner.check_speed("150", {**self.snap, "abc_sum": 1.176}, 150, 0.25)
        self.assertFalse(self.runner.failed)

    def test_aggregated_id_error_only_warns(self):
        report = (
            "Hall current diag (1/16 control samples)\n"
            "state samples sum_over sum_max id_over id_max\n"
            "1 1468 117 0.853 456 1.565\n"
            "2 1785 117 0.920 427 1.999\n"
            "3 1546 133 1.094 27 1.638\n"
            "4 1595 115 0.933 0 0.885\n"
            "5 1786 159 1.001 98 1.483\n"
            "6 1472 110 1.001 35 1.134\n"
            "total samples: 9652\n"
        )
        self.assertEqual(sum(row[3] for row in parse_current_diag(report).values()), 1043)

        class FakeShell:
            def run(self, command, timeout):
                assert command == "olcurdiag"
                return report

        runner = HallRunner(FakeShell(), 1, True)
        runner.take_snap = lambda name: self.snap
        runner.check_speed = lambda name, snap, rpm, tolerance: None
        runner.sample_stage(Step(kind="set", rpm=1800, tolerance=0.02, name="up1800"))
        self.assertFalse(runner.failed)
        self.assertTrue(any("samples |Id error|" in text for _, text in runner.sections))

    def test_aggregated_phase_sum_peak_only_warns(self):
        report = (
            "Hall current diag (1/16 control samples)\n"
            "state samples sum_over sum_max id_over id_max\n"
            "1 100 10 1.820 0 0.500\n"
            "2 100 10 0.920 0 0.500\n"
            "3 100 10 0.920 0 0.500\n"
            "4 100 10 0.920 0 0.500\n"
            "5 100 10 0.920 0 0.500\n"
            "6 100 10 0.920 0 0.500\n"
            "phase state first_samples first_id_over second_samples second_id_over\n"
            "phase 1 50 0 50 0\n"
            "total samples: 600\n"
        )

        class FakeShell:
            def run(self, command, timeout):
                assert command == "olcurdiag"
                return report

        runner = HallRunner(FakeShell(), 1, True)
        runner.take_snap = lambda name: self.snap
        runner.check_speed = lambda name, snap, rpm, tolerance: None
        runner.sample_stage(Step(kind="set", rpm=150, tolerance=0.25, name="up150"))
        self.assertFalse(runner.failed)
        self.assertTrue(any("1.820 A" in text for _, text in runner.sections))

    def test_late_fault_after_stop_fails_run(self):
        class FakeShell:
            def stop(self):
                return "Open-loop stopped\n"

            def run(self, command, timeout):
                if command == "olcurdiag":
                    return "Hall current diag\n"
                if command == "olsnap":
                    return "Enable     : OFF\nTarget RPM : 0\nFault      : 4 ABS_OVER_CURRENT (LATCHED)\n"
                if command == "olfault":
                    return "Fault = 4 ABS_OVER_CURRENT (LATCHED)\n"
                raise AssertionError(command)

        self.assertEqual(HallRunner(FakeShell(), 1, False).run([]), 2)


if __name__ == "__main__":
    unittest.main()
