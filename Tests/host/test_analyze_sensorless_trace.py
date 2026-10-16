import importlib.util
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("trace", Path("Scripts/analyze_sensorless_trace.py"))
trace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trace)


class TraceTest(unittest.TestCase):
    def test_flux_phase_relation(self):
        rows = []
        angle = 3.1
        for index in range(12):
            angle += 0.10 + 0.02 * index
            rows.append(dict(time_us=(2**32 - 500 + index * 1000) % 2**32,
                             pll=(angle + 3.141592653589793) % 6.283185307179586 - 3.141592653589793,
                             blend=1.0, flux=0.003 - index * 0.0001, rpm=600.0))
        result = trace.flux_phase_relation(rows, 7)
        self.assertEqual(result['intervals'], 11)
        self.assertLess(result['correlation_abs_angle_rate_vs_flux'], -0.99)
        self.assertGreater(result['correlation_abs_angle_rate_vs_inverse_flux_squared'], 0.9)
        self.assertEqual(result['reported_filtered_rpm']['mean'], 600)
        for row in rows:
            row['flux'] = 0.002
        self.assertIsNone(trace.flux_phase_relation(rows, 7)['correlation_abs_angle_rate_vs_flux'])
        with self.assertRaises(ValueError):
            trace.flux_phase_relation(rows, 0)
        for row in rows:
            row['blend'] = 0.5
        self.assertEqual(trace.flux_phase_relation(rows, 7)['intervals'], 0)

    def analyze(self, content):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.log"
            path.write_text(content, encoding="utf-8")
            return trace.analyze(path)

    def test_lite_wrap_and_loss_event(self):
        text = "time_us,flags,theta,pll,flux,rpm,id,iq,id_cmd,iq_cmd,blend,loss,va,vb,gap_max_us,full_max_us\n"
        for stamp, flags in ((4294967000, 270), (237, 398), (770, 398)):
            text += f"{stamp},{flags},3.1,-3.1,.002,600,.1,.7,0,.5,.2,0,1,0,67,55\n"
        out = self.analyze(text)
        self.assertEqual(out["format"], "lite")
        self.assertEqual(out["loss_events"], 1)
        self.assertEqual(out["record_interval_us"]["median"], 533)
        self.assertAlmostEqual(out["iq_tracking_error_A"]["mean"], .2)
        self.assertAlmostEqual(out["entry_span_ms"], 1.066)
        self.assertFalse(out["independent_rotor_reference"])

    def test_bad_rows_and_no_entry(self):
        text = "time_us,flags,theta,pll,flux,rpm,id,iq,blend,iq_cmd,enc\n"
        text += "100,14,0,0,.002,600,0,.5,0,.5,0\n"
        text += "101,14,0,0,nan,600,0,.5,0,.5,0\n"
        text += "90,14,0,0,.002,600,0,.5,0,.5,0\n"
        out = self.analyze(text)
        self.assertEqual(out["rows"], 1)
        self.assertEqual(out["malformed"], 2)
        self.assertIsNone(out["entry_us"])
        self.assertIsNone(out["record_interval_us"])
        self.assertEqual(out["format"], "full")

    def test_mixed_formats_rejected(self):
        with self.assertRaises(ValueError):
            self.analyze("time_us,flags,theta,pll,flux,rpm,id,iq,blend,iq_cmd\n"
                         "time_us,flags,theta,pll,flux,rpm,id,iq,blend,iq_cmd,enc\n")

    def test_flight_windows_and_damaged_count(self):
        text = "trace flight entry_rows=1 rolling_rows=1\n"
        text += "time_us,flags,theta,pll,flux,rpm,id,iq,blend,iq_cmd\n"
        text += "100,270,0,0,.002,600,.1,.5,1,.5\n"
        text += "10000,398,0,0,.003,1600,1,.4,1,.5\n"
        out = self.analyze(text)["flight_windows"]
        self.assertEqual(out["initial"]["metrics"]["rpm"]["mean"], 600)
        self.assertEqual(out["rolling"]["last"]["rpm"], 1600)
        self.assertEqual(out["gap_us"], 9900)
        with self.assertRaises(ValueError):
            self.analyze(text.replace("rolling_rows=1", "rolling_rows=2"))

    def test_speed_task_evidence(self):
        text = "time_us,flags,theta,pll,flux,rpm,id,iq,blend,iq_cmd,task_count,task_age_us,task_gate,speed_target,speed_feedback,speed_iq\n"
        text += "100,270,0,0,.002,600,0,.5,1,.5,10,20,1,600,600,.5\n"
        text += "1000,270,0,0,.002,900,0,.5,1,.5,10,920,1,600,600,.5\n"
        out = self.analyze(text)["speed_task"]
        self.assertEqual(out["distinct_counts"], 1)
        self.assertEqual(out["age_us"]["max"], 920)
        self.assertEqual(out["feedback_error_RPM"]["mean"], 0)


if __name__ == "__main__":
    unittest.main()
