import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from analyze_handoff import HEADER, analyze, analyze_currents, analyze_phase_motion, read_frames
from compact_trace import crc32_msb


class HandoffTest(unittest.TestCase):
    def frame(self, index=0, stamp=1000, delta=0.02):
        row = {key: 0 for key in HEADER.split(',')[:-1]}
        row.update(seq=index, time_us=stamp, flags=1 if index == 0 else 2, delta=delta,
                   old_iq=0.5, iq_cmd=0.3, kp=0.12, vd_before=0.8, vq_before=1.1)
        row['vd_transport'] = math.cos(delta) * 0.8 + math.sin(delta) * 1.1 + 0.12 * math.sin(delta) * 0.5
        row['vq_transport'] = -math.sin(delta) * 0.8 + math.cos(delta) * 1.1 + 0.12 * (math.cos(delta) * 0.5 - 0.3)
        payload = ','.join(str(row[key]) for key in HEADER.split(',')[:-1])
        return payload + f',{crc32_msb(payload.encode()):08x}'

    def read(self, *texts):
        with tempfile.TemporaryDirectory() as directory:
            paths = []
            for index, content in enumerate(texts):
                path = Path(directory) / f'{index}.log'
                path.write_text(content, encoding='utf-8')
                paths.append(path)
            return read_frames(paths)

    def text(self, *frames, count=2):
        return f'handoff samples={count}\n{HEADER}\n' + '\n'.join(frames)

    def test_round_trip_and_transport(self):
        rows, bad = self.read(self.text(self.frame(), self.frame(1, 2000)))
        self.assertEqual(bad, 0)
        self.assertLess(analyze(rows)['max_transport_residual_V'], 1e-12)

    def test_recover_only_crc_valid_frames(self):
        first = self.frame()
        second = self.frame(1, 2000)
        rows, bad = self.read(self.text(first, second[:-1] + 'x'), self.text(first, second))
        self.assertEqual((len(rows), bad), (2, 1))
        with self.assertRaises(ValueError):
            self.read(self.text(first))

    def test_refuse_changed_or_unordered_buffer(self):
        with self.assertRaises(ValueError):
            self.read(self.text(self.frame(), self.frame(1, 2000)),
                      self.text(self.frame(delta=0.03), self.frame(1, 2000)))
        with self.assertRaises(ValueError):
            self.read(self.text(self.frame(), self.frame(1, 999)))

    def test_wrap_and_missing_trigger(self):
        rows, _ = self.read(self.text(self.frame(stamp=2**32 - 10), self.frame(1, 10)))
        self.assertEqual(len(rows), 2)
        rows[0]['flags'] = 0
        with self.assertRaises(ValueError):
            analyze(rows)

    def test_current_mapping_and_park(self):
        row = dict(adc0=2058, adc1=2045, adc2=2042, ia=1.0, ib=-0.6, ic=-0.3,
                   theta=0.4, time_us=1000, flags=1)
        alpha, beta = (2.0 + 0.6 + 0.3) / 3, (-0.6 + 0.3) / math.sqrt(3)
        row['id'] = alpha * math.cos(0.4) + beta * math.sin(0.4)
        row['iq'] = -alpha * math.sin(0.4) + beta * math.cos(0.4)
        result = analyze_currents([row], [2048, 2048, 2048], 0.1)
        self.assertLess(result['raw_to_abc_max_error_A'], 1e-12)
        self.assertLess(result['abc_to_dq_max_error_A'], 1e-12)
        row['adc1'], row['adc2'] = row['adc2'], row['adc1']
        self.assertGreater(analyze_currents([row], [2048] * 3, 0.1)['raw_to_abc_max_error_A'], 0.29)
        row['id'] += 0.1
        self.assertGreater(analyze_currents([row], [2048] * 3, 0.1)['abc_to_dq_max_error_A'], 0.09)
        with self.assertRaises(ValueError):
            analyze_currents([row], [2048] * 3, 0)

    def test_phase_motion_wrap(self):
        rows = [dict(flags=1, time_us=2**32 - 500, theta=3.1),
                dict(flags=2, time_us=500, theta=3.54 - 2 * math.pi)]
        result = analyze_phase_motion(rows, 7)
        self.assertEqual(result['intervals'], 1)
        self.assertAlmostEqual(result['equivalent_rpm_time_mean'], 0.44 * 60e3 / (2 * math.pi * 7))
        self.assertEqual(result['span_ms'], 1)
        with self.assertRaises(ValueError):
            analyze_phase_motion(rows, 0)
        rows[1]['flags'] = 0
        with self.assertRaises(ValueError):
            analyze_phase_motion(rows, 7)


if __name__ == '__main__':
    unittest.main()
