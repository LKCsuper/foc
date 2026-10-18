import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from compact_trace import SnapshotChangedError, crc32_msb
from fault_trace import HEADER, export_fault_trace, parse_export


def row(sequence, stamp, adc=2045, dt='0.000066'):
    fields = [str(sequence), str(stamp), '0', str(adc), '2045', '2045'] + ['0'] * 16 + [dt]
    payload = ','.join(fields)
    return f'{payload},{crc32_msb(payload.encode()):08x}'


def reply(rows, count=3):
    return f'trace samples={count}\ntrace format=fault stride=4;\n{HEADER}\n' + '\n'.join(rows)


class FaultTraceTests(unittest.TestCase):
    def runner(self, replies, status='Enable : OFF\nTarget RPM : 0\n'):
        pending = iter(replies)
        def run(command, timeout):
            self.assertIn(command, ('olstatus', 'sltrace'))
            return status if command == 'olstatus' else next(pending)
        return run

    def test_retry_complementary_damage(self):
        frames = [row(index, 100 + index * 267) for index in range(3)]
        csv, report = export_fault_trace(self.runner([
            reply([frames[0], frames[1][:-1] + 'x']), reply([frames[0], frames[1], frames[2]])]))
        self.assertEqual(report, {'rows': 3, 'attempts': 2, 'rejected_rows': 1})
        self.assertEqual(csv.splitlines(), [HEADER] + frames)

    def test_conflict_and_count_change(self):
        for second in (reply([row(0, 100, 2046), row(1, 367), row(2, 634)]),
                       reply([row(0, 100)], 2), reply([row(1, 367), row(2, 634)])):
            with self.assertRaises(SnapshotChangedError):
                export_fault_trace(self.runner([reply([row(0, 100)]), second]))

    def test_reject_nonfinite_and_adc_range(self):
        for frame in (row(0, 100, 4096), row(0, 100, dt='nan'), row(0, 100, dt='-1')):
            count, rows, rejected = parse_export(reply([frame]))
            self.assertEqual((count, rows, rejected), (3, {}, 1))

    def test_stopped_guard(self):
        with self.assertRaises(RuntimeError):
            export_fault_trace(self.runner([], 'Enable : ON\nTarget RPM : 300\n'))

    def test_missing_never_interpolated(self):
        with self.assertRaises(RuntimeError):
            export_fault_trace(self.runner([reply([row(0, 100)])] * 3))

    def test_time_order_and_wrap(self):
        frames = [row(0, 0xffffff00), row(1, 11), row(2, 278)]
        csv, _ = export_fault_trace(self.runner([reply(frames)]))
        self.assertIn(frames[-1], csv)
        with self.assertRaises(SnapshotChangedError):
            export_fault_trace(self.runner([reply([row(0, 100), row(1, 100), row(2, 200)])]))

    def test_reject_format_and_empty(self):
        for text in ('missing', reply([], 0), reply([row(0, 100)]).replace('stride=4', 'stride=1')):
            with self.assertRaises(ValueError):
                parse_export(text)


if __name__ == '__main__':
    unittest.main()
