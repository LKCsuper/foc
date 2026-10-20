import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from compact_trace import SnapshotChangedError
from jlink_schedule import SNAPSHOT, decode_snapshot, read_snapshot, snapshot_address


def snapshot(**changes):
    words = [0x464f4353, 3, 2, 2, 11] + list(range(5, 70))
    words[64] = 0
    for index, value in changes.items():
        words[int(index)] = value
    return SNAPSHOT.pack(*words)


class ScheduleTests(unittest.TestCase):
    def test_layout(self):
        report = decode_snapshot(snapshot())
        self.assertEqual(report['task']['pi_max_gap_us'], 9)
        self.assertEqual(report['phase_max_us']['wait'], 13)
        self.assertEqual(report['pi_gap_bins']['gt2000'], 22)
        self.assertEqual(report['worst_pi_wake']['dma_us'], 26)
        self.assertEqual(report['worst_wake']['delay_us'], 28)
        self.assertEqual(report['dma']['count'], 32)
        self.assertEqual(report['systick']['switch_pending'], 46)
        self.assertEqual(report['task_gap_bins']['gt2000'], 52)
        self.assertEqual(report['task_gap_summary']['intervals'], sum(range(47, 53)))
        self.assertEqual(report['task_gap_summary']['over_1ms'], sum(range(48, 53)))
        self.assertEqual(report['worst_task_gap']['sequence'], 53)
        self.assertEqual(report['worst_task_gap']['wake']['dma_us'], 59)
        self.assertEqual(report['worst_task_gap']['previous_wake']['delay_us'], 61)
        self.assertEqual(report['worst_task_gap']['phase_us']['completion_snapshot'], 69)
        wrapped = decode_snapshot(snapshot(**{'54': 10, '55': 0xfffffff0}))
        self.assertEqual(wrapped['worst_task_gap']['gap_us'], 26)

    def test_map(self):
        self.assertEqual(snapshot_address(' g_motor_schedule_debug 0x20000100 Data 280 motor_timing.o\n'),
                         0x20000100)
        for text in ('', ' g_motor_schedule_debug 0x08000100 Data 280 motor_timing.o\n',
                     ' g_motor_schedule_debug 0x20000100 Data 256 motor_timing.o\n',
                     ' g_motor_schedule_debug 0x20000100 Data 188 motor_timing.o\n',
                     ' g_motor_schedule_debug 0x20000100 Data 184 motor_timing.o\n'):
            with self.assertRaises(ValueError):
                snapshot_address(text)

    def test_reject_invalid(self):
        for changes in ({'0': 0}, {'1': 1}, {'2': 3}, {'3': 0}, {'4': 16}):
            with self.assertRaises(ValueError):
                decode_snapshot(snapshot(**changes))
        with self.assertRaises(ValueError):
            decode_snapshot(b'')

    def test_double_read(self):
        calls = []
        def reader(address, size):
            calls.append((address, size))
            return snapshot()
        raw, report = read_snapshot(reader, 0x20000100)
        self.assertEqual(raw, snapshot())
        self.assertEqual(report['sequence'], 2)
        self.assertEqual(calls, [(0x20000100, 280)] * 2)
        pending = iter([snapshot(), snapshot(**{'2': 4})])
        with self.assertRaises(SnapshotChangedError):
            read_snapshot(lambda address, size: next(pending), 0x20000100)

    def test_unfrozen_rejected_before_second_read(self):
        calls = []
        def reader(address, size):
            calls.append(address)
            return snapshot(**{'3': 0})
        with self.assertRaises(ValueError):
            read_snapshot(reader, 0x20000100)
        self.assertEqual(len(calls), 1)

    def test_empty_distribution(self):
        raw = snapshot(**{str(index): 0 for index in range(5, 70)})
        report = decode_snapshot(raw)
        self.assertEqual(report['task_gap_summary']['intervals'], 0)
        self.assertIsNone(report['task_gap_summary']['over_1ms_percent'])
        self.assertFalse(report['worst_task_gap']['captured'])

    def test_event_phase_sum(self):
        changes = {'54': 2250, '55': 1000, '64': 1, '65': 10,
                   '66': 990, '67': 100, '68': 100, '69': 50}
        report = decode_snapshot(snapshot(**changes))
        self.assertTrue(report['worst_task_gap']['phase_valid'])
        self.assertEqual(report['worst_task_gap']['gap_us'], 1250)
        changes['69'] = 51
        with self.assertRaises(ValueError):
            decode_snapshot(snapshot(**changes))


if __name__ == '__main__':
    unittest.main()
