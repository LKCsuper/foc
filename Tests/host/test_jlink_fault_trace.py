import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from compact_trace import SnapshotChangedError
from fault_trace import parse_export
from jlink_fault_trace import (DESCRIPTOR, FIELDS, SAMPLE, check_ram, decode_ring,
                               descriptor_address, parse_descriptor, read_frozen, read_script)


def header(**changes):
    info = dict(zip(FIELDS, (0x464f4344, 1, 6, 2, 0x20001000, 84, 512, 4, 3, 3, 4, 66, 198, 56)))
    info.update(changes)
    return DESCRIPTOR.pack(*(info[field] for field in FIELDS))


def ring(head=3, stamps=(66, 132, 198), invalid=None):
    data = bytearray(512 * SAMPLE.size)
    for index, stamp in enumerate(stamps):
        floats = [0.0] * 17
        floats[-2:] = [24.0, 0.000066]
        adc = 2045
        if invalid == 'nan':
            floats[0] = math.nan
        if invalid == 'adc':
            adc = 4096
        sample = (stamp, 4 << 16 if index == 2 else 0, adc, 2045, 2045, 0, *floats)
        SAMPLE.pack_into(data, ((head - 3 + index) % 512) * SAMPLE.size, *sample)
    return bytes(data)


class JlinkFaultTests(unittest.TestCase):
    def test_abi_and_map(self):
        self.assertEqual(SAMPLE.size, 84)
        self.assertEqual(DESCRIPTOR.size, 56)
        self.assertEqual(descriptor_address(' g_motor_fault_debug 0x20000100 Data 56 motor_timing.o\n'), 0x20000100)
        with self.assertRaises(ValueError):
            descriptor_address(' g_motor_fault_debug 0x08000100 Data 56 motor_timing.o\n')
        with self.assertRaises(ValueError):
            descriptor_address('')

    def test_complete_export_and_crc_compatibility(self):
        pending = iter([header(), ring(), header(), ring(), header()])
        requests = []
        def reader(address, size):
            requests.append((address, size))
            return next(pending)
        info, raw, data, csv = read_frozen(reader, 0x20000100)
        self.assertEqual(info['count'], 3)
        self.assertEqual(raw, header())
        self.assertEqual(data, ring())
        count, rows, rejected = parse_export('trace samples=3\ntrace format=fault stride=4;\n' + csv)
        self.assertEqual((count, len(rows), rejected), (3, 3, 0))
        self.assertEqual(requests, [(0x20000100, 56), (0x20001000, 43008), (0x20000100, 56),
                                    (0x20001000, 43008), (0x20000100, 56)])

    def test_wrap_and_time_rollover(self):
        stamps = (0xffffff00, 0xffffff80, 0)
        info = parse_descriptor(header(head=1, first_us=stamps[0], last_us=0))
        csv = decode_ring(info, ring(head=1, stamps=stamps))
        self.assertIn('0,4294967040,', csv)
        self.assertIn('2,0,', csv)

    def test_reject_active_odd_empty_and_bad_geometry(self):
        for changes in ({'state': 1}, {'sequence': 7}, {'count': 0}, {'head': 512},
                        {'count': 513}, {'sample_bytes': 88}, {'version': 2},
                        {'address': 0x2001fff0}, {'address': 0x20001001}, {'fault': 65536}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                parse_descriptor(header(**changes))
        with self.assertRaises(ValueError):
            parse_descriptor(header()[:-1])

    def test_active_refused_before_payload_read(self):
        reads = []
        def reader(address, size):
            reads.append(size)
            return header(state=1)
        with self.assertRaises(ValueError):
            read_frozen(reader, 0x20000100)
        self.assertEqual(reads, [56])

    def test_descriptor_and_data_changes(self):
        for values in ([header(), ring(), header(sequence=8)],
                       [header(), ring(), header(), ring(invalid='adc'), header()],
                       [header(), ring(), header(), ring(), header(sequence=8)]):
            pending = iter(values)
            with self.assertRaises(SnapshotChangedError):
                read_frozen(lambda address, size: next(pending), 0x20000100)

    def test_reject_invalid_samples(self):
        info = parse_descriptor(header())
        for data in (ring()[:-1], ring(invalid='nan'), ring(invalid='adc'), ring(stamps=(66, 66, 198))):
            with self.assertRaises(ValueError):
                decode_ring(info, data)
        with self.assertRaises(ValueError):
            decode_ring(parse_descriptor(header(fault=5)), ring())

    def test_read_commands_only(self):
        script = read_script(Path('dump.bin'), 0x20000100, 56)
        self.assertEqual([line.split()[0] for line in script.splitlines()], ['connect', 'savebin', 'q'])
        self.assertIn('0x20000100 0x38', script)
        for address, size in ((0x08000000, 56), (0x20000001, 56), (0x2001fffc, 8), (0x20000000, 0)):
            with self.assertRaises(ValueError):
                check_ram(address, size)
        with self.assertRaises(ValueError):
            read_script(Path('bad"file.bin'), 0x20000100, 56)


if __name__ == '__main__':
    unittest.main()
