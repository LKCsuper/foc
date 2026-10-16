import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from compact_trace import HEADER, crc32_msb, export_compact_trace, validate_compact_trace


def frame(sequence, stamp, flags=2):
    payload = f'{sequence},{stamp},{flags},0.1,600.0,0.0,0.2,0.0,0.2'
    return f'{payload},{crc32_msb(payload.encode()):08x}'


def dump(rows):
    return '\n'.join([f'trace samples={len(rows)}', HEADER, *rows, '[T+1s] $ '])


class CompactTraceTests(unittest.TestCase):
    def test_known_crc_and_empty(self):
        self.assertEqual(crc32_msb(b'123456789'), 0x89a1897f)
        self.assertEqual(validate_compact_trace(dump([])), [])

    def test_timestamp_wrap(self):
        rows = validate_compact_trace(dump([frame(0, 0xfffffff0), frame(1, 20, 2048)]))
        self.assertEqual(rows[-1]['flags'], 2048)

    def test_corruption(self):
        valid = dump([frame(0, 100), frame(1, 200)])
        damaged = [valid.replace('600.0', '601.0', 1), valid.replace(',600.0,', ',600.0', 1),
                   dump([frame(0, 100), frame(2, 200)]), dump([frame(0, 100), frame(1, 100)]),
                   valid.replace(frame(1, 200), ''), valid.replace('trace samples=2', 'trace samples=3')]
        for text in damaged:
            with self.subTest(text=text), self.assertRaises(ValueError):
                validate_compact_trace(text)

    def test_retry_is_bounded(self):
        valid = dump([frame(0, 100)])
        replies = iter([valid.replace('600.0', '601.0'), valid])
        calls = []
        def run(command, timeout):
            calls.append((command, timeout))
            return next(replies)
        self.assertEqual(len(export_compact_trace(run)), 1)
        self.assertEqual(len(calls), 2)
        timeout_replies = iter([RuntimeError('UART response incomplete'), valid])
        def retry_timeout(*_):
            reply = next(timeout_replies)
            if isinstance(reply, Exception):
                raise reply
            return reply
        self.assertEqual(len(export_compact_trace(retry_timeout)), 1)
        with self.assertRaises(ValueError):
            export_compact_trace(lambda *_: 'broken', attempts=2)

    def test_merge_only_verified_frames(self):
        first, second = frame(0, 100), frame(1, 200)
        replies = iter([dump([first, second.replace('600.0', '601.0')]),
                        dump([first.replace('600.0', '601.0'), second])])
        rows = export_compact_trace(lambda *_: next(replies))
        self.assertEqual([row['csv'] for row in rows], [first, second])

    def test_refuse_changed_snapshot(self):
        replies = iter([dump([frame(0, 100), frame(1, 200).replace('600.0', '601.0')]),
                        dump([frame(0, 101), frame(1, 200)])])
        with self.assertRaisesRegex(ValueError, 'frame changed'):
            export_compact_trace(lambda *_: next(replies))


if __name__ == '__main__':
    unittest.main()
