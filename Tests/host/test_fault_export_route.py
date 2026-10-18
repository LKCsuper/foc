import subprocess
import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from sensorless_observe_test import export_stopped_fault, main


class FaultExportRouteTests(unittest.TestCase):
    def test_missing_jlink_rejected_before_serial_connection(self):
        with patch('sys.argv', ['observe', '--fault-trace']), \
                patch('sensorless_observe_test.shutil.which', return_value=None), \
                patch('sensorless_observe_test.Shell') as shell, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                main()
            self.assertEqual(error.exception.code, 2)
            shell.assert_not_called()

    def test_jlink_default_route_freezes_without_uart_rows(self):
        run = Mock()
        with patch('sensorless_observe_test.subprocess.run') as capture, \
                patch('sensorless_observe_test.export_fault_trace') as uart:
            export_stopped_fault(run, Path('test.log'), 'jlink', 'JLink.exe')
        run.assert_called_once_with('sltrace debug')
        uart.assert_not_called()
        arguments = capture.call_args.args[0]
        self.assertIn('JLink.exe', arguments)
        self.assertEqual(arguments[-1], 'test_fault_jlink')
        self.assertEqual(capture.call_args.kwargs, {'check': True, 'timeout': 180})

    def test_export_error_is_not_silently_reported_as_success(self):
        with patch('sensorless_observe_test.subprocess.run', side_effect=subprocess.CalledProcessError(1, 'jlink')):
            with self.assertRaises(subprocess.CalledProcessError):
                export_stopped_fault(Mock(), Path('test.log'), 'jlink', 'JLink.exe')

    def test_uart_explicit_fallback(self):
        parent = Path(__file__).resolve().parents[2] / '.build' / 'host'
        parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as folder:
            path = Path(folder) / 'test.log'
            with patch('sensorless_observe_test.export_fault_trace', return_value=('csv\n', {'rows': 1})), \
                    patch('sensorless_observe_test.subprocess.run') as capture:
                export_stopped_fault(Mock(), path, 'uart', None)
            capture.assert_not_called()
            self.assertEqual((Path(folder) / 'test_fault.csv').read_text(), 'csv\n')


if __name__ == '__main__':
    unittest.main()
