import cmath
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from reconstruct_fault_emf import derivative, load_rows, reconstruct


def synthetic_rows(current_dq=.3 + .6j):
    rows = []
    omega = 2550.0
    for index in range(512):
        time_us = index * 267
        theta = omega * time_us * 1e-6
        current = current_dq * cmath.exp(1j * theta)
        emf = 5 * cmath.exp(1j * (theta + .4 + math.pi / 2))
        voltage = .474 * current + .000112 * 1j * omega * current + emf
        rows.append({'seq': index, 'time_us': time_us,
                     'ia': current.real, 'ib': -.5 * current.real + math.sqrt(3) / 2 * current.imag,
                     'ic': -.5 * current.real - math.sqrt(3) / 2 * current.imag,
                     'prev_theta': math.remainder(theta, math.tau),
                     'prev_pll': math.remainder(theta + .4, math.tau),
                     'prev_v_alpha': voltage.real, 'prev_v_beta': voltage.imag})
    return rows


class EmfReconstructionTests(unittest.TestCase):
    def test_nonuniform_derivative(self):
        times = [0, .2, .5]
        self.assertAlmostEqual(derivative([time * time for time in times], times, 1), .4)

    def test_known_rl_and_rotor_angle(self):
        for window in (1, 3, 5):
            result = reconstruct(synthetic_rows(), window=window)
            for row in result:
                self.assertAlmostEqual(row['emf_v'], 5, places=8)
                self.assertAlmostEqual(row['emf_relative'], .4, places=8)
                self.assertAlmostEqual(row['emf_drift'], 0, places=8)

    def test_voltage_shift_phase(self):
        for shift in (-1 / 15000, 0, 1 / 15000):
            result = reconstruct(synthetic_rows(0j), voltage_shift=shift)
            for row in result:
                self.assertAlmostEqual(row['emf_relative'], .4 - 2550 * shift, places=8)

    def test_reject_filter_and_timestamp_errors(self):
        with self.assertRaises(ValueError):
            reconstruct(synthetic_rows(), method='direct', window=3)
        rows = synthetic_rows()
        rows[1]['time_us'] = rows[0]['time_us']
        with self.assertRaises(ValueError):
            reconstruct(rows)

    def test_reject_bad_crc(self):
        path = Mock()
        path.read_text.return_value = 'seq,time_us,crc32\n0,0,00000000\n'
        with self.assertRaises(ValueError):
            load_rows(path)


if __name__ == '__main__':
    unittest.main()
