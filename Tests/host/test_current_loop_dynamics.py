from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from check_current_loop_dynamics import KI, KP, LIMIT, pi_step, poles, simulate


class CurrentLoopDynamicsTests(unittest.TestCase):
    def test_pi_update_and_clamp(self):
        voltage, integral = pi_step(1j, 0j, 0j, 0.001)
        self.assertAlmostEqual(integral.imag, KI * 0.001)
        self.assertAlmostEqual(voltage.imag, KI * 0.001 + KP)
        voltage, integral = pi_step(100 + 100j, 0j, 0j, 0.001)
        self.assertAlmostEqual(abs(voltage), LIMIT)
        self.assertAlmostEqual(abs(integral + KP * (100 + 100j) - voltage), 0)

    def test_nominal_poles_and_equilibrium(self):
        self.assertTrue(all(value.real < 0 for value in poles()))
        for lag in (0, 1):
            run = simulate(-5.0 + 0.3j, lag=lag)
            self.assertAlmostEqual(run['peak_current_vector_A'], 1)
            self.assertLess(run['final_target_error_A'], 1e-10)

    def test_disturbance_recovery_and_convergence(self):
        for case in ('current_step', 'phase_step'):
            run = simulate(-5.0 + 0.3j, case, lag=1)
            self.assertLess(run['final_target_error_A'], 0.001)
        first = simulate(-5.0 + 0.3j, 'slip_50', substeps=8)
        second = simulate(-5.0 + 0.3j, 'slip_50', substeps=16)
        self.assertLess(abs(first['peak_current_vector_A'] - second['peak_current_vector_A']), 0.01)


if __name__ == '__main__':
    unittest.main()
