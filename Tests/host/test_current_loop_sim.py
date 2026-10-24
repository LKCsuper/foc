import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from simulate_current_loop import Config, phase_peak, pi_step, simulate


class CurrentLoopSimulationTests(unittest.TestCase):
    def test_pi_update_and_antiwindup(self):
        integrals, output = pi_step((0, 0), (0, 0), Config(), 1 / 15000)
        self.assertAlmostEqual(integrals[1], 18 / 15000)
        self.assertAlmostEqual(output[1], .12 + 18 / 15000)
        integrals, output = pi_step((100, 100), (-100, -100), Config(), 1 / 15000)
        self.assertAlmostEqual(math.hypot(*output), 8.5)
        self.assertAlmostEqual(integrals[0] + 100 * .12, output[0])

    def test_equilibrium_with_delays(self):
        for coupling in (False, True):
            for delay in (0, 1, 2):
                result = simulate(Config(coupling=coupling, sample_delay=delay, output_delay=delay))
                self.assertAlmostEqual(result['final_id_a'], 0, places=8)
                self.assertAlmostEqual(result['final_iq_a'], 1, places=8)
                self.assertIsNone(result['overcurrent_ms'])

    def test_integration_convergence(self):
        first = simulate(Config(angle_change=-0.5, substeps=8))
        second = simulate(Config(angle_change=-0.5, substeps=16))
        self.assertAlmostEqual(first['phase_peak_a'], second['phase_peak_a'], places=6)
        self.assertAlmostEqual(first['final_id_a'], second['final_id_a'], places=6)

    def test_balanced_phase_peak(self):
        self.assertEqual(phase_peak((1, 0), 0), 1)
        self.assertAlmostEqual(phase_peak((0, 1), 0), math.sqrt(3) / 2)

    def test_small_current_perturbation_does_not_run_away(self):
        for delay in (0, 1, 2):
            result = simulate(Config(initial_id=0.05, sample_delay=delay, output_delay=delay))
            self.assertLess(result['phase_peak_a'], 1.1)
            self.assertLess(abs(result['final_id_a']), 0.002)
            self.assertIsNone(result['overcurrent_ms'])

    def test_invalid_configuration(self):
        for config in (Config(inductance=0), Config(sample_delay=-1), Config(output_delay=.5),
                       Config(substeps=0), Config(rpm=float('nan')), Config(voltage_limit=1)):
            with self.assertRaises(ValueError):
                simulate(config)


if __name__ == '__main__':
    unittest.main()
