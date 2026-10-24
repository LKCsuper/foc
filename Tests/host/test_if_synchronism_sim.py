import sys
import unittest
import math
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Scripts'))
from simulate_if_synchronism import Config, advance, equilibrium, stability, trajectory
from simulate_current_loop import Config as ElectricalConfig


class IfSynchronismTests(unittest.TestCase):
    def test_equilibrium_and_delay_pipelines(self):
        for sample_delay in (0, 1):
            for output_delay in (0, 1):
                config = Config(sample_delay=sample_delay, output_delay=output_delay)
                state, electric, load = equilibrium(config)
                result = advance(state, config, electric, load)
                for before, after in zip(state, result):
                    self.assertAlmostEqual(before, after, places=9)

    def test_local_linearization_and_substep_convergence(self):
        for config in (Config(), Config(inertia=1e-6, friction=0, equilibrium_angle=1, output_delay=1)):
            first = stability(config)
            for other in (stability(config, .5), stability(replace(config, substeps=16))):
                self.assertAlmostEqual(first['growth_per_s'], other['growth_per_s'], delta=.002)
                self.assertAlmostEqual(first['oscillation_hz'], other['oscillation_hz'], delta=.002)

    def test_zero_perturbation_preserves_equilibrium(self):
        config = Config()
        state, _, _ = equilibrium(config)
        result = trajectory(config, duration=.01, angle_perturbation=0)
        self.assertLess(result['max_angle_deviation_rad'], 1e-10)
        self.assertAlmostEqual(result['final_rpm'], state[3] * 60 / 6.283185307179586, places=7)

    def test_ideal_and_stationary_holds_are_distinct(self):
        ideal, _, _ = equilibrium(Config(stationary_hold=False))
        stationary, _, _ = equilibrium(Config(stationary_hold=True))
        self.assertGreater(abs(ideal[4] - stationary[4]) + abs(ideal[5] - stationary[5]), .01)

    def test_predicted_mode_grows_from_small_perturbation(self):
        config = Config(inertia=1e-6, friction=0, equilibrium_angle=1, output_delay=1)
        result = trajectory(config, duration=.05, modal_perturbation=True)
        self.assertLess(result['initial_current_deviation_a'], .1)
        self.assertGreater(result['max_angle_deviation_rad'], .0051)

    def test_invalid_equilibrium(self):
        for config in (Config(inertia=0), Config(friction=-1), Config(equilibrium_angle=2),
                       Config(sample_delay=2), Config(voltage_limit=1), Config(rpm=float('nan')),
                       Config(output_fraction=-.1), Config(output_fraction=1), Config(output_fraction=float('nan'))):
            with self.assertRaises(ValueError):
                equilibrium(config)

    def test_fractional_delay_equilibrium_and_convergence(self):
        for sample_delay in (0, 1):
            for output_delay in (0, 1):
                config = Config(sample_delay=sample_delay, output_delay=output_delay, output_fraction=.5)
                state, electric, load = equilibrium(config)
                self.assertEqual(len(state), 8 + 2 * sample_delay + 2 * output_delay)
                result = advance(state, config, electric, load)
                for before, after in zip(state, result):
                    self.assertAlmostEqual(before, after, places=9)
                first = stability(config)
                second = stability(replace(config, substeps=16))
                self.assertAlmostEqual(first['growth_per_s'], second['growth_per_s'], delta=.002)

    def test_fractional_delay_queue_order(self):
        for output_delay in (0, 1):
            config = Config(output_delay=output_delay, output_fraction=.5)
            state, electric, load = equilibrium(config)
            previous = list(state)
            previous[6:8] = (2.0, 3.0)
            if output_delay:
                previous[8:10] = (4.0, 5.0)
            result = advance(tuple(previous), config, electric, load)
            if output_delay:
                self.assertEqual(result[-2:], (2.0, 3.0))
            self.assertEqual(len(result), len(previous))

    def test_fractional_switch_matches_analytic_rl_response(self):
        electric = ElectricalConfig(flux=0, pole_pairs=0, kp=0, ki=0)
        for fraction in (.25, .5, .75):
            config = Config(rpm=0, stationary_hold=False, output_fraction=fraction, substeps=16)
            state = (0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 0.0)
            result = advance(state, config, electric, 0.0)
            active_time = (1 - fraction) / electric.frequency
            expected = 2 / electric.resistance * (1 - math.exp(-electric.resistance * active_time / electric.inductance))
            self.assertAlmostEqual(result[0], expected, places=8)
            self.assertAlmostEqual(result[1], 0.0, places=12)


if __name__ == '__main__':
    unittest.main()
