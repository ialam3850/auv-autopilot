import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from simulator import GRAVITY, SensorConfig, Simulator


class SimulatorTest(unittest.TestCase):
    def test_stationary_vehicle_reads_gravity(self):
        quiet = SensorConfig(gyro_noise=0, gyro_bias_sigma=0, gyro_bias_walk=0, accel_noise=0, depth_noise=0)
        sim = Simulator(quiet, initial_depth=5.0)
        sim.step([0, 0, 0], 0.02)
        gyro, accel, depth = sim.read_sensors()
        np.testing.assert_allclose(accel, [0, 0, -GRAVITY], atol=0.05)
        np.testing.assert_allclose(gyro, [0, 0, 0], atol=1e-6)
        self.assertAlmostEqual(depth, 5.0, delta=0.02)

    def test_pitched_vehicle_reads_tilted_gravity(self):
        quiet = SensorConfig(gyro_noise=0, gyro_bias_sigma=0, gyro_bias_walk=0, accel_noise=0, depth_noise=0)
        theta = np.radians(10)
        sim = Simulator(quiet, initial_pitch=theta)
        sim.step([0, 0, 0], 0.001)  # tiny step: attitude barely changes
        _, accel, _ = sim.read_sensors()
        np.testing.assert_allclose(accel[0], GRAVITY * np.sin(theta), atol=0.05)
        np.testing.assert_allclose(accel[2], -GRAVITY * np.cos(theta), atol=0.05)

    def test_depth_is_quantised(self):
        sim = Simulator(seed=4, initial_depth=3.0)
        for _ in range(20):
            _, _, depth = sim.read_sensors()
            self.assertAlmostEqual(depth * 100, round(depth * 100), places=6)

    def test_propeller_drives_vehicle_forward(self):
        sim = Simulator(seed=1)
        for _ in range(int(30 / 0.02)):
            sim.step([0, 0, 1200], 0.02)
        self.assertGreater(sim.nu[0], 1.5)  # ~2 m/s at 1200 rpm

    def test_stern_plane_changes_pitch(self):
        # Sign convention check used by the autopilot: positive stern plane -> nose down.
        sim = Simulator(seed=1)
        for _ in range(int(20 / 0.02)):
            sim.step([0, 0, 1200], 0.02)
        for _ in range(int(3 / 0.02)):
            sim.step([0, np.radians(10), 1200], 0.02)
        self.assertLess(sim.eta[4], 0.0)


if __name__ == "__main__":
    unittest.main()
