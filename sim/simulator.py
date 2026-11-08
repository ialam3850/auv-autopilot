"""Truth simulation: REMUS-100 dynamics plus the sensors the autopilot sees.

The vehicle model is Fossen's (vendored in sim/vehicle). This module adds
what a hardware-in-the-loop setup needs on top of it:

* an IMU model: gyro with a constant-plus-random-walk bias and white noise,
  accelerometer measuring specific force (not just gravity) with noise
* a depth sensor model: noise and 1 cm quantisation, like an MS5837
* fixed-step integration with sub-stepping, so the control rate and the
  physics rate are independent
"""

import math
from dataclasses import dataclass

import numpy as np

from vehicle import Remus100, Rzyx, attitudeEuler

GRAVITY = 9.81


@dataclass
class SensorConfig:
    gyro_noise: float = 0.002  # rad/s per sample (1 sigma)
    gyro_bias_sigma: float = 0.01  # rad/s, initial bias spread (1 sigma)
    gyro_bias_walk: float = 1e-4  # rad/s/sqrt(s)
    accel_noise: float = 0.08  # m/s^2 per sample
    depth_noise: float = 0.02  # m
    depth_resolution: float = 0.01  # m


@dataclass
class Truth:
    t: float
    eta: np.ndarray  # x, y, z, phi, theta, psi
    nu: np.ndarray  # u, v, w, p, q, r
    gyro_bias: np.ndarray


class Simulator:
    def __init__(self, sensors: SensorConfig = SensorConfig(), current_speed=0.0, current_dir_deg=0.0,
                 initial_depth=0.0, initial_pitch=0.0, seed=0, physics_dt=0.005):
        self.rng = np.random.default_rng(seed)
        self.cfg = sensors
        self.vehicle = Remus100(V_current=current_speed, beta_current=current_dir_deg)
        self.eta = np.array([0, 0, initial_depth, 0, initial_pitch, 0], float)
        self.nu = np.zeros(6)
        self.u_actual = np.zeros(3)
        self.t = 0.0
        self.physics_dt = physics_dt
        self.gyro_bias = self.rng.normal(0.0, sensors.gyro_bias_sigma, 3)
        self._last_linear_accel = np.zeros(3)

    def truth(self) -> Truth:
        return Truth(self.t, self.eta.copy(), self.nu.copy(), self.gyro_bias.copy())

    def step(self, u_control, dt):
        """Advance by dt seconds with the given [rudder, stern plane, rpm] command."""
        n = max(1, int(round(dt / self.physics_dt)))
        h = dt / n
        nu_before = self.nu.copy()
        for _ in range(n):
            self.nu, self.u_actual = self.vehicle.dynamics(self.eta, self.nu, self.u_actual,
                                                           np.asarray(u_control, float), h)
            self.eta = attitudeEuler(self.eta, self.nu, h)
        self.t += dt
        # Linear acceleration of the body origin, in body axes: v_dot + omega x v.
        v_dot = (self.nu[0:3] - nu_before[0:3]) / dt
        self._last_linear_accel = v_dot + np.cross(self.nu[3:6], self.nu[0:3])
        self.gyro_bias += self.rng.normal(0.0, self.cfg.gyro_bias_walk * math.sqrt(dt), 3)

    def read_sensors(self):
        """Returns (gyro[3], accel[3], depth) as the IMU and depth sensor would report them."""
        cfg = self.cfg
        gyro = self.nu[3:6] + self.gyro_bias + self.rng.normal(0.0, cfg.gyro_noise, 3)

        phi, theta, psi = self.eta[3:6]
        R = Rzyx(phi, theta, psi)  # body -> NED
        g_body = R.T @ np.array([0.0, 0.0, GRAVITY])
        specific_force = self._last_linear_accel - g_body
        accel = specific_force + self.rng.normal(0.0, cfg.accel_noise, 3)

        depth = self.eta[2] + self.rng.normal(0.0, cfg.depth_noise)
        depth = round(depth / cfg.depth_resolution) * cfg.depth_resolution
        return gyro, accel, depth
