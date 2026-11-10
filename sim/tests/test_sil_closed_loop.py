"""Closed-loop test: the real autopilot code (host build) flying the simulated
vehicle through the framed protocol. Skipped if build/sil_runner isn't built."""

import os
import socket
import subprocess
import sys
import unittest

import numpy as np

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))

from hil_bridge import LOG_FIELDS, Mission, TcpLink, run
from simulator import Simulator

RUNNER = os.path.join(HERE, "..", "..", "build", "sil_runner")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@unittest.skipUnless(os.path.exists(RUNNER), "build/sil_runner not built")
class SilClosedLoopTest(unittest.TestCase):
    def fly(self, mission, **sim_kwargs):
        port = free_port()
        proc = subprocess.Popen([RUNNER, str(port), "--once"], stderr=subprocess.DEVNULL)
        try:
            link = TcpLink("127.0.0.1", port)
            rows, stats = run(Simulator(**sim_kwargs), link, mission)
            link.close()
        finally:
            proc.wait(timeout=10)
        return np.array(rows, dtype=float), stats

    def test_reaches_and_holds_depth(self):
        mission = Mission(setpoints=[(0.0, 0.0), (5.0, 10.0)], duration=150.0)
        a, stats = self.fly(mission, seed=11)
        col = {n: i for i, n in enumerate(LOG_FIELDS)}
        self.assertEqual(stats["late"], 0)
        self.assertEqual(stats["crc_errors"], 0)
        last = a[a[:, col["t"]] > 120.0]
        self.assertLess(np.max(np.abs(last[:, col["z"]] - 10.0)), 0.5)
        # Attitude estimate stays within a degree of truth throughout.
        err = np.degrees(np.abs(a[:, col["est_theta"]] - a[:, col["theta"]]))
        self.assertLess(np.nanmax(err[a[:, col["t"]] > 10]), 1.0)

    def test_holds_depth_in_a_current(self):
        mission = Mission(setpoints=[(0.0, 0.0), (5.0, 15.0)], duration=180.0)
        a, stats = self.fly(mission, seed=12, current_speed=0.3, current_dir_deg=90.0)
        col = {n: i for i, n in enumerate(LOG_FIELDS)}
        last = a[a[:, col["t"]] > 150.0]
        self.assertLess(np.max(np.abs(last[:, col["z"]] - 15.0)), 0.5)


if __name__ == "__main__":
    unittest.main()
