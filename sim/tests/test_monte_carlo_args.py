import os
from pathlib import Path
import subprocess
import sys
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "monte_carlo.py"


class MonteCarloArgsTest(unittest.TestCase):
    def test_bad_run_counts_and_noise_fail_before_starting_workers(self):
        for args in [["--runs", "0"], ["--workers", "0"], ["--gyro-noise", "nan"],
                     ["--gyro-noise", "-1"]]:
            p = subprocess.run([sys.executable, str(SCRIPT), *args],
                               capture_output=True, text=True, timeout=10)
            self.assertEqual(p.returncode, 2)
            self.assertIn("must be positive", p.stderr)


if __name__ == "__main__":
    unittest.main()
