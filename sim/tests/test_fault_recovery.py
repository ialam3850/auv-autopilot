"""Fault recovery through the real TCP runner and binary protocol."""
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from hil_bridge import TcpLink
from protocol import Actuators, Command, FrameReceiver, Sensors, make_frame

RUNNER = Path(__file__).resolve().parents[2] / "build" / "sil_runner"


@unittest.skipUnless(RUNNER.exists(), "build/sil_runner not built")
class FaultRecoveryTest(unittest.TestCase):
    def test_bad_numeric_and_old_timestamp_do_not_poison_estimator(self):
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        proc = subprocess.Popen([str(RUNNER), str(port), "--once"], stderr=subprocess.DEVNULL)
        link = None
        try:
            link = TcpLink("127.0.0.1", port)
            link.write(make_frame(Command(True, 10.0, 1200.0).pack()))
            rx = FrameReceiver()
            for seq, stamp, ax, enabled in [(1, 0, 0.0, 1), (2, 20000, 0.0, 1),
                                            (3, 40000, math.nan, 0), (4, 60000, 0.0, 1),
                                            (5, 20000, 0.0, 0), (6, 80000, 0.0, 1)]:
                link.write(make_frame(Sensors(seq, stamp, (0, 0, 0), (ax, 0, -9.81), 0).pack()))
                reply = None
                for _ in range(10):
                    for payload in rx.feed(link.read(1.0)):
                        if len(payload) == 60:
                            reply = Actuators.unpack(payload)
                    if reply is not None:
                        break
                self.assertIsNotNone(reply)
                self.assertEqual(reply.enabled, enabled)
                self.assertTrue(math.isfinite(reply.pitch))
        finally:
            if link is not None:
                link.close()
            proc.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
