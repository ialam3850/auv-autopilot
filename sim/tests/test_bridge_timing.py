import math
import os
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import hil_bridge as hb
from protocol import FrameReceiver, make_frame
from simulator import Simulator


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, seconds)


class DelayedLink:
    def __init__(self, clock, delay):
        self.clock, self.delay = clock, delay
        self.seq = 0

    def write(self, data):
        for payload in FrameReceiver().feed(data):
            if payload[0] == ord("S"):
                self.seq = struct.unpack_from("<H", payload, 2)[0]

    def read(self, timeout):
        self.clock.now += self.delay
        return make_frame(struct.pack("<BBH3f4f3f3fI", ord("A"), 1, self.seq,
                                      0.0, 0.0, 0.0, *([0.0] * 10), 1))


class BridgeTimingTest(unittest.TestCase):
    def fly(self, delay, **kwargs):
        clock = Clock()
        with patch.object(hb, "time", clock):
            return hb.run(Simulator(), DelayedLink(clock, delay),
                          hb.Mission(duration=0.1), realtime=True, **kwargs)

    def test_thirty_ms_reply_misses_twenty_ms_period(self):
        _, stats = self.fly(0.030)
        self.assertEqual(stats["late"], 5)

    def test_fast_reply_is_accepted(self):
        _, stats = self.fly(0.005)
        self.assertEqual(stats["late"], 0)

    def test_loose_timeout_cannot_extend_period(self):
        _, stats = self.fly(0.030, reply_timeout=0.05)
        self.assertEqual(stats["late"], 5)

    def test_reports_round_trip_and_period_overruns(self):
        _, stats = self.fly(0.030)
        self.assertEqual(stats["period_overruns"], 5)
        self.assertAlmostEqual(stats["reply_ms_max"], 30.0)
        self.assertEqual(stats["timing_mode"], "realtime")

    def test_invalid_period_is_rejected(self):
        for dt in [0, -1, math.nan]:
            with self.assertRaises(ValueError):
                hb.run(Simulator(), None, hb.Mission(duration=0.1), dt=dt)


if __name__ == "__main__":
    unittest.main()
