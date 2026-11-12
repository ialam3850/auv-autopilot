import os
import socket
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import hil_bridge as hb
from protocol import make_frame
from simulator import Simulator
from test_bridge_timing import Clock, DelayedLink


class BadThenGoodLink(DelayedLink):
    def read(self, timeout):
        good = super().read(timeout)
        bad = struct.pack("<BBH3f4f3f3fI", ord("A"), 1, self.seq,
                          float("nan"), 0.0, 1200.0, *([0.0] * 10), 1)
        return make_frame(bad) + good


class LinkValidationTest(unittest.TestCase):
    def test_bad_numeric_reply_is_ignored_and_good_reply_is_used(self):
        clock = Clock()
        with patch.object(hb, "time", clock):
            _, stats = hb.run(Simulator(), BadThenGoodLink(clock, 0.001),
                              hb.Mission(duration=0.1), realtime=True)
        self.assertEqual(stats["invalid_replies"], 5)
        self.assertEqual(stats["late"], 0)

    def test_closed_tcp_socket_is_reported(self):
        left, right = socket.socketpair()
        link = object.__new__(hb.TcpLink)
        link.sock = left
        right.close()
        try:
            with self.assertRaises(ConnectionError):
                link.read(0.05)
        finally:
            link.close()


if __name__ == "__main__":
    unittest.main()
