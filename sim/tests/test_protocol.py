import os
import random
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from protocol import (Actuators, Command, FrameReceiver, Sensors, cobs_decode, cobs_encode, crc16_ccitt,
                      make_frame)


class ProtocolTest(unittest.TestCase):
    def test_crc_known_vector(self):
        # Same check value the C++ test uses.
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_cobs_known_vectors(self):
        self.assertEqual(cobs_encode(b"\x00"), b"\x01\x01")
        self.assertEqual(cobs_encode(b"\x11\x22\x00\x33"), b"\x03\x11\x22\x02\x33")

    def test_cobs_round_trip_random(self):
        rng = random.Random(3)
        for _ in range(300):
            n = rng.randrange(0, 600)
            data = bytes(0 if rng.random() < 0.25 else rng.randrange(256) for _ in range(n))
            enc = cobs_encode(data)
            self.assertNotIn(0, enc)
            self.assertEqual(cobs_decode(enc), data)

    def test_message_sizes_match_cpp(self):
        self.assertEqual(len(Sensors(1, 2, (0, 0, 0), (0, 0, -9.81), 1.0).pack()), 36)
        self.assertEqual(len(Command(True, 10.0, 1200.0).pack()), 12)

    def test_frame_receiver_resyncs_after_corruption(self):
        good = make_frame(Command(True, 12.5, 1200).pack())
        bad = bytearray(good)
        bad[4] ^= 0x10
        rx = FrameReceiver()
        got = list(rx.feed(b"\x05\x06\x00" + bytes(bad) + good))
        self.assertEqual(len(got), 1)
        self.assertEqual(struct.unpack("<BBHff", got[0])[3], 12.5)
        self.assertGreaterEqual(rx.crc_errors + rx.framing_errors, 1)

    def test_overflow_is_bounded_and_receiver_resyncs(self):
        rx = FrameReceiver()
        self.assertEqual(list(rx.feed(b"\x01" * 100000)), [])
        self.assertLessEqual(len(rx._buf), 69)
        good = make_frame(Command(True, 10.0, 1200.0).pack())
        self.assertEqual(len(list(rx.feed(b"\x00" + good))), 1)
        self.assertEqual(rx.framing_errors, 1)
        for payload in [b"", bytes(65)]:
            with self.assertRaises(ValueError):
                make_frame(payload)

    def test_actuators_unpack(self):
        payload = struct.pack("<BBH3f4f3f3fI", ord("A"), 1, 7, 0.1, 0.0, 1200.0, *([0.0] * 10), 1234)
        a = Actuators.unpack(payload)
        self.assertEqual(a.seq, 7)
        self.assertAlmostEqual(a.stern_plane, 0.1, places=6)
        self.assertEqual(a.compute_cycles, 1234)


if __name__ == "__main__":
    unittest.main()
