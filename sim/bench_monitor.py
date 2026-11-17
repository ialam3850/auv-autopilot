#!/usr/bin/env python3
"""Watch the board's attitude estimate live (firmware built with AP_FW_MODE=BENCH).

In BENCH mode the NUCLEO reads the real ICM-42688-P at 50 Hz, runs the same
autopilot code, and streams Actuators messages. Tilt the board by hand and
the roll/pitch printed here should follow.

  python3 sim/bench_monitor.py /dev/ttyACM0            # print 5 lines/s
  python3 sim/bench_monitor.py /dev/ttyACM0 --log bench.csv --seconds 60

Checks worth doing (see docs/HARDWARE.md):
  * board flat on the desk: roll and pitch within about 1 deg of zero
  * nose up (USB end up) by ~30 deg: pitch about +30
  * leave it still for a minute: bias_q settles and pitch doesn't drift
  * compute_cycles / 84 = microseconds per control step
"""

import argparse
import csv
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from protocol import Actuators, FrameReceiver  # noqa: E402

FIELDS = ["t_host", "seq", "roll_deg", "pitch_deg", "bias_p", "bias_q", "sigma_pitch_deg", "compute_us"]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("port", help="serial port, e.g. /dev/ttyACM0 or COM5")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl-C)")
    p.add_argument("--log", help="write every sample to this CSV")
    args = p.parse_args()

    import serial

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer()
    rx = FrameReceiver()
    writer = None
    if args.log:
        f = open(args.log, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(FIELDS)

    t0 = time.monotonic()
    last_print = 0.0
    last_seq = None
    dropped = 0
    n = 0
    try:
        while args.seconds <= 0 or time.monotonic() - t0 < args.seconds:
            for payload in rx.feed(ser.read(max(1, ser.in_waiting))):
                try:
                    a = Actuators.unpack(payload)
                except Exception:
                    continue  # not an Actuators frame
                if last_seq is not None:
                    dropped += (a.seq - last_seq - 1) & 0xFFFF
                last_seq = a.seq
                n += 1
                now = time.monotonic() - t0
                row = [f"{now:.3f}", a.seq, f"{math.degrees(a.roll):.2f}", f"{math.degrees(a.pitch):.2f}",
                       f"{a.bias_p:.5f}", f"{a.bias_q:.5f}", f"{math.degrees(math.sqrt(max(a.p_pitch, 0))):.3f}",
                       f"{a.compute_cycles / 84.0:.1f}"]
                if writer:
                    writer.writerow(row)
                if now - last_print >= 0.2:
                    last_print = now
                    print(f"roll {row[2]:>7s}  pitch {row[3]:>7s} deg   bias_q {row[5]:>9s} rad/s   "
                          f"step {row[7]:>6s} us   dropped {dropped}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    print(f"\n{n} samples, {dropped} dropped, CRC errors {rx.crc_errors}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
