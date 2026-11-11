#!/usr/bin/env python3
"""Run one depth mission against an autopilot and plot it.

  # software in the loop (start build/sil_runner first, or use scripts/run_sil.sh)
  python3 sim/run_mission.py --link tcp:127.0.0.1:5760

  # hardware in the loop: NUCLEO-F446RE on its ST-LINK USB serial port
  python3 sim/run_mission.py --link serial:/dev/ttyACM0 --realtime
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

from hil_bridge import LOG_FIELDS, Mission, open_link, run
from simulator import Simulator


def summarize(rows):
    a = np.array(rows, dtype=float)
    col = {name: i for i, name in enumerate(LOG_FIELDS)}
    t, z, z_cmd = a[:, col["t"]], a[:, col["z"]], a[:, col["z_cmd"]]
    after = t > 10.0
    track_rms = math.sqrt(np.nanmean((z[after] - z_cmd[after]) ** 2))
    theta_err = np.degrees(a[:, col["est_theta"]] - a[:, col["theta"]])
    phi_err = np.degrees(a[:, col["est_phi"]] - a[:, col["phi"]])
    compute = a[:, col["compute"]]
    return {
        "depth tracking RMS vs prefiltered command (m)": track_rms,
        "pitch estimate RMS error (deg)": math.sqrt(np.nanmean(theta_err[after] ** 2)),
        "roll estimate RMS error (deg)": math.sqrt(np.nanmean(phi_err[after] ** 2)),
        "max |stern plane| (deg)": float(np.degrees(np.nanmax(np.abs(a[:, col["stern_plane"]])))),
        "median autopilot compute (raw units)": float(np.median(compute[compute > 0])) if np.any(compute > 0) else 0,
    }


def plot(rows, path, title):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a = np.array(rows, dtype=float)
    col = {name: i for i, name in enumerate(LOG_FIELDS)}
    t = a[:, col["t"]]
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    ax[0].plot(t, a[:, col["z_setpoint"]], "k--", lw=1, label="setpoint")
    ax[0].plot(t, a[:, col["z_cmd"]], color="0.6", lw=1, label="prefiltered command")
    ax[0].plot(t, a[:, col["z"]], color="C0", lw=1.5, label="true depth")
    ax[0].invert_yaxis()
    ax[0].set_ylabel("depth (m)")
    ax[0].legend(loc="lower right", fontsize=8)
    ax[1].plot(t, np.degrees(a[:, col["theta"]]), color="C0", lw=1.5, label="true pitch")
    ax[1].plot(t, np.degrees(a[:, col["est_theta"]]), color="C1", lw=1, label="EKF pitch")
    ax[1].plot(t, np.degrees(a[:, col["theta_cmd"]]), "k--", lw=1, label="pitch command")
    ax[1].set_ylabel("pitch (deg)")
    ax[1].legend(loc="upper right", fontsize=8)
    ax[2].plot(t, np.degrees(a[:, col["stern_plane"]]), color="C2", lw=1)
    ax[2].set_ylabel("stern plane (deg)")
    ax[2].set_xlabel("time (s)")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--link", default="tcp:127.0.0.1:5760")
    p.add_argument("--realtime", action="store_true", help="pace to the wall clock (use with real hardware)")
    p.add_argument("--duration", type=float, default=360.0)
    p.add_argument("--current", type=float, default=0.0, help="water current speed (m/s)")
    p.add_argument("--current-dir", type=float, default=0.0, help="current direction (deg)")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--out", default="runs/mission")
    args = p.parse_args()

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    sim = Simulator(current_speed=args.current, current_dir_deg=args.current_dir, seed=args.seed)
    link = open_link(args.link)
    try:
        rows, stats = run(sim, link, Mission(duration=args.duration), realtime=args.realtime)
    finally:
        link.close()

    with open(args.out + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(LOG_FIELDS)
        w.writerows(rows)
    plot(rows, args.out + ".png", f"REMUS-100 depth mission via {args.link}")

    print(f"steps {stats['steps']}  late replies {stats['late']}  crc errors {stats['crc_errors']}")
    print(f"timing {stats['timing_mode']}  period overruns {stats['period_overruns']}  "
          f"reply mean/max {stats['reply_ms_mean']:.2f}/{stats['reply_ms_max']:.2f} ms")
    for k, v in summarize(rows).items():
        print(f"{k:48s} {v:8.3f}")
    print(f"wrote {args.out}.csv and {args.out}.png")
    return 0 if stats["late"] == 0 or args.realtime else 1


if __name__ == "__main__":
    sys.exit(main())
