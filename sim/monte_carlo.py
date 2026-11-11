#!/usr/bin/env python3
"""Monte Carlo validation of the autopilot in software-in-the-loop.

Each run flies the same depth mission with a different random seed (sensor
noise, gyro biases), water current (0-0.3 m/s, any direction) and starting
pitch. The autopilot is the real core code via build/sil_runner.

Reported:
  * control: steady-state depth error, overshoot, tracking error
  * estimation: roll/pitch error of the EKF vs a complementary filter on the
    *same* sensor data (build/estimator_replay)
  * consistency: NEES of the EKF's [roll, pitch] estimate averaged over runs,
    against the chi-square 95% band. Inside the band means the filter's
    covariance honestly describes its error; above means overconfident,
    below means overly cautious.

  python3 sim/monte_carlo.py --runs 40 --out docs/img
"""

import argparse
import math
import json
import os
import socket
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from hil_bridge import LOG_FIELDS, Mission, TcpLink, run
from simulator import Simulator

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "..", "build")
COL = {n: i for i, n in enumerate(LOG_FIELDS)}
MISSION = Mission(setpoints=[(0.0, 0.0), (5.0, 10.0), (120.0, 20.0)], duration=240.0)
SETTLE_WINDOWS = [(90.0, 120.0, 10.0), (210.0, 240.0, 20.0)]  # (t0, t1, setpoint)


def chi2_quantile(p, k):
    """Chi-square quantile by the Wilson-Hilferty approximation.

    Good to well under 1% for the degrees of freedom used here (2 x runs), and
    it avoids a scipy dependency. Only the 2.5% and 97.5% points are needed.
    """
    z = {0.025: -1.959964, 0.975: 1.959964}[p]
    return k * (1 - 2 / (9 * k) + z * math.sqrt(2 / (9 * k))) ** 3


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def one_run(i, gyro_noise=None):
    rng = np.random.default_rng(1000 + i)
    params = dict(
        seed=i,
        current_speed=float(rng.uniform(0.0, 0.3)),
        current_dir_deg=float(rng.uniform(0.0, 360.0)),
        initial_pitch=float(np.radians(rng.normal(0.0, 3.0))),
    )
    port = free_port()
    cmd = [os.path.join(BUILD, "sil_runner"), str(port), "--once"]
    if gyro_noise is not None:
        cmd += ["--gyro-noise", str(gyro_noise)]
    proc = subprocess.Popen(cmd, stderr=subprocess.DEVNULL)
    try:
        link = TcpLink("127.0.0.1", port)
        rows, stats = run(Simulator(**params), link, MISSION)
        link.close()
    finally:
        proc.wait(timeout=10)
    a = np.array(rows, dtype=float)

    # Baseline estimator on identical sensor data.
    imu = a[:, [COL["t"], COL["gyro_p"], COL["gyro_q"], COL["gyro_r"], COL["acc_x"], COL["acc_y"], COL["acc_z"]]]
    out = subprocess.run([os.path.join(BUILD, "estimator_replay")], input="\n".join(
        " ".join(f"{v:.9g}" for v in row) for row in imu), capture_output=True, text=True, check=True)
    rep = np.array([list(map(float, line.split())) for line in out.stdout.splitlines()])
    return params, stats, a, rep


def metrics(a, rep):
    t = a[:, COL["t"]]
    m = {}
    errs = []
    for t0, t1, sp in SETTLE_WINDOWS:
        w = (t >= t0) & (t < t1)
        errs.append(np.mean(np.abs(a[w, COL["z"]] - sp)))
    m["steady_depth_err"] = float(np.mean(errs))
    seg = (t >= 120) & (t < 240)
    m["overshoot_20m"] = float(max(0.0, a[seg, COL["z"]].max() - 20.0))
    after = t > 10
    m["track_rms"] = float(np.sqrt(np.nanmean((a[after, COL["z"]] - a[after, COL["z_cmd"]]) ** 2)))
    m["max_plane_deg"] = float(np.degrees(np.max(np.abs(a[:, COL["stern_plane"]]))))
    true_phi, true_theta = a[:, COL["phi"]], a[:, COL["theta"]]
    # EKF numbers come from the autopilot's own estimate (what the controller used);
    # the complementary filter is the replay on the same IMU samples.
    m["ekf_pitch_rms_deg"] = float(np.degrees(np.sqrt(np.mean((a[after, COL["est_theta"]] - true_theta[after]) ** 2))))
    m["ekf_roll_rms_deg"] = float(np.degrees(np.sqrt(np.mean((a[after, COL["est_phi"]] - true_phi[after]) ** 2))))
    m["cf_pitch_rms_deg"] = float(np.degrees(np.sqrt(np.mean((rep[after, 3] - true_theta[after]) ** 2))))
    m["cf_roll_rms_deg"] = float(np.degrees(np.sqrt(np.mean((rep[after, 2] - true_phi[after]) ** 2))))
    m["bias_q_err"] = float(abs(a[-1, COL["est_bias_q"]] - a[-1, COL["bias_q"]]))
    m["compute_ns_median"] = float(np.median(a[:, COL["compute"]]))
    return m


def nees_series(a):
    """NEES of the [roll, pitch] error under the EKF's reported covariance, per time step."""
    e1 = a[:, COL["est_phi"]] - a[:, COL["phi"]]
    e2 = a[:, COL["est_theta"]] - a[:, COL["theta"]]
    p11, p22, p12 = a[:, COL["P_phi"]], a[:, COL["P_theta"]], a[:, COL["P_phi_theta"]]
    det = p11 * p22 - p12 * p12
    return (p22 * e1 * e1 - 2 * p12 * e1 * e2 + p11 * e2 * e2) / det


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--runs", type=int, default=40)
    p.add_argument("--workers", type=int, default=os.cpu_count() or 2)
    p.add_argument("--gyro-noise", type=float, default=None,
                   help="override the EKF gyro noise (default: the value in attitude_ekf.h)")
    p.add_argument("--no-plots", action="store_true")
    p.add_argument("--out", default=os.path.join(HERE, "..", "docs", "img"))
    args = p.parse_args()
    if args.runs < 1 or args.workers < 1:
        p.error("runs and workers must be positive")
    if args.gyro_noise is not None and (not math.isfinite(args.gyro_noise) or args.gyro_noise <= 0):
        p.error("gyro noise must be positive and finite")
    for exe in ("sil_runner", "estimator_replay"):
        if not os.path.exists(os.path.join(BUILD, exe)):
            sys.exit(f"build/{exe} not found: build the host project first")
    os.makedirs(args.out, exist_ok=True)

    t0 = time.time()
    with ProcessPoolExecutor(args.workers) as pool:
        results = list(pool.map(one_run, range(args.runs), [args.gyro_noise] * args.runs))
    print(f"{args.runs} runs in {time.time() - t0:.0f} s")

    ms = [metrics(a, rep) for _, _, a, rep in results]
    late = sum(s["late"] for _, s, _, _ in results)
    keys = list(ms[0].keys())
    print(f"late replies across all runs: {late}")
    print(f"{'metric':22s} {'median':>9s} {'p95':>9s} {'max':>9s}")
    for k in keys:
        v = np.array([m[k] for m in ms])
        print(f"{k:22s} {np.median(v):9.3f} {np.percentile(v, 95):9.3f} {v.max():9.3f}")

    # NEES averaged across runs at each time step (skip the first 10 s).
    t = results[0][2][:, COL["t"]]
    nees = np.array([nees_series(a) for _, _, a, _ in results])
    keep = t > 10
    avg = nees[:, keep].mean(axis=0)
    n = len(results)
    lo, hi = chi2_quantile(0.025, 2 * n) / n, chi2_quantile(0.975, 2 * n) / n
    inside = np.mean((avg >= lo) & (avg <= hi))
    print(f"NEES (2 dof, {n} runs): time-averaged {avg.mean():.2f} (ideal 2.00), "
          f"95% band [{lo:.2f}, {hi:.2f}], {100 * inside:.0f}% of steps inside")

    summary = {
        "runs": args.runs,
        "gyro_noise": args.gyro_noise if args.gyro_noise is not None else 0.003,
        "timing_mode": "lockstep",
        "late_replies": late,
        "metrics": {k: {"median": float(np.median([m[k] for m in ms])),
                         "max": float(max(m[k] for m in ms))} for k in keys},
        "nees_mean": float(avg.mean()),
        "seeds": list(range(args.runs)),
    }
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2, allow_nan=False)
        f.write("\n")
    if not args.no_plots:
        plot(results, ms, t, keep, avg, lo, hi, args.out)
    return 0


def plot(results, ms, t, keep, avg, lo, hi, out):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # 1. Depth tracking, all runs.
    fig, ax = plt.subplots(figsize=(9, 4))
    for _, _, a, _ in results:
        ax.plot(a[:, COL["t"]], a[:, COL["z"]], color="C0", alpha=0.25, lw=0.8)
    a0 = results[0][2]
    ax.plot(a0[:, COL["t"]], a0[:, COL["z_setpoint"]], "k--", lw=1, label="setpoint")
    ax.plot(a0[:, COL["t"]], a0[:, COL["z_cmd"]], color="0.4", lw=1, label="prefiltered command")
    ax.invert_yaxis()
    ax.set_xlabel("time (s)")
    ax.set_ylabel("depth (m)")
    ax.set_title(f"Depth tracking, {len(results)} Monte Carlo runs (currents 0-0.3 m/s, random biases)")
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "mc_depth.png"), dpi=120)

    # 2. NEES consistency.
    fig, ax = plt.subplots(figsize=(9, 3.5))
    ax.plot(t[keep], avg, color="C0", lw=0.8, label="average NEES over runs")
    ax.axhline(2.0, color="k", lw=1, label="expected (2 dof)")
    ax.axhspan(lo, hi, color="C2", alpha=0.2, label="95% band")
    ax.set_yscale("log")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("NEES [roll, pitch]")
    ax.set_title("EKF consistency: is the reported covariance honest?")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "mc_nees.png"), dpi=120)

    # 3. EKF vs complementary filter.
    fig, ax = plt.subplots(figsize=(6, 3.5))
    data = [[m["ekf_pitch_rms_deg"] for m in ms], [m["cf_pitch_rms_deg"] for m in ms],
            [m["ekf_roll_rms_deg"] for m in ms], [m["cf_roll_rms_deg"] for m in ms]]
    ax.boxplot(data)
    ax.set_xticks([1, 2, 3, 4], ["EKF\npitch", "CF\npitch", "EKF\nroll", "CF\nroll"])
    ax.set_ylabel("RMS error (deg)")
    ax.set_title("Attitude error per run, same sensor data")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "mc_estimators.png"), dpi=120)

    # 4. One run: pitch error with the EKF's 3-sigma bound.
    a = results[0][2]
    tt = a[:, COL["t"]]
    err = np.degrees(a[:, COL["est_theta"]] - a[:, COL["theta"]])
    sig = np.degrees(3 * np.sqrt(a[:, COL["P_theta"]]))
    fig, ax = plt.subplots(figsize=(9, 3.5))
    ax.plot(tt, err, color="C0", lw=0.6, label="pitch error")
    ax.fill_between(tt, -sig, sig, color="C1", alpha=0.2, label="EKF 3-sigma")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("deg")
    ax.set_ylim(-1.0, 1.0)
    ax.set_title("Pitch estimate error vs. the filter's own uncertainty (run 0)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "mc_pitch_error.png"), dpi=120)


if __name__ == "__main__":
    sys.exit(main())
