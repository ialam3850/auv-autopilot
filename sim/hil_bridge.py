"""Closes the loop between the simulator and an autopilot.

The autopilot is either the STM32 over USB serial (hardware in the loop) or
the host build of the same code over TCP (software in the loop). Both speak
the protocol in protocol.py, so this file doesn't care which.

Each control step:
  1. sample the simulated sensors and send a Sensors message
  2. wait for the Actuators message that answers it (matched by seq)
  3. apply the commanded planes/rudder/rpm and advance the physics by dt

In lockstep mode (SIL) the physics waits for the autopilot, so runs are
deterministic and faster than real time. In real-time mode (HIL) each step
is paced to the wall clock, and a late or missing reply is counted and the
previous command is held, which is what a real vehicle's actuators would do.
"""

import math
import socket
import time
from dataclasses import dataclass, field

from protocol import ACTUATORS, Actuators, Command, FrameReceiver, Sensors, make_frame
from simulator import Simulator


class TcpLink:
    def __init__(self, host, port, timeout=2.0):
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.sock = socket.create_connection((host, port), timeout=timeout)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.05)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def write(self, data: bytes):
        self.sock.sendall(data)

    def read(self, timeout: float) -> bytes:
        self.sock.settimeout(timeout)
        try:
            data = self.sock.recv(4096)
            if not data:
                raise ConnectionError("autopilot connection closed")
            return data
        except socket.timeout:
            return b""

    def close(self):
        self.sock.close()


class SerialLink:
    def __init__(self, port, baud=921600):
        import serial  # only needed with real hardware

        self.ser = serial.Serial(port, baud, timeout=0)
        self.ser.reset_input_buffer()

    def write(self, data: bytes):
        self.ser.write(data)

    def read(self, timeout: float) -> bytes:
        self.ser.timeout = timeout
        return self.ser.read(max(1, self.ser.in_waiting))

    def close(self):
        self.ser.close()


def open_link(spec: str):
    """'tcp:127.0.0.1:5760' or 'serial:/dev/ttyACM0[:baud]'."""
    kind, _, rest = spec.partition(":")
    if kind == "tcp":
        host, _, port = rest.rpartition(":")
        return TcpLink(host, int(port))
    if kind == "serial":
        parts = rest.split(":")
        return SerialLink(parts[0], int(parts[1]) if len(parts) > 1 else 921600)
    raise ValueError(f"unknown link '{spec}'")


@dataclass
class Mission:
    """Depth setpoints over time: [(t_start_s, depth_m), ...] and a fixed propeller rpm."""

    setpoints: list = field(default_factory=lambda: [(0.0, 0.0), (5.0, 10.0), (120.0, 25.0), (240.0, 15.0)])
    duration: float = 360.0
    prop_rpm: float = 1200.0

    def setpoint_at(self, t):
        z = self.setpoints[0][1]
        for t0, depth in self.setpoints:
            if t >= t0:
                z = depth
        return z


LOG_FIELDS = [
    "t", "z", "phi", "theta", "w", "q", "bias_p", "bias_q",
    "gyro_p", "gyro_q", "gyro_r", "acc_x", "acc_y", "acc_z", "depth_meas",
    "z_setpoint", "z_cmd", "theta_cmd", "stern_plane", "rudder", "rpm",
    "est_phi", "est_theta", "est_bias_p", "est_bias_q", "P_phi", "P_theta", "P_phi_theta",
    "est_depth_rate", "compute", "late",
]


def run(sim: Simulator, link, mission: Mission, dt=0.02, realtime=False, reply_timeout=None):
    """Runs one mission. Returns (rows, stats) where rows follow LOG_FIELDS."""
    if dt <= 0 or not math.isfinite(dt):
        raise ValueError("dt must be positive and finite")
    if reply_timeout is not None and (not math.isfinite(reply_timeout) or reply_timeout <= 0):
        raise ValueError("reply_timeout must be positive and finite")
    timeout = min(dt, reply_timeout) if reply_timeout is not None else dt
    rx = FrameReceiver()
    rows, late, seq = [], 0, 0
    reply_times, overruns = [], 0
    u = [0.0, 0.0, 0.0]
    setpoint = None
    last_command_t = -math.inf
    invalid_replies = 0
    t_wall = time.monotonic()
    steps = int(round(mission.duration / dt))

    for k in range(steps):
        t = k * dt
        z_sp = mission.setpoint_at(t)
        if z_sp != setpoint or t - last_command_t >= 1.0:
            setpoint = z_sp
            last_command_t = t
            link.write(make_frame(Command(True, z_sp, mission.prop_rpm).pack()))

        step_start = time.monotonic()
        gyro, accel, depth = sim.read_sensors()
        seq = (seq + 1) & 0xFFFF
        link.write(make_frame(Sensors(seq, int(round(t * 1e6)), tuple(gyro), tuple(accel), depth).pack()))

        reply = None
        deadline = step_start + timeout if realtime else time.monotonic() + 5.0
        while reply is None and time.monotonic() < deadline:
            for payload in rx.feed(link.read(max(0.0, deadline - time.monotonic()))):
                if len(payload) == 60 and payload[0] == ACTUATORS:
                    msg = Actuators.unpack(payload)
                    values = (msg.stern_plane, msg.rudder, msg.prop_rpm, msg.roll, msg.pitch)
                    if (not all(math.isfinite(v) for v in values) or
                            abs(msg.stern_plane) > 0.262 or abs(msg.rudder) > 0.262 or
                            not 0 <= msg.prop_rpm <= 2000):
                        invalid_replies += 1
                        continue
                    if msg.seq == seq and (not realtime or time.monotonic() <= deadline):
                        reply = msg
        reply_times.append(time.monotonic() - step_start)
        if reply is None:
            late += 1  # hold the previous command
        else:
            u = [reply.rudder, reply.stern_plane, reply.prop_rpm]

        tr = sim.truth()
        r = reply
        rows.append([
            t, tr.eta[2], tr.eta[3], tr.eta[4], tr.nu[2], tr.nu[4], tr.gyro_bias[0], tr.gyro_bias[1],
            gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], depth,
            z_sp, r.depth_cmd if r else math.nan, r.pitch_cmd if r else math.nan,
            u[1], u[0], u[2],
            r.roll if r else math.nan, r.pitch if r else math.nan, r.bias_p if r else math.nan,
            r.bias_q if r else math.nan, r.p_roll if r else math.nan, r.p_pitch if r else math.nan,
            r.p_roll_pitch if r else math.nan, r.depth_rate if r else math.nan,
            r.compute_cycles if r else 0, 0 if r else 1,
        ])

        sim.step(u, dt)
        if realtime and time.monotonic() - step_start > dt:
            overruns += 1
        if realtime:
            t_wall += dt
            time.sleep(max(0.0, t_wall - time.monotonic()))

    stats = {"steps": steps, "late": late, "crc_errors": rx.crc_errors,
             "framing_errors": rx.framing_errors, "period_overruns": overruns,
             "reply_ms_mean": 1000 * sum(reply_times) / len(reply_times) if reply_times else 0.0,
             "reply_ms_max": 1000 * max(reply_times, default=0.0),
             "timing_mode": "realtime" if realtime else "lockstep",
             "invalid_replies": invalid_replies}
    return rows, stats
