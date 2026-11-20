# Design

## The idea in one paragraph

An autonomous underwater vehicle (AUV) holds depth by pitching its nose up
or down with its stern planes while the propeller pushes it forward. This
repo is the flight computer for that job: an attitude estimator and a depth
autopilot, written once in portable C++ and run in three places. On a laptop
the code runs against a simulated REMUS-100 vehicle (software in the loop,
SIL). On an STM32 board it runs the same code on simulated sensor data sent
over USB serial (hardware in the loop, HIL). In bench mode the board reads a
real IMU and estimates its own attitude. The simulator, the unit tests and a
Monte Carlo study say how well it works and how sure we can be.

```
                 ┌──────────────── PC (Python) ────────────────┐
                 │  REMUS-100 6-DOF model (Fossen)              │
                 │  IMU + depth sensor models (noise, bias)     │
                 │  mission: depth setpoints, propeller rpm     │
                 └───────┬──────────────────────────▲───────────┘
           Sensors (36 B) │   COBS + CRC-16 frames   │ Actuators (60 B)
           Command (12 B) │   TCP (SIL) / UART (HIL) │
                 ┌───────▼──────────────────────────┴───────────┐
                 │  Autopilot::step()   — same code everywhere  │
                 │   attitude EKF ─► depth controller ─► planes │
                 │   depth-rate filter, yaw damping, arming     │
                 └──────────────────────────────────────────────┘
                   host/sil_runner (Linux)  or  firmware/ (STM32F446RE + FreeRTOS)
```

## Repository layout

| Path | What | Language |
|---|---|---|
| `core/` | Estimator, controller, protocol, `Autopilot` glue. No heap, no exceptions, no OS calls. | C++17 |
| `core/tests/` | Unit tests | GoogleTest |
| `host/sil_runner.cpp` | Runs `core` behind a TCP socket, speaking the board's protocol | C++ |
| `host/estimator_replay.cpp` | Runs both attitude filters over recorded IMU data | C++ |
| `firmware/` | NUCLEO-F446RE build: clocks, UART+DMA, SPI IMU driver, FreeRTOS tasks | C++ / C / asm (ST startup) |
| `sim/vehicle/` | REMUS-100 dynamics, vendored from Fossen (MIT) | Python |
| `sim/simulator.py` | Vehicle + sensor models + ocean current | Python |
| `sim/hil_bridge.py` | Lockstep loop: sensors out, actuators in, over TCP or serial | Python |
| `sim/run_mission.py`, `sim/monte_carlo.py`, `sim/bench_monitor.py` | Tools | Python |

## Conventions

* NED world frame, body frame x forward, y starboard, z down (Fossen).
* Depth `z` positive down. Pitch `theta` positive nose-up. Roll `phi`
  positive starboard-down.
* The accelerometer measures specific force `f = a - g`. Level and still it
  reads `[0, 0, -9.81]`.
* A positive stern-plane angle pitches the nose down (REMUS-100 model sign).
  The closed-loop test in `sim/tests/test_sil_closed_loop.py` exists to catch
  a sign flip here, because a unit test can't.

## Attitude estimator: a 4-state EKF

State `x = [phi, theta, b_p, b_q]`: roll, pitch, and the gyro biases on the
two axes that affect them. Yaw isn't estimated (no magnetometer); the yaw
rate is used directly for rudder damping.

**Predict** with the gyro (bias removed) through the Euler-angle kinematics:

```
phi_dot   = p + (q sin(phi) + r cos(phi)) tan(theta)
theta_dot = q cos(phi) - r sin(phi)
```

with `p = p_meas - b_p`, `q = q_meas - b_q`, biases modelled as random walks.
The Jacobian is written out by hand in `attitude_ekf.cpp` and discretised to
first order (`Phi = I + F dt`), which is accurate at 50 Hz.

**Update** with the accelerometer, treating it as a gravity sensor:

```
h(x) = g · [ sin(theta), -cos(theta) sin(phi), -cos(theta) cos(phi) ]
```

That assumption is wrong whenever the vehicle accelerates, so the
measurement noise grows with how far `|f|` is from `g`:

```
R = sigma_a^2 · (1 + ((|f| - g) / gate)^2) · I
```

Accelerating hard → the filter leans on the gyro. Still → it leans on the
accelerometer and learns the gyro bias. The covariance update uses the
Joseph form, which stays symmetric and positive definite in single
precision. Pitch is clamped at ±80° to keep `tan(theta)` finite.

The **complementary filter** in `complementary_filter.cpp` is the baseline:
integrate the gyro, nudge toward the accelerometer angles with a 2 s time
constant, no bias estimate. It is what most hobby autopilots start with, and
the Monte Carlo study compares the two on identical data.

## Depth controller: successive loop closure

```
z_ref ─► 2nd-order prefilter ─► z_d, ż_d
              outer:  θ_cmd = kp·(z − z_d) + ki·∫(z − z_d) + kd·(ż − ż_d) − kff·ż_d    (limit ±20°)
              inner:  δs   = kp·(θ − θ_cmd) + ki·∫(θ − θ_cmd) + kd·q_filtered          (limit ±15°, 0.5 rad/s)
```

* The **prefilter** is a critically damped second-order filter (bandwidth
  0.1 rad/s) stepped with its exact solution. It turns a 10 m step into a
  smooth dive with no overshoot, and its rate `ż_d` starts at zero.
* The **outer loop** turns depth error into a pitch command. Its damping term
  acts on the depth-rate *error*, and `kff` adds the pitch a commanded dive
  rate needs. Without these, the vehicle lags the command, the integrator
  winds up during the lag, and it overshoots by tens of centimetres and
  creeps back over a minute or more (the first version did exactly this;
  see REPORT.md).
* The **inner loop** is a PID on pitch. The pitch-rate damping uses a
  low-passed gyro so noise doesn't make the plane chatter.
* **Anti-windup** is conditional integration: an integrator stops
  accumulating while its output is saturated, unless the error would unwind
  it.
* The **plane** is saturated at ±15° and rate-limited at 0.5 rad/s, like a
  real servo.
* **Depth rate** comes from an alpha-beta filter on the depth sensor (the
  steady-state Kalman filter for constant velocity), because differentiating
  1 cm-quantised depth directly is mostly noise.

The inner loop is roughly 10× faster than the outer loop, which is what lets
them be tuned one at a time.

### Arming delay

For the first 5 s after enable, the planes stay neutral while the propeller
spins up. Spin-up is a forward acceleration of about 1 m/s², which the
accelerometer reads exactly like a few degrees of nose-down pitch. The first
SIL run showed the controller "correcting" that phantom pitch with a full
plane deflection. Waiting until the speed is steady is what a real vehicle
does too.

## Protocol

One format for SIL (TCP) and HIL (UART), so the simulator can't tell them
apart:

```
payload = packed little-endian struct (first byte = message type 'S', 'C' or 'A')
frame   = COBS( payload ‖ CRC-16/CCITT-FALSE(payload) ) ‖ 0x00
```

COBS removes every zero byte from the encoded data, so `0x00` is an
unambiguous frame delimiter: a receiver that starts mid-stream or loses a
byte recovers at the next zero. The CRC catches corrupted frames, which are
counted and dropped. Overhead is 1 byte per 254 plus 3 bytes.

| Message | Direction | Size | Contents |
|---|---|---|---|
| `SensorsMsg` | PC → AP | 36 B | seq, time (µs), gyro[3], accel[3], depth |
| `CommandMsg` | PC → AP | 12 B | enable, depth setpoint, propeller rpm |
| `ActuatorsMsg` | AP → PC | 60 B | seq echo, planes, rudder, rpm, roll/pitch/bias estimates, covariance entries, controller internals, compute time |

The sizes are `static_assert`ed in C++ and mirrored by `struct` formats in
`sim/protocol.py`, whose tests check the sizes and the same CRC/COBS vectors.

The simulator runs in **lockstep**: it sends a sensor sample, waits for the
matching `seq` in the reply, applies the actuators and steps the physics.
That makes SIL runs deterministic and lets the PC run faster than real time.
With `--realtime` it paces to the wall clock instead, which is what HIL needs
to be meaningful, and replies that miss their 20 ms control period are counted as
late.

## Firmware

* **Clock**: 16 MHz HSI → PLL → 84 MHz (M=16, N=336, P=4). The F446 can run
  at 180 MHz; 84 MHz is plenty and keeps the USART divisor simple.
* **UART**: USART2 at 921 600 baud on the ST-LINK virtual COM port. RX uses
  DMA in circular mode into a 512-byte ring; the half-transfer, transfer
  complete and line-idle interrupts wake the RX task, so bytes are never
  lost and the CPU isn't interrupted per byte. TX is polled (a 60-byte reply
  is 0.7 ms on the wire).
* **IMU**: ICM-42688-P on SPI1, ±250 °/s and ±4 g at 200 Hz, read in one
  12-byte burst per control step.
* **Tasks** (FreeRTOS, heap_4, static sizes):
  `control` (priority 4) runs `Autopilot::step` and replies;
  `uart_rx` (3) de-frames bytes into a queue;
  `blinky` (1) shows the scheduler is alive.
* **Timing**: the DWT cycle counter brackets `Autopilot::step`, and the count
  goes back to the PC in every reply, so HIL runs record their own compute
  time.
* **No heap after start-up**: all core objects are fixed-size; the only
  dynamic allocation is FreeRTOS creating its tasks and queue in `main`.
  A malloc-failed hook traps if that ever goes wrong.

Flash use: about 18 KB of 512 KB. RAM: about 28 KB of 128 KB, mostly the
FreeRTOS heap.

## Testing strategy

| Level | What it catches | Where |
|---|---|---|
| Unit | Math errors: static tilt and rotation tracking, bias convergence, covariance stays symmetric positive definite, accelerometer distrust while accelerating, saturation, anti-windup, prefilter shape, CRC check value, COBS edge cases (254-byte blocks, trailing zeros), frame resync after corruption | `core/tests` |
| Cross-language | Python mirror of the protocol uses the same known vectors; the closed-loop test fails if the two sides disagree on a single byte | `sim/tests/test_protocol.py` |
| Closed-loop SIL | Sign errors and gross tuning errors, through the real socket and protocol | `sim/tests/test_sil_closed_loop.py` |
| Monte Carlo | How good, and how consistent, over many currents, biases and seeds | `sim/monte_carlo.py` |
| HIL / bench | Timing, drivers, the parts a laptop can't check | `docs/HARDWARE.md` |
| Sanitizers | Memory and undefined-behaviour bugs in the core | CI `sanitize` job |

## What it deliberately doesn't do

* No yaw or position estimate (needs a magnetometer or DVL; listed in the
  README as next steps).
* No speed control: the propeller rpm is passed through from the mission.
* Input and timestamp checks reject bad samples. The HIL watchdog disables
  control after a sensor-stream timeout. Physical actuator cutoff, stuck-plane
  detection, and vehicle-level recovery still need implementation and hardware tests.
* The controller gains are fixed and tuned at 1200 rpm (about 2 m/s). Plane
  effectiveness scales with speed squared, so a vehicle that changes speed
  would schedule the gains on rpm.
