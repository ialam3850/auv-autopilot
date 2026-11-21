# Hardware setup

Everything here costs about $35 and needs no soldering beyond header pins.
The vehicle itself is simulated; the hardware is the flight computer and,
optionally, a real IMU to check the estimator against.

> Status: the firmware builds cleanly for both modes, but it has **not yet
> been run on a board**. The register values come from the datasheets. The
> bring-up checklist below is the order to verify things in.

## Bill of materials

| Part | Approx. price | Notes |
|---|---|---|
| ST NUCLEO-F446RE | $15 | Cortex-M4F at 84 MHz (this firmware), built-in ST-LINK programmer and USB serial port |
| ICM-42688-P breakout (SPI) | $10–15 | Any breakout that exposes SCLK/SDI/SDO/CS and runs at 3.3 V |
| 6 female–female jumper wires | $2 | |
| USB Mini-B cable | $3 | Some NUCLEO revisions use Micro-B; check yours |

HIL mode needs only the NUCLEO. BENCH mode adds the IMU.

## Wiring (BENCH mode)

| NUCLEO pin | Arduino header label | IMU breakout pin |
|---|---|---|
| PB3 | D3 | SCLK / SCL |
| PB4 | D5 | SDO / AD0 (MISO) |
| PB5 | D4 | SDI / SDA (MOSI) |
| PB6 | D10 | CS |
| 3V3 | 3V3 | VCC / VDD |
| GND | GND | GND |

SPI1 is routed to port B because PA5, the usual SPI1 clock pin, drives the
green LED. Mount the breakout flat with the chip's X arrow pointing toward
the "bow" (pick an edge of the board) and its Z axis up; the driver maps
sensor axes to vehicle axes as `body = (x, -y, -z)`. A different mounting
needs a different line in `firmware/src/icm42688.cpp`.

## Toolchain

* Arm GNU toolchain (`arm-none-eabi-gcc` 12 or newer), CMake 3.20+, Ninja.
  On Ubuntu `sudo apt install gcc-arm-none-eabi` works; the CI uses the
  xPack 13.3.1 build.
* A flasher: STM32CubeProgrammer, `st-flash` (stlink-tools), or OpenOCD.
* `pip install -r requirements.txt` for the Python side (pyserial is needed
  only for hardware).

## Build and flash

```bash
# HIL: the board runs the autopilot on simulated sensor data from the PC
cmake -S firmware -B build-fw-hil -G Ninja -DAP_FW_MODE=HIL \
      -DCMAKE_TOOLCHAIN_FILE=$PWD/firmware/cmake/arm-none-eabi.cmake
cmake --build build-fw-hil
st-flash --reset write build-fw-hil/autopilot_fw.bin 0x08000000

# BENCH: the board reads the real IMU and streams its attitude estimate
cmake -S firmware -B build-fw-bench -G Ninja -DAP_FW_MODE=BENCH \
      -DCMAKE_TOOLCHAIN_FILE=$PWD/firmware/cmake/arm-none-eabi.cmake
cmake --build build-fw-bench
st-flash --reset write build-fw-bench/autopilot_fw.bin 0x08000000
```

The first configure downloads FreeRTOS, CMSIS and the ST device headers at
pinned tags (see `firmware/CMakeLists.txt`).

Alternatively, drag `autopilot_fw.bin` onto the `NOD_F446RE` USB drive the
board shows up as.

## Bring-up checklist

Do these in order; each one rules out a class of problem before the next.

1. **Scheduler alive.** After flashing, the green LED (LD2) toggles every
   0.5 s. If it doesn't, the clock setup or the FreeRTOS port is wrong; stop
   in the debugger and check where it is.
2. **Serial link.** Find the port (`/dev/ttyACM0` on Linux,
   `/dev/tty.usbmodem*` on macOS, `COMx` on Windows). The ST-LINK virtual COM
   port must support 921 600 baud; recent ST-LINK V2-1 firmware does. If
   frames come back with CRC errors, update the ST-LINK firmware or lower
   `kBaud` in `uart.h` and `SerialLink` in `sim/hil_bridge.py` together.
3. **HIL closed loop.**
   ```bash
   python3 sim/run_mission.py --link serial:/dev/ttyACM0 --realtime --out runs/hil
   ```
   The printed summary should be close to the SIL numbers in the README
   (the same code is running; only the float rounding and timing differ).
   "late replies" should be 0. The `compute` column is in CPU cycles here:
   divide by 84 for microseconds.
4. **IMU answers (BENCH firmware).** If the IMU doesn't return WHO_AM_I
   = 0x47, the LED blinks fast (10 Hz) instead of slow. Check wiring and CS.
5. **Axes and signs.**
   ```bash
   python3 sim/bench_monitor.py /dev/ttyACM0
   ```
   Flat on the desk: roll and pitch within about 1°. Raise the bow edge:
   pitch goes **positive**. Lower the starboard edge: roll goes
   **positive**. If a sign is wrong, fix the axis mapping, not the filter.
6. **Gyro bias.** Leave the board still for a minute: `bias_q` should
   settle to a constant (typically under 0.01 rad/s) and pitch shouldn't
   drift. Then tap the desk or slide the board briskly: the estimate should
   barely move, because the EKF distrusts the accelerometer when |f| ≠ g.
7. **Timing.** Note the step time printed by `bench_monitor.py` and record
   it in the README (expected: tens of microseconds at 84 MHz).

## What to measure once it runs (fill in the README)

* Control step time: median and maximum, in µs (from `compute_cycles`).
* HIL round-trip: `late replies` over a 6-minute mission.
* Bench: static roll/pitch noise (standard deviation over 60 s) and the
  pitch read with the board on a known wedge (e.g. a 3D-printed 20° block).

## Fault checks

- Disconnect the HIL sensor stream: expect one neutral reply after 100 ms.
- Resume traffic and send an enabled command: the arming delay starts again.
- In the debugger, inspect `inbox_drops`, `control_stack_free_words`, and
  `uart::overrun_count()`. Stack values are words, not bytes.
- Disconnect the SPI IMU: reads should fail after bounded polling instead of
  blocking the task. These checks still need a board.
