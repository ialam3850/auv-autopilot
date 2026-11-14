# AUV autopilot

Portable control and estimation code, a vehicle simulator, and STM32 firmware
for a small underwater vehicle.

## Available components

- Depth controller, setpoint filtering, actuator limits, and depth-rate estimation.
- Attitude EKF, complementary filter, gyro-bias estimation, and covariance checks.
- COBS/CRC sensor and actuator framing in C++ and Python.
- Autopilot command handling, arming, sample timing, and link watchdog.
- TCP SIL runner and recorded-IMU replay tool.
- REMUS-100 dynamics and synthetic sensor observations.
- TCP/serial control bridge, timing metrics, and closed-loop SIL checks.
- Mission runner and Monte Carlo summaries.
- STM32F446RE clocks, pins, cycle counter, and DMA UART driver.
- ICM-42688 SPI driver with bounded waits.

## Checks

From the repository root, with CMake 3.20+, Ninja, and a C++17 compiler:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

Python checks:

```bash
python -m pip install -r requirements.txt
(cd sim && python -m unittest discover -s tests -v)
```

For a mission, start `build/sil_runner 5760 --once` in one terminal, then run:

```bash
python sim/run_mission.py --duration 360 --out runs/mission
python sim/monte_carlo.py --runs 40 --out runs/monte-carlo
```

With an Arm GNU toolchain on PATH:

```bash
cmake -S firmware -B build-fw -G Ninja -DCMAKE_TOOLCHAIN_FILE="$PWD/firmware/cmake/arm-none-eabi.cmake"
cmake --build build-fw --target board_drivers
```

Hardware files have compile checks; physical board and underwater tests have not been performed.
