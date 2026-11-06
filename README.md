# AUV autopilot

Portable control and estimation code, a vehicle simulator, and STM32 firmware
for a small underwater vehicle.

## Available components

- Depth controller, setpoint filtering, actuator limits, and depth-rate estimation.
- Attitude EKF, complementary filter, gyro-bias estimation, and covariance checks.
- COBS/CRC sensor and actuator framing in C++ and Python.
- Autopilot command handling, arming, sample timing, and link watchdog.
- TCP SIL runner and recorded-IMU replay tool.

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
