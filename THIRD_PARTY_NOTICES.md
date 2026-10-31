# Third-party notices

## Vendored into this repository

### Python Vehicle Simulator (Thor I. Fossen) — MIT License
`sim/vehicle/remus100.py` and `sim/vehicle/fossen_gnc.py` are adapted from
https://github.com/cybergalactic/PythonVehicleSimulator (`vehicles/remus100.py`
and `lib/gnc.py`). Changes: upstream autopilots and plotting removed; only the
vehicle model and the kinematics/hydrodynamics helpers it needs are kept.

Upstream's LICENSE file is the MIT text below, with the copyright line as it
appears there (the project itself is by Thor I. Fossen):

    Copyright (c) 2018 The Python Packaging Authority

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

## Fetched at build time (not stored in this repository)

| Dependency | Version | License | Used by |
|---|---|---|---|
| FreeRTOS-Kernel | V11.1.0 | MIT | firmware |
| ARM CMSIS_6 (Core) | v6.1.0 | Apache-2.0 | firmware |
| STM32CubeF4 cmsis_device_f4 (startup file, system file, register headers) | v2.6.10 | Apache-2.0 | firmware |
| GoogleTest | v1.15.2 | BSD-3-Clause | host unit tests |

Python packages used at run time (installed by the user): numpy (BSD-3),
matplotlib (PSF-based), pyserial (BSD-3, hardware only).

## References (no code copied)

* T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley, 2021.
* R. W. Beard and T. W. McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton, 2012 (successive loop closure, attitude EKF).
* Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001 (NEES consistency test).
* S. Cheshire and M. Baker, "Consistent Overhead Byte Stuffing," IEEE/ACM Trans. Networking, 1999.
* TDK InvenSense ICM-42688-P datasheet, DS-000347.
* ST RM0390, STM32F446xx reference manual.
