# Changes

## Connect commands, estimation, and depth control

Add arming, neutral outputs, command validation, sample timing, and the link watchdog.

## Add framed sensor and actuator messages

Implement COBS and CRC framing in C++ and Python, with message bounds and stream recovery tests.

## Add attitude filters and gyro-bias estimation

Add matrix helpers, a complementary filter, and an attitude EKF. Test tilt, bias, invalid inputs, and covariance.

## Add depth control and depth-rate estimation

Implement reference filtering, pitch control, actuator limits, and depth-rate tracking with unit tests.

## Set up the AUV repository

Add the project outline, license notices, formatting rules, ignore rules, and repository checks.
