# Changes

## Add the IMU SPI driver with bounded waits

Read the ICM-42688 through SPI and expose failures without waiting indefinitely.

## Add STM32 board setup and the DMA serial driver

Add clocks, pins, cycle timing, and USART DMA reception. Cross-compile the drivers before adding an application.

## Add mission runs and Monte Carlo summaries

Run depth missions, compare estimators, and save run settings and metrics as JSON.

## Close the simulation loop through the framed link

Add TCP and serial bridge interfaces, timing metrics, actuator checks, and closed-loop SIL tests.

## Add the vehicle model and synthetic sensors

Integrate the REMUS-100 dynamics with configurable sensor noise, bias, and water current.

## Add the TCP control runner and estimator replay tool

Run the portable controller on the host and replay recorded IMU samples through both attitude estimators.

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
