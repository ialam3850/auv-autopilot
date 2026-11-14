#pragma once

#include <cstdint>

// Minimal driver for the TDK InvenSense ICM-42688-P 6-axis IMU over SPI1.
//
// Configured for gyro +/-250 dps and accel +/-4 g, both at 200 Hz output data
// rate in low-noise mode; read by polling one 12-byte burst per control step.
//
// Register addresses and field values follow the ICM-42688-P datasheet
// (DS-000347). Bench-check before trusting: whoami() must return 0x47, and
// tilting the board nose-up must make the estimated pitch go positive.
namespace icm42688 {

struct Sample {
  float accel[3];  // m/s^2, vehicle body frame (x fwd, y stbd, z down)
  float gyro[3];   // rad/s, vehicle body frame
};

void spi_init();
// Soft-reset, check WHO_AM_I, configure. Returns false if the chip doesn't answer.
bool init();
std::uint8_t whoami();
bool read(Sample& out);

}  // namespace icm42688
