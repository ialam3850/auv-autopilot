#include "autopilot/complementary_filter.h"

#include <cmath>

namespace ap {

void ComplementaryFilter::update(const float gyro[3], const float accel[3], float dt) {
  // Accelerometer-only attitude (valid when the vehicle isn't accelerating).
  const float acc_roll = std::atan2(-accel[1], -accel[2]);
  const float acc_pitch = std::atan2(accel[0], std::sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

  // Gyro propagation, same kinematics as the EKF but without bias correction.
  const float sphi = std::sin(roll_), cphi = std::cos(roll_);
  const float ttheta = std::tan(pitch_);
  const float roll_gyro = roll_ + (gyro[0] + (gyro[1] * sphi + gyro[2] * cphi) * ttheta) * dt;
  const float pitch_gyro = pitch_ + (gyro[1] * cphi - gyro[2] * sphi) * dt;

  const float alpha = tau_ / (tau_ + dt);
  roll_ = alpha * roll_gyro + (1.0f - alpha) * acc_roll;
  pitch_ = alpha * pitch_gyro + (1.0f - alpha) * acc_pitch;
}

}  // namespace ap
