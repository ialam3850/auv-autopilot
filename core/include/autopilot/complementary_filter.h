#pragma once

namespace ap {

// Baseline estimator to compare the EKF against: integrate the gyros, and
// pull slowly toward the roll/pitch implied by the accelerometer. No bias
// estimation, no covariance. This is what most hobby flight controllers start with.
class ComplementaryFilter {
 public:
  explicit ComplementaryFilter(float time_constant_s = 2.0f) : tau_(time_constant_s) {}

  void reset(float roll = 0.0f, float pitch = 0.0f) {
    roll_ = roll;
    pitch_ = pitch;
  }
  void update(const float gyro[3], const float accel[3], float dt);

  float roll() const { return roll_; }
  float pitch() const { return pitch_; }

 private:
  float tau_;
  float roll_ = 0.0f, pitch_ = 0.0f;
};

}  // namespace ap
