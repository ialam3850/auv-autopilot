#pragma once

#include "autopilot/matrix.h"

namespace ap {

// Roll/pitch attitude estimator with gyro-bias estimation.
//
// State x = [phi, theta, b_p, b_q]: roll and pitch (rad) and the bias of the
// roll-rate and pitch-rate gyros (rad/s). Yaw is not estimated: without a
// magnetometer or GPS it isn't observable, and the depth autopilot doesn't
// need it.
//
// Predict: integrate bias-corrected body rates through the Euler-angle
// kinematics. Update: the accelerometer measures specific force, which for a
// slow vehicle is mostly gravity, so it tells us roll and pitch. When the
// measured magnitude is far from 1 g the vehicle is accelerating and the
// update is trusted less (its noise is inflated).
//
// Conventions: NED world frame, body x forward / y starboard / z down.
// At rest a level accelerometer reads f = [0, 0, -g].
class AttitudeEkf {
 public:
  struct Config {
    float gyro_noise = 0.003f;         // rad/s/sqrt(Hz): deliberately inflated, see docs/REPORT.md (NEES)
    float gyro_bias_walk = 0.0002f;    // rad/s/sqrt(s), bias random walk
    float accel_noise = 0.2f;          // m/s^2 per axis: sensor noise plus vibration and small manoeuvres
    float accel_gate = 0.2f;           // m/s^2: deviation of |f| from g that doubles R
    float initial_angle_sigma = 0.1f;  // rad
    float initial_bias_sigma = 0.02f;  // rad/s
  };

  static constexpr float kGravity = 9.81f;

  AttitudeEkf() : AttitudeEkf(Config{}) {}
  explicit AttitudeEkf(const Config& cfg);

  void reset(float roll = 0.0f, float pitch = 0.0f);

  // gyro = [p, q, r] rad/s, dt in seconds.
  void predict(const float gyro[3], float dt);
  // accel = specific force [fx, fy, fz] m/s^2.
  void update(const float accel[3]);

  float roll() const { return x_[0]; }
  float pitch() const { return x_[1]; }
  float bias_p() const { return x_[2]; }
  float bias_q() const { return x_[3]; }
  const Mat<4, 4>& covariance() const { return P_; }
  // Normalised innovation squared of the last update (for consistency checks).
  float last_nis() const { return last_nis_; }

 private:
  Config cfg_;
  float x_[4] = {0, 0, 0, 0};
  Mat<4, 4> P_;
  float last_nis_ = 0.0f;
};

}  // namespace ap
