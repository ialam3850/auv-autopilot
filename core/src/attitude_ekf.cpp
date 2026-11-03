#include "autopilot/attitude_ekf.h"

#include <cmath>

namespace ap {

bool invert3(const Mat<3, 3>& m, Mat<3, 3>& out) {
  const float a = m(0, 0), b = m(0, 1), c = m(0, 2);
  const float d = m(1, 0), e = m(1, 1), f = m(1, 2);
  const float g = m(2, 0), h = m(2, 1), i = m(2, 2);
  const float A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const float det = a * A + b * B + c * C;
  if (std::fabs(det) < 1e-12f) return false;
  const float inv = 1.0f / det;
  out(0, 0) = A * inv;
  out(0, 1) = -(b * i - c * h) * inv;
  out(0, 2) = (b * f - c * e) * inv;
  out(1, 0) = B * inv;
  out(1, 1) = (a * i - c * g) * inv;
  out(1, 2) = -(a * f - c * d) * inv;
  out(2, 0) = C * inv;
  out(2, 1) = -(a * h - b * g) * inv;
  out(2, 2) = (a * e - b * d) * inv;
  return true;
}

AttitudeEkf::AttitudeEkf(const Config& cfg) : cfg_(cfg) {
  reset();
}

void AttitudeEkf::reset(float roll, float pitch) {
  x_[0] = roll;
  x_[1] = pitch;
  x_[2] = x_[3] = 0.0f;
  P_ = Mat<4, 4>{};
  const float sa = cfg_.initial_angle_sigma, sb = cfg_.initial_bias_sigma;
  P_(0, 0) = P_(1, 1) = sa * sa;
  P_(2, 2) = P_(3, 3) = sb * sb;
}

void AttitudeEkf::predict(const float gyro[3], float dt) {
  if (!std::isfinite(dt) || dt <= 0.0f || dt > 0.1f) return;
  for (int i = 0; i < 3; ++i) if (!std::isfinite(gyro[i])) return;
  const float phi = x_[0], theta = x_[1];
  const float p = gyro[0] - x_[2];
  const float q = gyro[1] - x_[3];
  const float r = gyro[2];  // yaw-rate bias isn't observable here, so it's left alone

  const float sphi = std::sin(phi), cphi = std::cos(phi);
  const float ctheta = std::cos(theta), ttheta = std::tan(theta);

  // Euler-angle kinematics (Fossen 2021, eq. 2.29)
  const float phi_dot = p + (q * sphi + r * cphi) * ttheta;
  const float theta_dot = q * cphi - r * sphi;

  // Jacobian of the continuous dynamics, then a first-order discretisation.
  Mat<4, 4> F;
  F(0, 0) = (q * cphi - r * sphi) * ttheta;
  F(0, 1) = (q * sphi + r * cphi) / (ctheta * ctheta);
  F(0, 2) = -1.0f;
  F(0, 3) = -sphi * ttheta;
  F(1, 0) = -q * sphi - r * cphi;
  F(1, 3) = -cphi;
  Mat<4, 4> Phi = Mat<4, 4>::identity();
  for (std::size_t k = 0; k < 16; ++k) Phi.v[k] += F.v[k] * dt;

  Mat<4, 4> Q;
  const float qg = cfg_.gyro_noise * cfg_.gyro_noise * dt;
  const float qb = cfg_.gyro_bias_walk * cfg_.gyro_bias_walk * dt;
  Q(0, 0) = Q(1, 1) = qg;
  Q(2, 2) = Q(3, 3) = qb;

  x_[0] += phi_dot * dt;
  x_[1] += theta_dot * dt;
  // Keep clear of the Euler singularity; an AUV never gets near +/-90 deg pitch.
  constexpr float kMaxPitch = 1.4f;
  if (x_[1] > kMaxPitch) x_[1] = kMaxPitch;
  if (x_[1] < -kMaxPitch) x_[1] = -kMaxPitch;

  P_ = Phi * P_ * Phi.transpose() + Q;
}

void AttitudeEkf::update(const float accel[3]) {
  float norm_sq = 0.0f;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(accel[i]) || std::fabs(accel[i]) > 100.0f) return;
    norm_sq += accel[i] * accel[i];
  }
  if (norm_sq < 1.0f) return;
  const float phi = x_[0], theta = x_[1];
  const float sphi = std::sin(phi), cphi = std::cos(phi);
  const float sth = std::sin(theta), cth = std::cos(theta);
  const float g = kGravity;

  // Expected specific force when the only acceleration is gravity.
  const float h[3] = {g * sth, -g * sphi * cth, -g * cphi * cth};
  Mat<3, 4> H;
  H(0, 1) = g * cth;
  H(1, 0) = -g * cphi * cth;
  H(1, 1) = g * sphi * sth;
  H(2, 0) = g * sphi * cth;
  H(2, 1) = g * cphi * sth;

  // Trust the accelerometer less while the vehicle is accelerating.
  const float norm = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
  const float dev = std::fabs(norm - g) / cfg_.accel_gate;
  const float r = cfg_.accel_noise * cfg_.accel_noise * (1.0f + dev * dev);
  Mat<3, 3> R;
  R(0, 0) = R(1, 1) = R(2, 2) = r;

  const Mat<3, 3> S = H * P_ * H.transpose() + R;
  Mat<3, 3> S_inv;
  if (!invert3(S, S_inv)) return;
  const Mat<4, 3> K = P_ * H.transpose() * S_inv;

  Mat<3, 1> y;
  for (std::size_t i = 0; i < 3; ++i) y(i, 0) = accel[i] - h[i];
  const Mat<4, 1> dx = K * y;
  for (std::size_t i = 0; i < 4; ++i) x_[i] += dx(i, 0);
  last_nis_ = (y.transpose() * S_inv * y)(0, 0);

  // Joseph form keeps P symmetric and positive definite in single precision.
  const Mat<4, 4> I_KH = Mat<4, 4>::identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

}  // namespace ap
