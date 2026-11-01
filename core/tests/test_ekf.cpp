#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <limits>

#include "autopilot/attitude_ekf.h"
#include "autopilot/complementary_filter.h"

using namespace ap;

namespace {

constexpr float kDeg = 3.14159265f / 180.0f;
constexpr float g = AttitudeEkf::kGravity;

// Specific force a stationary IMU reads at the given attitude.
void gravity_reading(float roll, float pitch, float out[3]) {
  out[0] = g * std::sin(pitch);
  out[1] = -g * std::sin(roll) * std::cos(pitch);
  out[2] = -g * std::cos(roll) * std::cos(pitch);
}

TEST(Matrix, Invert3) {
  Mat<3, 3> m;
  m(0, 0) = 4;
  m(0, 1) = 7;
  m(0, 2) = 2;
  m(1, 0) = 3;
  m(1, 1) = 6;
  m(1, 2) = 1;
  m(2, 0) = 2;
  m(2, 1) = 5;
  m(2, 2) = 3;
  Mat<3, 3> inv;
  ASSERT_TRUE(invert3(m, inv));
  const auto id = m * inv;
  for (std::size_t r = 0; r < 3; ++r)
    for (std::size_t c = 0; c < 3; ++c) EXPECT_NEAR(id(r, c), r == c ? 1.0f : 0.0f, 1e-5f);

  Mat<3, 3> singular;  // all zeros
  EXPECT_FALSE(invert3(singular, inv));
}

TEST(AttitudeEkf, ConvergesToStaticTilt) {
  AttitudeEkf ekf;
  ekf.reset(0, 0);
  float acc[3];
  gravity_reading(10 * kDeg, -5 * kDeg, acc);
  const float gyro[3] = {0, 0, 0};
  for (int i = 0; i < 500; ++i) {
    ekf.predict(gyro, 0.02f);
    ekf.update(acc);
  }
  EXPECT_NEAR(ekf.roll(), 10 * kDeg, 0.2f * kDeg);
  EXPECT_NEAR(ekf.pitch(), -5 * kDeg, 0.2f * kDeg);
}

TEST(AttitudeEkf, EstimatesGyroBias) {
  // Vehicle is still, but the gyros report a constant offset. The EKF should
  // learn the offset instead of letting the angle drift.
  AttitudeEkf ekf;
  const float bias_p = 0.01f, bias_q = -0.008f;
  const float gyro[3] = {bias_p, bias_q, 0};
  float acc[3];
  gravity_reading(0, 0, acc);
  for (int i = 0; i < 6000; ++i) {  // 2 minutes at 50 Hz
    ekf.predict(gyro, 0.02f);
    ekf.update(acc);
  }
  EXPECT_NEAR(ekf.bias_p(), bias_p, 0.0005f);
  EXPECT_NEAR(ekf.bias_q(), bias_q, 0.0005f);
  EXPECT_NEAR(ekf.roll(), 0.0f, 0.1f * kDeg);
  EXPECT_NEAR(ekf.pitch(), 0.0f, 0.1f * kDeg);
}

TEST(AttitudeEkf, TracksRotationFromGyro) {
  // Pitch up at 5 deg/s for 4 s with consistent accelerometer readings.
  AttitudeEkf ekf;
  const float rate = 5 * kDeg, dt = 0.02f;
  float theta = 0;
  for (int i = 0; i < 200; ++i) {
    theta += rate * dt;
    const float gyro[3] = {0, rate, 0};
    float acc[3];
    gravity_reading(0, theta, acc);
    ekf.predict(gyro, dt);
    ekf.update(acc);
  }
  EXPECT_NEAR(ekf.pitch(), 20 * kDeg, 0.3f * kDeg);
}

TEST(AttitudeEkf, CovarianceStaysSymmetricPositive) {
  AttitudeEkf ekf;
  std::mt19937 rng(1);
  std::normal_distribution<float> n(0, 1);
  float acc[3];
  for (int i = 0; i < 5000; ++i) {
    const float gyro[3] = {0.01f * n(rng), 0.01f * n(rng), 0.01f * n(rng)};
    gravity_reading(0.05f * std::sin(static_cast<float>(i) * 0.01f),
                    0.1f * std::cos(static_cast<float>(i) * 0.007f), acc);
    for (float& a : acc) a += 0.05f * n(rng);
    ekf.predict(gyro, 0.02f);
    ekf.update(acc);
  }
  const auto& P = ekf.covariance();
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_GT(P(i, i), 0.0f);
    for (std::size_t j = 0; j < 4; ++j) EXPECT_NEAR(P(i, j), P(j, i), 1e-6f);
  }
}

TEST(AttitudeEkf, DistrustsAccelerometerWhileAccelerating) {
  // A 3 m/s^2 forward acceleration looks like pitch to a naive filter. With
  // the gate, the EKF should move far less than the raw tilt it implies.
  AttitudeEkf ekf;
  const float gyro[3] = {0, 0, 0};
  for (int i = 0; i < 200; ++i) {  // settle level first
    float acc[3];
    gravity_reading(0, 0, acc);
    ekf.predict(gyro, 0.02f);
    ekf.update(acc);
  }
  float acc[3];
  gravity_reading(0, 0, acc);
  acc[0] += 3.0f;                 // naive pitch = atan(3/9.81) ~ 17 deg
  for (int i = 0; i < 50; ++i) {  // one second of acceleration
    ekf.predict(gyro, 0.02f);
    ekf.update(acc);
  }
  EXPECT_LT(std::fabs(ekf.pitch()), 8 * kDeg);
}

TEST(ComplementaryFilter, ConvergesToStaticTilt) {
  ComplementaryFilter cf(1.0f);
  float acc[3];
  gravity_reading(-8 * kDeg, 12 * kDeg, acc);
  const float gyro[3] = {0, 0, 0};
  for (int i = 0; i < 1000; ++i) cf.update(gyro, acc, 0.02f);
  EXPECT_NEAR(cf.roll(), -8 * kDeg, 0.1f * kDeg);
  EXPECT_NEAR(cf.pitch(), 12 * kDeg, 0.1f * kDeg);
}

TEST(AttitudeEkf, InvalidInputsLeaveHealthyStateIntact) {
  AttitudeEkf ekf;
  const float gyro[3] = {0, 0, 0};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float bad[3] = {nan, 0, -g};
  ekf.predict(gyro, nan);
  ekf.predict(bad, 0.02f);
  ekf.update(bad);
  const float zero[3] = {0, 0, 0};
  ekf.update(zero);
  EXPECT_FLOAT_EQ(ekf.pitch(), 0.0f);
  float acc[3];
  gravity_reading(0, 0, acc);
  ekf.predict(gyro, 0.02f);
  ekf.update(acc);
  EXPECT_TRUE(std::isfinite(ekf.covariance()(1, 1)));
}

TEST(AttitudeEkf, CovarianceHasACholeskyFactorThroughoutMotion) {
  AttitudeEkf ekf;
  for (int k = 0; k < 2000; ++k) {
    const float gyro[3] = {0.01f, 0.02f, -0.01f};
    float acc[3];
    gravity_reading(0.05f * std::sin(static_cast<float>(k) * 0.01f), 0.1f, acc);
    ekf.predict(gyro, 0.02f);
    ekf.update(acc);
    const auto& P = ekf.covariance();
    double L[4][4] = {};
    for (std::size_t i = 0; i < 4; ++i) {
      for (std::size_t j = 0; j <= i; ++j) {
        EXPECT_NEAR(P(i, j), P(j, i), 1e-6f);
        double value = static_cast<double>(P(i, j));
        for (std::size_t m = 0; m < j; ++m) value -= L[i][m] * L[j][m];
        if (i == j) {
          ASSERT_GT(value, 0.0);
          L[i][j] = std::sqrt(value);
        } else {
          L[i][j] = value / L[j][j];
        }
      }
    }
  }
}

}  // namespace
