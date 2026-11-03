#include <gtest/gtest.h>

#include <cmath>

#include "autopilot/depth_controller.h"

using namespace ap;

namespace {

TEST(DepthController, TooDeepAsksForNoseUp) {
  DepthController dc;
  dc.reset(10.0f);
  float plane = 0;
  for (int i = 0; i < 50; ++i) plane = dc.step(5.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.02f);
  EXPECT_GT(dc.pitch_cmd(), 0.0f);
  // Vehicle level but commanded nose-up: pitch error is negative, so the
  // plane command is negative (convention checked in closed loop in sim/tests).
  EXPECT_LT(plane, 0.0f);
}

TEST(DepthController, PitchCommandIsLimited) {
  DepthController::Config cfg;
  cfg.ref_bandwidth = 100.0f;  // prefilter effectively off: (nearly) full error immediately
  DepthController dc(cfg);
  dc.reset(0.0f);
  dc.step(100.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);
  EXPECT_NEAR(std::fabs(dc.pitch_cmd()), cfg.max_pitch_cmd, 1e-6f);
}

TEST(DepthController, PlaneIsSaturatedAndRateLimited) {
  DepthController::Config cfg;
  DepthController dc(cfg);
  dc.reset(0.0f);
  float prev = 0.0f;
  for (int i = 0; i < 500; ++i) {
    // Huge pitch error drives the plane to its limit.
    const float plane = dc.step(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.02f);
    EXPECT_LE(std::fabs(plane), cfg.max_plane + 1e-6f);
    EXPECT_LE(std::fabs(plane - prev), cfg.max_plane_rate * 0.02f + 1e-6f);
    prev = plane;
  }
  EXPECT_NEAR(std::fabs(prev), cfg.max_plane, 1e-6f);
}

TEST(DepthController, IntegratorDoesNotWindUpWhileSaturated) {
  DepthController::Config cfg;
  cfg.ref_bandwidth = 100.0f;
  DepthController a(cfg), b(cfg);
  a.reset(0.0f);
  b.reset(0.0f);
  // `a` sits saturated for a long time; `b` doesn't. When the error goes
  // away, both should release the planes at about the same time.
  for (int i = 0; i < 3000; ++i) a.step(50.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);
  float pa = 0, pb = 0;
  for (int i = 0; i < 1000; ++i) {
    pa = a.step(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);
    pb = b.step(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);
  }
  EXPECT_NEAR(pa, pb, 0.02f);
}

TEST(DepthController, PrefilterIsSmoothAndDoesNotOvershoot) {
  DepthController dc;
  dc.reset(0.0f);
  float max_cmd = 0.0f, prev_rate = 0.0f, max_rate_jump = 0.0f;
  for (int i = 0; i < 10000; ++i) {  // 200 s
    dc.step(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);
    max_cmd = std::fmax(max_cmd, dc.depth_cmd());
    max_rate_jump = std::fmax(max_rate_jump, std::fabs(dc.depth_rate_cmd() - prev_rate));
    prev_rate = dc.depth_rate_cmd();
  }
  EXPECT_NEAR(dc.depth_cmd(), 10.0f, 1e-3f);
  EXPECT_LE(max_cmd, 10.0f + 1e-4f);  // critically damped: no overshoot
  EXPECT_LT(max_rate_jump, 0.002f);   // no step in the rate feed (a 1st-order filter jumps 1 m/s here)
}

TEST(DepthRateFilter, TracksConstantDescent) {
  DepthRateFilter f;
  float z = 0;
  for (int i = 0; i < 1000; ++i) {
    z += 0.5f * 0.02f;
    f.update(z, 0.02f);
  }
  EXPECT_NEAR(f.rate(), 0.5f, 0.01f);
  EXPECT_NEAR(f.depth(), z, 0.01f);
}

TEST(DepthRateFilter, ZeroOrNegativeDtDoesNotCorruptState) {
  DepthRateFilter f;
  f.reset(4.0f);
  f.update(8.0f, 0.0f);
  f.update(8.0f, -0.1f);
  EXPECT_FLOAT_EQ(f.depth(), 4.0f);
  EXPECT_FLOAT_EQ(f.rate(), 0.0f);
  f.update(4.1f, 0.02f);
  EXPECT_TRUE(std::isfinite(f.rate()));
}

}  // namespace
