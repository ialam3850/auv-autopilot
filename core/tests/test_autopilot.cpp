#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "autopilot/autopilot.h"
#include "autopilot/sample_clock.h"
#include "autopilot/link_watchdog.h"

using namespace ap;

namespace {

proto::SensorsMsg level_sample(std::uint16_t seq, std::uint32_t t_us, float depth) {
  proto::SensorsMsg s;
  s.seq = seq;
  s.t_us = t_us;
  s.accel[2] = -AttitudeEkf::kGravity;
  s.depth = depth;
  return s;
}

TEST(Autopilot, DisabledMeansNeutralActuators) {
  Autopilot ap;
  for (std::uint16_t i = 0; i < 100; ++i) {
    const auto out = ap.step(level_sample(i, i * 20000u, 5.0f), 0.02f);
    EXPECT_EQ(out.seq, i);
    EXPECT_EQ(out.enabled, 0);
    EXPECT_EQ(out.stern_plane, 0.0f);
    EXPECT_EQ(out.rudder, 0.0f);
    EXPECT_EQ(out.prop_rpm, 0.0f);
  }
}

TEST(Autopilot, EnabledFollowsCommand) {
  Autopilot ap;
  ap.step(level_sample(0, 0, 5.0f), 0.0f);
  proto::CommandMsg cmd;
  cmd.enable = 1;
  cmd.depth_setpoint = 15.0f;
  cmd.prop_rpm = 1200.0f;
  ap.set_command(cmd);
  proto::ActuatorsMsg out;
  // Past the 5 s arming delay (planes are held neutral while the propeller spins up).
  for (std::uint16_t i = 1; i < 400; ++i) out = ap.step(level_sample(i, i * 20000u, 5.0f), 0.02f);
  EXPECT_EQ(out.enabled, 1);
  EXPECT_FLOAT_EQ(out.prop_rpm, 1200.0f);
  // Too shallow -> nose-down pitch command.
  EXPECT_LT(out.pitch_cmd, 0.0f);
  EXPECT_GT(out.depth_cmd, 5.0f);
}

TEST(Autopilot, FirstSampleLevelsTheFilter) {
  Autopilot ap;
  proto::SensorsMsg s = level_sample(0, 0, 0.0f);
  // 10 deg nose-up.
  s.accel[0] = AttitudeEkf::kGravity * 0.17365f;
  s.accel[2] = -AttitudeEkf::kGravity * 0.98481f;
  const auto out = ap.step(s, 0.0f);
  EXPECT_NEAR(out.pitch, 0.1745f, 0.001f);
}

TEST(Autopilot, PlanesNeutralDuringArmingDelay) {
  Autopilot ap;
  ap.step(level_sample(0, 0, 0.0f), 0.0f);
  proto::CommandMsg cmd;
  cmd.enable = 1;
  cmd.depth_setpoint = 30.0f;  // large error: planes would move at once if armed
  cmd.prop_rpm = 1200.0f;
  ap.set_command(cmd);
  for (std::uint16_t i = 1; i < 240; ++i) {  // 4.8 s
    const auto out = ap.step(level_sample(i, i * 20000u, 0.0f), 0.02f);
    EXPECT_EQ(out.stern_plane, 0.0f);
    EXPECT_FLOAT_EQ(out.prop_rpm, 1200.0f);
  }
}

TEST(Autopilot, RejectsInvalidSamplesAndRecovers) {
  Autopilot ap;
  ap.step(level_sample(0, 0, 3.0f), 0.0f);
  proto::CommandMsg cmd;
  cmd.enable = 1;
  cmd.prop_rpm = 1200.0f;
  ap.set_command(cmd);
  for (float bad : {std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::infinity()}) {
    auto s = level_sample(1, 20000, 3.0f);
    s.accel[0] = bad;
    const auto out = ap.step(s, 0.02f);
    EXPECT_EQ(out.enabled, 0);
    EXPECT_EQ(out.prop_rpm, 0.0f);
    EXPECT_TRUE(std::isfinite(out.pitch));
  }
  const auto good = ap.step(level_sample(2, 40000, 3.0f), 0.02f);
  EXPECT_EQ(good.enabled, 1);
  EXPECT_TRUE(std::isfinite(good.pitch));
  EXPECT_FLOAT_EQ(good.prop_rpm, 1200.0f);
}

TEST(Autopilot, InvalidCommandDisablesAndBadFirstSampleDoesNotInitialize) {
  Autopilot ap;
  auto bad = level_sample(0, 0, 0.0f);
  bad.depth = std::numeric_limits<float>::quiet_NaN();
  ap.step(bad, 0.0f);
  const auto good = ap.step(level_sample(1, 20000, 5.0f), 0.0f);
  EXPECT_FLOAT_EQ(good.depth_cmd, 5.0f);
  proto::CommandMsg cmd;
  cmd.enable = 1;
  cmd.prop_rpm = -1.0f;
  ap.set_command(cmd);
  EXPECT_FALSE(ap.enabled());
  for (float dt : {-0.1f, 1.0f, std::numeric_limits<float>::quiet_NaN()}) {
    EXPECT_EQ(ap.step(level_sample(2, 40000, 5.0f), dt).enabled, 0);
  }
}

TEST(SampleClock, RejectsDuplicatesAndReorderingAndHandlesWrap) {
  SampleClock clock;
  EXPECT_FLOAT_EQ(clock.update(100000), 0.0f);
  EXPECT_NEAR(clock.update(120000), 0.02f, 1e-7f);
  EXPECT_LT(clock.update(120000), 0.0f);
  EXPECT_LT(clock.update(110000), 0.0f);
  EXPECT_NEAR(clock.update(140000), 0.02f, 1e-7f);
  EXPECT_LT(clock.update(400000), 0.0f);
  EXPECT_NEAR(clock.update(420000), 0.02f, 1e-7f);
  SampleClock wrap;
  wrap.update(0xfffffff0u);
  EXPECT_NEAR(wrap.update(0x00004e10u), 0.02f, 1e-7f);
}

TEST(LinkWatchdog, ExpiresAfterDeadlineAndAcrossTickWrap) {
  LinkWatchdog w;
  EXPECT_FALSE(w.expired(1000, 100));
  w.feed(200);
  EXPECT_FALSE(w.expired(299, 100));
  EXPECT_TRUE(w.expired(300, 100));
  w.feed(0xfffffff0u);
  EXPECT_FALSE(w.expired(0x20u, 100));
  EXPECT_TRUE(w.expired(0x80u, 100));
  w.clear();
  EXPECT_FALSE(w.expired(1000, 100));
}

}  // namespace
