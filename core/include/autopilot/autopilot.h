#pragma once

#include <cstdint>

#include "autopilot/attitude_ekf.h"
#include "autopilot/depth_controller.h"
#include "autopilot/protocol.h"

namespace ap {

// Everything the vehicle computer does each control step, independent of
// where the sensor data comes from (real IMU, simulator over serial, or a
// unit test) and where the commands go.
class Autopilot {
 public:
  struct Config {
    AttitudeEkf::Config ekf;
    DepthController::Config depth;
    float yaw_damping = 0.5f;  // rudder rad per rad/s of yaw rate; no compass, so damp only
    float max_rudder = 0.2618f;
    // Planes stay neutral this long after enabling: while the propeller spins
    // up, forward acceleration looks like pitch to the accelerometer.
    float arming_delay = 5.0f;  // s
  };

  Autopilot() : Autopilot(Config{}) {}
  explicit Autopilot(const Config& cfg) : cfg_(cfg), ekf_(cfg.ekf), depth_(cfg.depth) {}

  void set_command(const proto::CommandMsg& cmd);

  // One control step. `dt` is the time since the previous sensor sample.
  proto::ActuatorsMsg step(const proto::SensorsMsg& s, float dt);

  const AttitudeEkf& ekf() const { return ekf_; }
  bool enabled() const { return enabled_; }

 private:
  Config cfg_;
  AttitudeEkf ekf_;
  DepthController depth_;
  DepthRateFilter depth_rate_;
  bool enabled_ = false;
  bool initialised_ = false;
  float time_enabled_ = 0.0f;
  float depth_setpoint_ = 0.0f;
  float prop_rpm_ = 0.0f;
};

}  // namespace ap
