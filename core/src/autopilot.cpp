#include "autopilot/autopilot.h"

#include <cmath>

namespace ap {

void Autopilot::set_command(const proto::CommandMsg& cmd) {
  if (cmd.enable && (!std::isfinite(cmd.depth_setpoint) || !std::isfinite(cmd.prop_rpm) ||
                     cmd.depth_setpoint < 0.0f || cmd.depth_setpoint > 1000.0f ||
                     cmd.prop_rpm < 0.0f || cmd.prop_rpm > 2000.0f)) {
    enabled_ = false;
    return;
  }
  const bool was_enabled = enabled_;
  enabled_ = cmd.enable != 0;
  depth_setpoint_ = cmd.depth_setpoint;
  prop_rpm_ = cmd.prop_rpm;
  // Re-arm the controller from wherever the vehicle is now, so enabling
  // mid-run doesn't produce a jump from stale integrators.
  if (enabled_ && !was_enabled) {
    depth_.reset(depth_rate_.depth());
    time_enabled_ = 0.0f;
  }
}

proto::ActuatorsMsg Autopilot::step(const proto::SensorsMsg& s, float dt) {
  bool valid = std::isfinite(s.depth) && s.depth >= -2.0f && s.depth <= 1000.0f &&
               std::isfinite(dt) && dt >= 0.0f && dt <= 0.1f;
  float accel_sq = 0.0f;
  for (int i = 0; i < 3; ++i) {
    valid = valid && std::isfinite(s.gyro[i]) && std::fabs(s.gyro[i]) <= 35.0f &&
            std::isfinite(s.accel[i]) && std::fabs(s.accel[i]) <= 100.0f;
    accel_sq += s.accel[i] * s.accel[i];
  }
  valid = valid && accel_sq >= 1.0f;
  if (!valid) {
    proto::ActuatorsMsg out;
    out.seq = s.seq;
    out.roll = ekf_.roll();
    out.pitch = ekf_.pitch();
    return out;  // neutral output; keep the last healthy filter state
  }
  if (!initialised_) {
    // Level the filter from the first accelerometer sample.
    const float roll0 = std::atan2(-s.accel[1], -s.accel[2]);
    const float pitch0 = std::atan2(s.accel[0], std::sqrt(s.accel[1] * s.accel[1] + s.accel[2] * s.accel[2]));
    ekf_.reset(roll0, pitch0);
    depth_rate_.reset(s.depth);
    depth_.reset(s.depth);
    initialised_ = true;
    dt = 0.0f;
  }

  if (dt > 0.0f) {
    ekf_.predict(s.gyro, dt);
    ekf_.update(s.accel);
    depth_rate_.update(s.depth, dt);
  }

  proto::ActuatorsMsg out;
  out.seq = s.seq;
  out.enabled = enabled_ ? 1 : 0;
  if (enabled_ && dt > 0.0f) {
    time_enabled_ += dt;
    if (time_enabled_ < cfg_.arming_delay) {
      depth_.reset(depth_rate_.depth());  // hold planes neutral; track depth so there's no jump later
    } else {
      const float q = s.gyro[1] - ekf_.bias_q();
      out.stern_plane =
          depth_.step(depth_setpoint_, depth_rate_.depth(), depth_rate_.rate(), ekf_.pitch(), q, dt);
    }
    float rudder = -cfg_.yaw_damping * s.gyro[2];
    if (rudder > cfg_.max_rudder) rudder = cfg_.max_rudder;
    if (rudder < -cfg_.max_rudder) rudder = -cfg_.max_rudder;
    out.rudder = rudder;
    out.prop_rpm = prop_rpm_;
  }

  const auto& P = ekf_.covariance();
  out.roll = ekf_.roll();
  out.pitch = ekf_.pitch();
  out.bias_p = ekf_.bias_p();
  out.bias_q = ekf_.bias_q();
  out.p_roll = P(0, 0);
  out.p_pitch = P(1, 1);
  out.p_roll_pitch = P(0, 1);
  out.depth_cmd = depth_.depth_cmd();
  out.pitch_cmd = depth_.pitch_cmd();
  out.depth_rate = depth_rate_.rate();
  return out;
}

}  // namespace ap
