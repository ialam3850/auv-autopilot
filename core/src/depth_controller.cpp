#include "autopilot/depth_controller.h"

#include <cmath>

namespace ap {

namespace {
float clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

void DepthController::reset(float depth_now) {
  z_d_ = depth_now;
  z_d_rate_ = 0.0f;
  theta_d_ = 0.0f;
  z_int_ = theta_int_ = 0.0f;
  plane_ = 0.0f;
  q_filt_ = 0.0f;
}

float DepthController::step(float z_ref, float depth, float depth_rate, float pitch, float pitch_rate,
                            float dt) {
  // Smooth the setpoint so a 10 m step doesn't demand a full dive instantly.
  // Critically damped second-order prefilter, stepped with its exact solution
  // (stable for any dt). Second order matters: its rate starts at zero, so the
  // rate term below gets a smooth feed instead of a jump at every new setpoint.
  {
    const float w = cfg_.ref_bandwidth;
    const float e = z_d_ - z_ref;
    const float decay = std::exp(-w * dt);
    z_d_ = z_ref + decay * (e * (1.0f + w * dt) + z_d_rate_ * dt);
    z_d_rate_ = decay * (z_d_rate_ * (1.0f - w * dt) - e * w * w * dt);
  }

  // Outer loop: depth -> pitch command. The rate term acts on the rate *error*,
  // so it damps the approach without fighting a commanded dive (damping on
  // the raw depth rate makes the vehicle lag the command, the integrator
  // winds up during the lag, and the result is a slow overshoot).
  const float ez = depth - z_d_;
  // The feedforward term is the pitch a commanded dive rate needs, so the
  // integrator isn't left to supply it (and overshoot later).
  const float feedback = cfg_.kp_z * ez + cfg_.ki_z * z_int_ + cfg_.kd_z * (depth_rate - z_d_rate_);
  const float feedforward = -cfg_.kff_z * z_d_rate_;
  const float theta_d = feedback + feedforward;
  const bool outer_saturated = std::fabs(theta_d) >= cfg_.max_pitch_cmd;
  theta_d_ = clamp(theta_d, -cfg_.max_pitch_cmd, cfg_.max_pitch_cmd);
  // Conditional integration: don't wind up while the pitch command is pinned.
  if (!outer_saturated || (ez * theta_d < 0.0f)) z_int_ += ez * dt;

  // Inner loop: pitch -> stern plane. The rate term uses a low-passed gyro so
  // sensor noise doesn't turn into servo chatter.
  q_filt_ += (dt / (cfg_.rate_filter_tau + dt)) * (pitch_rate - q_filt_);
  const float etheta = pitch - theta_d_;
  const float raw = cfg_.kp_theta * etheta + cfg_.ki_theta * theta_int_ + cfg_.kd_theta * q_filt_;
  const float sat = clamp(raw, -cfg_.max_plane, cfg_.max_plane);
  if (raw == sat || (etheta * raw < 0.0f)) theta_int_ += etheta * dt;

  // Servo rate limit.
  const float max_step = cfg_.max_plane_rate * dt;
  plane_ += clamp(sat - plane_, -max_step, max_step);
  return plane_;
}

void DepthRateFilter::update(float measured_depth, float dt) {
  if (!std::isfinite(measured_depth) || !std::isfinite(dt) || dt <= 0.0f || dt > 0.1f) return;
  if (!init_) {
    reset(measured_depth);
    return;
  }
  const float predicted = z_ + v_ * dt;
  const float residual = measured_depth - predicted;
  z_ = predicted + a_ * residual;
  v_ += (b_ / dt) * residual;
}

}  // namespace ap
