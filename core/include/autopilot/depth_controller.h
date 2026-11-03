#pragma once

namespace ap {

// Depth autopilot by successive loop closure (Fossen 2021, sec. 16.2): an
// outer loop turns depth error into a pitch command, and an inner loop turns
// pitch error into a stern-plane angle.
//
//   z_ref --[prefilter]--> z_d, z_d_rate --(PI on depth, D on depth-rate error)--> theta_d (limited)
//   theta_d --(PID on pitch, damping on q)--> stern plane (rate-limited, saturated)
//
// Depth z is positive down; pitch theta is positive nose-up. Being too deep
// (z > z_d) asks for nose-up pitch, which the vehicle's forward speed turns
// into a climb.
class DepthController {
 public:
  struct Config {
    float ref_bandwidth = 0.1f;   // rad/s, critically damped 2nd-order prefilter on the setpoint
    float kp_z = 0.12f;           // rad per metre of depth error
    float ki_z = 0.002f;          // rad per metre-second
    float kd_z = 0.3f;            // rad per (m/s) of depth-rate error
    float kff_z = 0.6f;           // rad per (m/s) of commanded depth rate; tuned at 1200 rpm (~2 m/s)
    float max_pitch_cmd = 0.35f;  // rad (~20 deg)
    float kp_theta = 4.0f;        // stern plane rad per rad of pitch error
    float ki_theta = 0.2f;
    float kd_theta = 2.0f;         // per rad/s of pitch rate
    float max_plane = 0.2618f;     // rad (15 deg), the REMUS-100 limit
    float max_plane_rate = 0.5f;   // rad/s
    float rate_filter_tau = 0.1f;  // s, low-pass on pitch rate for the D term (gyro noise -> plane jitter)
  };

  DepthController() : DepthController(Config{}) {}
  explicit DepthController(const Config& cfg) : cfg_(cfg) {}

  void reset(float depth_now);
  // Returns the stern-plane command in radians.
  float step(float z_ref, float depth, float depth_rate, float pitch, float pitch_rate, float dt);

  float depth_cmd() const { return z_d_; }
  float depth_rate_cmd() const { return z_d_rate_; }
  float pitch_cmd() const { return theta_d_; }

 private:
  Config cfg_;
  float z_d_ = 0.0f;
  float z_d_rate_ = 0.0f;
  float theta_d_ = 0.0f;
  float z_int_ = 0.0f;
  float theta_int_ = 0.0f;
  float plane_ = 0.0f;
  float q_filt_ = 0.0f;
};

// Depth-rate estimate from a noisy depth sensor: a two-state alpha-beta
// tracker, which is the steady-state Kalman filter for constant velocity.
class DepthRateFilter {
 public:
  explicit DepthRateFilter(float alpha = 0.05f, float beta = 0.002f) : a_(alpha), b_(beta) {}
  void reset(float depth) {
    z_ = depth;
    v_ = 0.0f;
    init_ = true;
  }
  void update(float measured_depth, float dt);
  float depth() const { return z_; }
  float rate() const { return v_; }

 private:
  float a_, b_;
  float z_ = 0.0f, v_ = 0.0f;
  bool init_ = false;
};

}  // namespace ap
