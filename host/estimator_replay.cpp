// Replays recorded IMU samples through both attitude estimators so they can be
// compared on identical data.
//
// stdin:  one sample per line: t gyro_p gyro_q gyro_r acc_x acc_y acc_z
// stdout: one line per sample: ekf_roll ekf_pitch cf_roll cf_pitch   (radians)

#include <cmath>
#include <cstdio>

#include "autopilot/attitude_ekf.h"
#include "autopilot/complementary_filter.h"

int main() {
  ap::AttitudeEkf ekf;
  ap::ComplementaryFilter cf;
  double t = 0, prev_t = 0;
  float g[3], a[3];
  bool first = true;
  while (std::scanf("%lf %f %f %f %f %f %f", &t, &g[0], &g[1], &g[2], &a[0], &a[1], &a[2]) == 7) {
    if (first) {
      // Same initialisation as Autopilot: level from the first accelerometer sample.
      const float roll0 = std::atan2(-a[1], -a[2]);
      const float pitch0 = std::atan2(a[0], std::sqrt(a[1] * a[1] + a[2] * a[2]));
      ekf.reset(roll0, pitch0);
      cf.reset(roll0, pitch0);
      first = false;
    } else {
      const auto dt = static_cast<float>(t - prev_t);
      ekf.predict(g, dt);
      ekf.update(a);
      cf.update(g, a, dt);
    }
    prev_t = t;
    std::printf("%.6f %.6f %.6f %.6f\n", static_cast<double>(ekf.roll()), static_cast<double>(ekf.pitch()),
                static_cast<double>(cf.roll()), static_cast<double>(cf.pitch()));
  }
  return 0;
}
