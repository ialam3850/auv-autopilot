#pragma once

#include <cstdint>

namespace ap {

// Unwrap 32-bit microsecond timestamps. Negative dt marks a rejected interval.
class SampleClock {
 public:
  float update(std::uint32_t t_us) {
    if (!have_last_) {
      last_ = t_us;
      have_last_ = true;
      return 0.0f;
    }
    const std::uint32_t elapsed = t_us - last_;
    if (elapsed == 0 || elapsed > 0x7fffffffu) return -1.0f;
    last_ = t_us;
    if (elapsed > 100000u) return -1.0f;  // rebase after a long gap
    return static_cast<float>(elapsed) * 1e-6f;
  }

 private:
  std::uint32_t last_ = 0;
  bool have_last_ = false;
};

}  // namespace ap
