#pragma once

#include <cstdint>

namespace ap {

// Uses wrapping RTOS ticks. No dependency on the scheduler or board headers.
class LinkWatchdog {
 public:
  void feed(std::uint32_t now) { last_ = now; armed_ = true; }
  bool expired(std::uint32_t now, std::uint32_t timeout) const {
    return armed_ && now - last_ >= timeout;
  }
  void clear() { armed_ = false; }

 private:
  std::uint32_t last_ = 0;
  bool armed_ = false;
};

}  // namespace ap
