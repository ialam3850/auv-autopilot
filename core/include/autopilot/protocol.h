#pragma once

#include <cstddef>
#include <cstdint>

// Wire protocol between the simulator and the autopilot (real board over USB
// serial, or the host build over TCP). Each frame is
//
//   COBS( payload || CRC-16/CCITT-FALSE(payload) ) 0x00
//
// COBS removes every zero byte from the encoded frame, so 0x00 is an
// unambiguous frame delimiter and a receiver can resynchronise after line
// noise by waiting for the next zero. The CRC catches corrupted frames.
// Payloads are packed little-endian structs (both ends are little-endian:
// x86-64 and Cortex-M4).
namespace ap::proto {

enum class MsgType : std::uint8_t {
  Sensors = 'S',    // simulator -> autopilot, one per control step
  Command = 'C',    // simulator/operator -> autopilot: setpoints and mode
  Actuators = 'A',  // autopilot -> simulator: actuator commands + estimates
};

#pragma pack(push, 1)

struct SensorsMsg {
  std::uint8_t type = static_cast<std::uint8_t>(MsgType::Sensors);
  std::uint8_t reserved = 0;
  std::uint16_t seq = 0;
  std::uint32_t t_us = 0;  // simulation time
  float gyro[3] = {};      // rad/s, body frame
  float accel[3] = {};     // m/s^2, specific force, body frame
  float depth = 0;         // m, positive down
};

struct CommandMsg {
  std::uint8_t type = static_cast<std::uint8_t>(MsgType::Command);
  std::uint8_t enable = 0;  // 0 = planes neutral, propeller off
  std::uint16_t reserved = 0;
  float depth_setpoint = 0;  // m
  float prop_rpm = 0;
};

struct ActuatorsMsg {
  std::uint8_t type = static_cast<std::uint8_t>(MsgType::Actuators);
  std::uint8_t enabled = 0;
  std::uint16_t seq = 0;  // echoes the SensorsMsg this answers
  float stern_plane = 0;  // rad
  float rudder = 0;       // rad
  float prop_rpm = 0;
  // Estimator and controller state, for logging and consistency analysis.
  float roll = 0, pitch = 0, bias_p = 0, bias_q = 0;
  float p_roll = 0, p_pitch = 0, p_roll_pitch = 0;  // covariance entries
  float depth_cmd = 0, pitch_cmd = 0, depth_rate = 0;
  std::uint32_t compute_cycles = 0;  // time spent in Autopilot::step (DWT cycles on the MCU, ns on host)
};

#pragma pack(pop)

static_assert(sizeof(SensorsMsg) == 36);
static_assert(sizeof(CommandMsg) == 12);
static_assert(sizeof(ActuatorsMsg) == 60);

inline constexpr std::size_t kMaxPayload = 64;
// COBS adds at most one byte per 254, plus the CRC and the delimiter.
inline constexpr std::size_t kMaxFrame = kMaxPayload + 2 + 2 + 1;

std::uint16_t crc16_ccitt(const std::uint8_t* data, std::size_t len);

// Encodes `len` bytes into `out` (which needs len + len/254 + 1 bytes).
// Returns the encoded length. Does not append the 0x00 delimiter.
std::size_t cobs_encode(const std::uint8_t* in, std::size_t len, std::uint8_t* out);
// Decodes in place-compatible buffers; returns decoded length, or 0 on error.
std::size_t cobs_decode(const std::uint8_t* in, std::size_t len, std::uint8_t* out);

// payload -> complete frame including CRC and trailing 0x00. Returns frame length.
std::size_t make_frame(const void* payload, std::size_t len, std::uint8_t* frame_out);

// Incremental receiver: feed bytes as they arrive; when a full, valid frame
// is available, `feed` returns true and payload()/payload_len() hold it.
class FrameReceiver {
 public:
  bool feed(std::uint8_t byte);
  const std::uint8_t* payload() const { return payload_; }
  std::size_t payload_len() const { return payload_len_; }
  std::uint32_t crc_errors() const { return crc_errors_; }
  std::uint32_t framing_errors() const { return framing_errors_; }

 private:
  std::uint8_t buf_[kMaxFrame + 8];
  std::size_t len_ = 0;
  bool overflow_ = false;
  std::uint8_t payload_[kMaxPayload + 2];
  std::size_t payload_len_ = 0;
  std::uint32_t crc_errors_ = 0;
  std::uint32_t framing_errors_ = 0;
};

}  // namespace ap::proto
