#include "autopilot/protocol.h"

#include <cstring>

namespace ap::proto {

std::uint16_t crc16_ccitt(const std::uint8_t* data, std::size_t len) {
  // Poly 0x1021, init 0xFFFF, no reflection, no final xor ("CCITT-FALSE").
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i] << 8);
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<std::uint16_t>(crc << 1);
  }
  return crc;
}

std::size_t cobs_encode(const std::uint8_t* in, std::size_t len, std::uint8_t* out) {
  std::size_t read = 0, write = 1, code_idx = 0;
  std::uint8_t code = 1;
  while (read < len) {
    if (in[read] == 0) {
      out[code_idx] = code;
      code_idx = write++;
      code = 1;
    } else {
      out[write++] = in[read];
      if (++code == 0xFF) {
        out[code_idx] = code;
        code_idx = write++;
        code = 1;
      }
    }
    ++read;
  }
  out[code_idx] = code;
  return write;
}

std::size_t cobs_decode(const std::uint8_t* in, std::size_t len, std::uint8_t* out) {
  std::size_t read = 0, write = 0;
  while (read < len) {
    const std::uint8_t code = in[read++];
    if (code == 0) return 0;
    for (std::uint8_t i = 1; i < code; ++i) {
      if (read >= len) return 0;
      const std::uint8_t b = in[read++];
      if (b == 0) return 0;
      out[write++] = b;
    }
    if (code != 0xFF && read != len) out[write++] = 0;
  }
  return write;
}

std::size_t make_frame(const void* payload, std::size_t len, std::uint8_t* frame_out) {
  if (!payload || !frame_out || len == 0 || len > kMaxPayload) return 0;
  std::uint8_t raw[kMaxPayload + 2];
  std::memcpy(raw, payload, len);
  const std::uint16_t crc = crc16_ccitt(raw, len);
  raw[len] = static_cast<std::uint8_t>(crc & 0xFF);
  raw[len + 1] = static_cast<std::uint8_t>(crc >> 8);
  const std::size_t n = cobs_encode(raw, len + 2, frame_out);
  frame_out[n] = 0;
  return n + 1;
}

bool FrameReceiver::feed(std::uint8_t byte) {
  if (byte != 0) {
    if (len_ < sizeof buf_)
      buf_[len_++] = byte;
    else
      overflow_ = true;
    return false;
  }
  // Delimiter: try to decode what we collected.
  const std::size_t n = len_;
  const bool overflowed = overflow_;
  len_ = 0;
  overflow_ = false;
  if (n == 0) return false;  // back-to-back delimiters are harmless
  if (overflowed || n > kMaxFrame) {
    ++framing_errors_;
    return false;
  }
  std::uint8_t decoded[kMaxFrame];
  const std::size_t d = cobs_decode(buf_, n, decoded);
  if (d < 3 || d > kMaxPayload + 2) {
    ++framing_errors_;
    return false;
  }
  const std::uint16_t got = static_cast<std::uint16_t>(decoded[d - 2] | (decoded[d - 1] << 8));
  if (crc16_ccitt(decoded, d - 2) != got) {
    ++crc_errors_;
    return false;
  }
  std::memcpy(payload_, decoded, d - 2);
  payload_len_ = d - 2;
  return true;
}

}  // namespace ap::proto
