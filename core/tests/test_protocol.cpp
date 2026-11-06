#include <gtest/gtest.h>

#include <cstring>
#include <random>
#include <vector>

#include "autopilot/protocol.h"

using namespace ap::proto;

namespace {

TEST(Protocol, CrcKnownVector) {
  // CRC-16/CCITT-FALSE check value for "123456789" is 0x29B1.
  const char* s = "123456789";
  EXPECT_EQ(crc16_ccitt(reinterpret_cast<const std::uint8_t*>(s), 9), 0x29B1);
}

TEST(Protocol, CobsKnownVectors) {
  // From the COBS paper / Wikipedia examples.
  const std::uint8_t in1[] = {0x00};
  std::uint8_t out[16];
  ASSERT_EQ(cobs_encode(in1, 1, out), 2u);
  EXPECT_EQ(out[0], 0x01);
  EXPECT_EQ(out[1], 0x01);

  const std::uint8_t in2[] = {0x11, 0x22, 0x00, 0x33};
  ASSERT_EQ(cobs_encode(in2, 4, out), 5u);
  const std::uint8_t want2[] = {0x03, 0x11, 0x22, 0x02, 0x33};
  EXPECT_EQ(std::memcmp(out, want2, 5), 0);
}

TEST(Protocol, CobsLongRunsAndTrailingZero) {
  // 254 non-zero bytes is exactly one full COBS block (code 0xFF); 255 spills
  // into a second block. A trailing zero needs its own code byte.
  for (std::size_t n : {253u, 254u, 255u, 600u}) {
    std::uint8_t in[600 + 1], enc[700], dec[700];
    for (std::size_t i = 0; i < n; ++i) in[i] = static_cast<std::uint8_t>(1 + i % 255);
    in[n] = 0x00;
    for (std::size_t len : {n, n + 1}) {
      const std::size_t m = cobs_encode(in, len, enc);
      EXPECT_LE(m, len + len / 254 + 1);
      for (std::size_t i = 0; i < m; ++i) ASSERT_NE(enc[i], 0) << "n=" << n << " i=" << i;
      ASSERT_EQ(cobs_decode(enc, m, dec), len);
      EXPECT_EQ(std::memcmp(in, dec, len), 0);
    }
  }
}

TEST(Protocol, CobsRoundTripRandom) {
  std::mt19937 rng(7);
  for (int trial = 0; trial < 500; ++trial) {
    const std::size_t n = rng() % 600;
    std::vector<std::uint8_t> in(n);
    for (auto& b : in) b = (rng() % 4 == 0) ? 0 : static_cast<std::uint8_t>(rng());
    std::vector<std::uint8_t> enc(n + n / 254 + 2), dec(n + 2);
    const std::size_t e = cobs_encode(in.data(), n, enc.data());
    for (std::size_t i = 0; i < e; ++i) ASSERT_NE(enc[i], 0) << "zero inside encoded frame";
    const std::size_t d = cobs_decode(enc.data(), e, dec.data());
    ASSERT_EQ(d, n);
    EXPECT_EQ(std::memcmp(in.data(), dec.data(), n), 0);
  }
}

TEST(Protocol, FrameRoundTrip) {
  SensorsMsg s;
  s.seq = 42;
  s.t_us = 123456;
  s.gyro[1] = 0.25f;
  s.accel[2] = -9.81f;
  s.depth = 12.5f;
  std::uint8_t frame[kMaxFrame];
  const std::size_t n = make_frame(&s, sizeof s, frame);
  EXPECT_EQ(frame[n - 1], 0);

  FrameReceiver rx;
  bool got = false;
  for (std::size_t i = 0; i < n; ++i) got = rx.feed(frame[i]);
  ASSERT_TRUE(got);
  ASSERT_EQ(rx.payload_len(), sizeof s);
  SensorsMsg back;
  std::memcpy(&back, rx.payload(), sizeof back);
  EXPECT_EQ(back.seq, 42);
  EXPECT_EQ(back.t_us, 123456u);
  EXPECT_FLOAT_EQ(back.depth, 12.5f);
}

TEST(Protocol, CorruptedFrameIsRejectedAndReceiverResyncs) {
  CommandMsg c;
  c.enable = 1;
  c.depth_setpoint = 20.0f;
  std::uint8_t frame[kMaxFrame];
  const std::size_t n = make_frame(&c, sizeof c, frame);

  FrameReceiver rx;
  std::vector<std::uint8_t> stream = {0x13, 0x37, 0x99};  // junk, then a frame boundary
  stream.push_back(0);
  std::vector<std::uint8_t> bad(frame, frame + n);
  bad[3] ^= 0x40;  // flip a bit inside the payload
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), frame, frame + n);  // then a good copy

  int good = 0;
  for (auto b : stream) good += rx.feed(b) ? 1 : 0;
  EXPECT_EQ(good, 1);
  EXPECT_GE(rx.crc_errors() + rx.framing_errors(), 1u);
}

TEST(Protocol, OversizePayloadIsRejectedWithoutWritingOutput) {
  std::uint8_t raw[kMaxPayload + 1] = {};
  std::uint8_t out[kMaxFrame] = {};
  out[0] = 0x7e;
  EXPECT_EQ(make_frame(raw, sizeof raw, out), 0u);
  EXPECT_EQ(make_frame(raw, 0, out), 0u);
  EXPECT_EQ(out[0], 0x7e);
  FrameReceiver rx;
  for (int i = 0; i < 200; ++i) EXPECT_FALSE(rx.feed(0x01));
  EXPECT_FALSE(rx.feed(0));
  CommandMsg c;
  const auto n = make_frame(&c, sizeof c, out);
  bool received = false;
  for (std::size_t i = 0; i < n; ++i) received = rx.feed(out[i]);
  EXPECT_TRUE(received);
  EXPECT_GT(rx.framing_errors(), 0u);
}

}  // namespace
