// Software-in-the-loop runner: the autopilot core compiled for the laptop,
// speaking the same framed protocol the STM32 speaks over USB serial, but on
// a TCP socket. The simulator can't tell the difference, which is the point:
// everything except the drivers and the RTOS gets exercised before any
// hardware is involved.
//
//   ./sil_runner [port] [--once] [--gyro-noise X]
//
//   port            default 5760
//   --once          exit after the first client disconnects (used by tests)
//   --gyro-noise X  override the EKF's gyro noise setting (tuning studies)

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "autopilot/autopilot.h"
#include "autopilot/sample_clock.h"
#include "autopilot/protocol.h"

using namespace ap;

namespace {

bool send_all(int fd, const std::uint8_t* p, std::size_t n) {
  while (n > 0) {
    const ssize_t w = ::send(fd, p, n, MSG_NOSIGNAL);
    if (w <= 0) return false;
    p += w;
    n -= static_cast<std::size_t>(w);
  }
  return true;
}

void serve(int fd, const Autopilot::Config& cfg) {
  Autopilot autopilot(cfg);
  proto::FrameReceiver rx;
  SampleClock sample_clock;
  std::uint64_t steps = 0;

  std::uint8_t buf[4096];
  for (;;) {
    const ssize_t n = ::recv(fd, buf, sizeof buf, 0);
    if (n <= 0) break;
    for (ssize_t i = 0; i < n; ++i) {
      if (!rx.feed(buf[i])) continue;
      const std::uint8_t* p = rx.payload();
      const std::size_t len = rx.payload_len();
      if (len == sizeof(proto::CommandMsg) && p[0] == static_cast<std::uint8_t>(proto::MsgType::Command)) {
        proto::CommandMsg c;
        std::memcpy(&c, p, sizeof c);
        autopilot.set_command(c);
      } else if (len == sizeof(proto::SensorsMsg) &&
                 p[0] == static_cast<std::uint8_t>(proto::MsgType::Sensors)) {
        proto::SensorsMsg s;
        std::memcpy(&s, p, sizeof s);
        const float dt = sample_clock.update(s.t_us);

        const auto t0 = std::chrono::steady_clock::now();
        proto::ActuatorsMsg a = autopilot.step(s, dt);
        const auto t1 = std::chrono::steady_clock::now();
        a.compute_cycles =
            static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());

        std::uint8_t frame[proto::kMaxFrame];
        const std::size_t flen = proto::make_frame(&a, sizeof a, frame);
        if (!send_all(fd, frame, flen)) return;
        ++steps;
      }
    }
  }
  std::fprintf(stderr,
               "sil_runner: client disconnected after %llu steps (crc errors %u, framing errors %u)\n",
               static_cast<unsigned long long>(steps), rx.crc_errors(), rx.framing_errors());
}

}  // namespace

int main(int argc, char** argv) {
  int port = 5760;
  bool once = false;
  Autopilot::Config cfg;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--once") == 0) {
      once = true;
    } else if (std::strcmp(argv[i], "--gyro-noise") == 0 && i + 1 < argc) {
      cfg.ekf.gyro_noise = std::strtof(argv[++i], nullptr);
    } else {
      port = std::atoi(argv[i]);
    }
  }

  const int lfd = ::socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(lfd, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0 || ::listen(lfd, 1) < 0) {
    std::perror("sil_runner: bind/listen");
    return 1;
  }
  std::fprintf(stderr, "sil_runner: listening on 127.0.0.1:%d\n", port);

  do {
    const int fd = ::accept(lfd, nullptr, nullptr);
    if (fd < 0) continue;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    serve(fd, cfg);  // fresh Autopilot per connection, like a board reset
    ::close(fd);
  } while (!once);
  ::close(lfd);
  return 0;
}
