// Autopilot firmware for the NUCLEO-F446RE.
//
// Two build modes (CMake option AP_FW_MODE):
//
//   HIL    The simulator on the PC sends simulated IMU/depth samples over USB
//          serial; the board runs the real autopilot code on them and sends
//          back stern-plane/rudder/propeller commands. This is the mode for
//          closed-loop testing: the vehicle is simulated, the computer is real.
//
//   BENCH  The board reads the real ICM-42688-P over SPI at 50 Hz and streams
//          its attitude estimate in the same Actuators message, so you can
//          tilt the board by hand and watch roll/pitch (sim/bench_monitor.py).
//
// Tasks (higher number = higher priority):
//   control  (4)  runs Autopilot::step, measures its cycle count, replies
//   uart_rx  (3)  woken by the UART/DMA ISRs; de-frames bytes into messages
//   blinky   (1)  heartbeat LED, so you can see the scheduler is alive

#include <cstring>

#include "FreeRTOS.h"
#include "autopilot/autopilot.h"
#include "autopilot/sample_clock.h"
#include "autopilot/link_watchdog.h"
#include "autopilot/protocol.h"
#include "board.h"
#include "icm42688.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "task.h"
#include "uart.h"

#ifndef AP_FW_MODE_BENCH
#define AP_FW_MODE_HIL 1
#endif

using namespace ap;

namespace {

// One decoded message from the PC. Commands and sensor samples share a queue
// so they're applied in the order they arrived.
struct Inbox {
  proto::MsgType type;
  proto::SensorsMsg sensors;
  proto::CommandMsg command;
};

QueueHandle_t inbox;
TaskHandle_t rx_task_handle;
volatile std::uint32_t inbox_drops = 0;
volatile UBaseType_t control_stack_free_words = 0;
Autopilot autopilot;  // owned by the control task once the scheduler starts

void send_actuators(const proto::ActuatorsMsg& a) {
  std::uint8_t frame[proto::kMaxFrame];
  const std::size_t n = proto::make_frame(&a, sizeof a, frame);
  uart::write(frame, n);
  control_stack_free_words = uxTaskGetStackHighWaterMark(nullptr);
}

void uart_rx_task(void*) {
  proto::FrameReceiver rx;
  std::uint8_t chunk[64];
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));  // also poll every 10 ms in case of a missed wake
    std::size_t n;
    while ((n = uart::read_available(chunk, sizeof chunk)) > 0) {
      for (std::size_t i = 0; i < n; ++i) {
        if (!rx.feed(chunk[i])) continue;
        const std::uint8_t* p = rx.payload();
        const std::size_t len = rx.payload_len();
        Inbox msg;
        if (len == sizeof(proto::SensorsMsg) && p[0] == static_cast<std::uint8_t>(proto::MsgType::Sensors)) {
          msg.type = proto::MsgType::Sensors;
          std::memcpy(&msg.sensors, p, len);
        } else if (len == sizeof(proto::CommandMsg) &&
                   p[0] == static_cast<std::uint8_t>(proto::MsgType::Command)) {
          msg.type = proto::MsgType::Command;
          std::memcpy(&msg.command, p, len);
        } else {
          continue;
        }
        if (xQueueSend(inbox, &msg, 0) != pdTRUE) ++inbox_drops;
      }
    }
  }
}

#if AP_FW_MODE_HIL
void control_task(void*) {
  SampleClock sample_clock;
  LinkWatchdog watchdog;
  proto::ActuatorsMsg last_output;
  Inbox msg;
  for (;;) {
    if (xQueueReceive(inbox, &msg, pdMS_TO_TICKS(20)) != pdTRUE) {
      if (watchdog.expired(xTaskGetTickCount(), pdMS_TO_TICKS(100))) {
        proto::CommandMsg stop;
        autopilot.set_command(stop);
        proto::ActuatorsMsg neutral;
        neutral.seq = last_output.seq;
        send_actuators(neutral);
        watchdog.clear();
      }
      continue;
    }
    if (msg.type == proto::MsgType::Command) {
      autopilot.set_command(msg.command);
      continue;
    }
    const proto::SensorsMsg& s = msg.sensors;
    const float dt = sample_clock.update(s.t_us);
    if (dt >= 0.0f) watchdog.feed(xTaskGetTickCount());

    const std::uint32_t c0 = board::cycles();
    proto::ActuatorsMsg out = autopilot.step(s, dt);
    out.compute_cycles = board::cycles() - c0;  // 84 cycles = 1 us
    last_output = out;
    send_actuators(out);
  }
}
#else
void control_task(void*) {
  if (!icm42688::init()) {
    // No IMU: blink fast forever so the failure is obvious on the bench.
    for (;;) {
      board::led_toggle();
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
  constexpr TickType_t kPeriod = pdMS_TO_TICKS(20);  // 50 Hz
  TickType_t wake = xTaskGetTickCount();
  std::uint32_t t_us = 0;
  std::uint16_t seq = 0;
  for (;;) {
    vTaskDelayUntil(&wake, kPeriod);
    icm42688::Sample imu;
    if (!icm42688::read(imu)) continue;
    proto::SensorsMsg s;
    s.seq = ++seq;
    s.t_us = (t_us += 20000);
    std::memcpy(s.gyro, imu.gyro, sizeof s.gyro);
    std::memcpy(s.accel, imu.accel, sizeof s.accel);
    const std::uint32_t c0 = board::cycles();
    proto::ActuatorsMsg out = autopilot.step(s, seq == 1 ? 0.0f : 0.02f);
    out.compute_cycles = board::cycles() - c0;
    send_actuators(out);
    // Commands are ignored on the bench: the planes aren't connected to anything.
    Inbox drop;
    while (xQueueReceive(inbox, &drop, 0) == pdTRUE) {
    }
  }
}
#endif

void blinky_task(void*) {
  for (;;) {
    board::led_toggle();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

}  // namespace

int main() {
  board::init_clocks();
  board::init_gpio();
  board::cycle_counter_init();
  icm42688::spi_init();

  inbox = xQueueCreate(8, sizeof(Inbox));
  configASSERT(inbox != nullptr);
  configASSERT(xTaskCreate(control_task, "control", 1024, nullptr, 4, nullptr) == pdPASS);
  configASSERT(xTaskCreate(uart_rx_task, "uart_rx", 512, nullptr, 3, &rx_task_handle) == pdPASS);
  configASSERT(xTaskCreate(blinky_task, "blinky", 128, nullptr, 1, nullptr) == pdPASS);
  uart::init(rx_task_handle);  // after the task exists, so the ISR has someone to wake

  vTaskStartScheduler();
  for (;;) {
  }  // only reached if the scheduler couldn't start (heap too small)
}

// ---- FreeRTOS hooks --------------------------------------------------------

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char*) {
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }  // a debugger will stop here; the LED stops blinking
}

extern "C" void vApplicationMallocFailedHook() {
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}
