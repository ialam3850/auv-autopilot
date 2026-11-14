#pragma once

#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

// USART2 (the ST-LINK virtual COM port) with DMA reception.
//
// RX: DMA1 Stream 5 writes every received byte into a circular buffer with no
// CPU involvement. Three interrupts wake the reader task: the DMA half-transfer
// and transfer-complete interrupts (buffer half / fully wrapped) and the UART
// IDLE-line interrupt (a burst ended, e.g. one complete frame). The task then
// consumes everything between its last read position and the DMA's current
// write position. This is the standard way to receive variable-length
// messages on STM32 without an interrupt per byte.
//
// TX: blocking, polled. Frames are under 70 bytes (~0.8 ms at 921600 baud),
// sent once per control step from one task, so DMA TX isn't worth it yet.
namespace uart {

inline constexpr std::uint32_t kBaud = 921600;
inline constexpr std::size_t kRxBufferSize = 512;

// `reader` is the task to notify when bytes arrive.
void init(TaskHandle_t reader);

// Copies newly received bytes into `out` (up to `max`); returns the count.
std::size_t read_available(std::uint8_t* out, std::size_t max);

void write(const std::uint8_t* data, std::size_t len);

std::uint32_t overrun_count();

}  // namespace uart
