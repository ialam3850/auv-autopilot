#pragma once

#include <cstdint>

// NUCLEO-F446RE board bring-up: clocks, pins, LED, cycle counter.
//
// Pin map (Arduino header labels in brackets):
//   PA2 / PA3  USART2 TX / RX  -> ST-LINK virtual COM port (USB)
//   PA5        LD2 green LED
//   PB3 [D3]   SPI1 SCK  -> IMU SCLK
//   PB4 [D5]   SPI1 MISO -> IMU SDO
//   PB5 [D4]   SPI1 MOSI -> IMU SDI
//   PB6 [D10]  GPIO      -> IMU CS (active low)
// SPI1 is on port B because PA5 (the default SPI1 SCK) drives the LED.
namespace board {

inline constexpr std::uint32_t kSysClockHz = 84'000'000;
inline constexpr std::uint32_t kApb1Hz = 42'000'000;
inline constexpr std::uint32_t kApb2Hz = 84'000'000;

// HSI 16 MHz -> PLL -> 84 MHz SYSCLK, APB1 42 MHz, APB2 84 MHz.
void init_clocks();
void init_gpio();
void led_toggle();

// DWT cycle counter, for measuring how long the control step takes.
void cycle_counter_init();
std::uint32_t cycles();

}  // namespace board
