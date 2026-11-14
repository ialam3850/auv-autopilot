#include "icm42688.h"
#include "board.h"

#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"

namespace icm42688 {

namespace {

// Bank 0 registers (DS-000347, section 14).
constexpr std::uint8_t kDeviceConfig = 0x11;
constexpr std::uint8_t kAccelDataX1 = 0x1F;  // accel X/Y/Z then gyro X/Y/Z, 16-bit big-endian each
constexpr std::uint8_t kPwrMgmt0 = 0x4E;
constexpr std::uint8_t kGyroConfig0 = 0x4F;
constexpr std::uint8_t kAccelConfig0 = 0x50;
constexpr std::uint8_t kWhoAmI = 0x75;
constexpr std::uint8_t kWhoAmIValue = 0x47;
constexpr std::uint8_t kReadBit = 0x80;

// Full-scale settings -> scale factors.
constexpr std::uint8_t kGyroFs250Dps = 3u << 5;
constexpr std::uint8_t kAccelFs4G = 2u << 5;
constexpr std::uint8_t kOdr200Hz = 0x07;
constexpr float kGyroLsbPerDps = 131.0f;
constexpr float kAccelLsbPerG = 8192.0f;
constexpr float kDegToRad = 3.14159265f / 180.0f;
constexpr float kGravity = 9.81f;

void cs_low() {
  GPIOB->BSRR = GPIO_BSRR_BR6;
}
void cs_high() {
  GPIOB->BSRR = GPIO_BSRR_BS6;
}

bool wait_flag(std::uint32_t mask, bool set) {
  const std::uint32_t start = board::cycles();
  while (((SPI1->SR & mask) != 0) != set) {
    if (board::cycles() - start > board::kSysClockHz / 1000u) return false;
  }
  return true;
}

bool transfer(std::uint8_t out, std::uint8_t& in) {
  if (!wait_flag(SPI_SR_TXE, true)) return false;
  *reinterpret_cast<volatile std::uint8_t*>(&SPI1->DR) = out;
  if (!wait_flag(SPI_SR_RXNE, true)) return false;
  in = *reinterpret_cast<volatile std::uint8_t*>(&SPI1->DR);
  return true;
}

bool write_reg(std::uint8_t reg, std::uint8_t value) {
  cs_low();
  std::uint8_t ignored = 0;
  const bool ok = transfer(reg, ignored) && transfer(value, ignored) && wait_flag(SPI_SR_BSY, false);
  cs_high();
  return ok;
}

bool read_regs(std::uint8_t reg, std::uint8_t* out, std::size_t n) {
  cs_low();
  std::uint8_t ignored = 0;
  bool ok = transfer(reg | kReadBit, ignored);
  for (std::size_t i = 0; ok && i < n; ++i) ok = transfer(0x00, out[i]);
  ok = ok && wait_flag(SPI_SR_BSY, false);
  cs_high();
  return ok;
}

std::int16_t be16(const std::uint8_t* p) {
  return static_cast<std::int16_t>((p[0] << 8) | p[1]);
}

}  // namespace

void spi_init() {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  (void)RCC->APB2ENR;
  // Master, mode 3 (CPOL=1, CPHA=1), 84 MHz / 16 = 5.25 MHz (chip allows 24 MHz),
  // software chip select, 8-bit frames, MSB first.
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA | (3u << SPI_CR1_BR_Pos) | SPI_CR1_SSM | SPI_CR1_SSI;
  SPI1->CR1 |= SPI_CR1_SPE;
}

std::uint8_t whoami() {
  std::uint8_t v = 0;
  read_regs(kWhoAmI, &v, 1);
  return v;
}

bool init() {
  if (!write_reg(kDeviceConfig, 0x01)) return false;  // soft reset
  vTaskDelay(pdMS_TO_TICKS(2));    // datasheet: wait 1 ms after reset
  if (whoami() != kWhoAmIValue) return false;
  if (!write_reg(kGyroConfig0, kGyroFs250Dps | kOdr200Hz) ||
      !write_reg(kAccelConfig0, kAccelFs4G | kOdr200Hz) ||
      !write_reg(kPwrMgmt0, 0x0F)) return false;
  vTaskDelay(pdMS_TO_TICKS(50));  // gyro needs ~30 ms to start up
  return true;
}

bool read(Sample& out) {
  std::uint8_t raw[12];
  if (!read_regs(kAccelDataX1, raw, sizeof raw)) return false;
  const float ax = be16(raw + 0), ay = be16(raw + 2), az = be16(raw + 4);
  const float gx = be16(raw + 6), gy = be16(raw + 8), gz = be16(raw + 10);
  // -32768 is the chip's "data not ready" marker.
  if (be16(raw) == -32768) return false;

  // Sensor axes -> vehicle body axes. Assumes the breakout is mounted flat,
  // chip X toward the bow and chip Z up: body = (x, -y, -z). A level, still
  // board then reads specific force (0, 0, -g), matching the EKF's convention.
  // Change these three lines if your mounting differs.
  const float a_scale = kGravity / kAccelLsbPerG;
  const float g_scale = kDegToRad / kGyroLsbPerDps;
  out.accel[0] = ax * a_scale;
  out.accel[1] = -ay * a_scale;
  out.accel[2] = -az * a_scale;
  out.gyro[0] = gx * g_scale;
  out.gyro[1] = -gy * g_scale;
  out.gyro[2] = -gz * g_scale;
  return true;
}

}  // namespace icm42688
