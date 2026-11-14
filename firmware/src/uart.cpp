#include "uart.h"

#include "board.h"
#include "stm32f4xx.h"

namespace uart {

namespace {

std::uint8_t rx_buf[kRxBufferSize];
std::size_t read_pos = 0;
TaskHandle_t reader_task = nullptr;
volatile std::uint32_t overruns = 0;

// Where the DMA will write next: NDTR counts *down* from the buffer size and
// reloads at zero in circular mode. The modulo covers reading it as 0 in the
// instant before the reload, which would otherwise index one past the end.
std::size_t dma_write_pos() {
  return (kRxBufferSize - DMA1_Stream5->NDTR) % kRxBufferSize;
}

void wake_reader_from_isr() {
  BaseType_t woken = pdFALSE;
  if (reader_task) vTaskNotifyGiveFromISR(reader_task, &woken);
  portYIELD_FROM_ISR(woken);
}

}  // namespace

void init(TaskHandle_t reader) {
  reader_task = reader;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  (void)RCC->AHB1ENR;

  // 8N1, oversampling by 16. BRR = f_APB1 / baud, rounded: 42 MHz / 921600 =
  // 45.6 -> 46, i.e. 913 kbaud (0.9% error, within the ~2% the receiver tolerates).
  USART2->CR1 = 0;
  USART2->BRR = (board::kApb1Hz + kBaud / 2) / kBaud;
  USART2->CR3 = USART_CR3_DMAR;

  // DMA1 Stream 5, channel 4 = USART2_RX. Peripheral -> memory, byte-wide,
  // memory increment, circular, interrupts at half and full.
  DMA1_Stream5->CR = 0;
  while (DMA1_Stream5->CR & DMA_SxCR_EN) {
  }
  DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
  DMA1_Stream5->PAR = reinterpret_cast<std::uint32_t>(&USART2->DR);
  DMA1_Stream5->M0AR = reinterpret_cast<std::uint32_t>(rx_buf);
  DMA1_Stream5->NDTR = kRxBufferSize;
  DMA1_Stream5->CR =
      (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE;
  DMA1_Stream5->CR |= DMA_SxCR_EN;

  // Priority 6: below configMAX_SYSCALL_INTERRUPT_PRIORITY (5), so these ISRs
  // may call FreeRTOS ...FromISR functions.
  NVIC_SetPriority(DMA1_Stream5_IRQn, 6);
  NVIC_SetPriority(USART2_IRQn, 6);
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  NVIC_EnableIRQ(USART2_IRQn);

  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_UE;
}

std::size_t read_available(std::uint8_t* out, std::size_t max) {
  std::size_t n = 0;
  const std::size_t write_pos = dma_write_pos();
  // If the reader fell a whole buffer behind, the DMA has overwritten unread
  // data; we can't detect that from positions alone, which is why the buffer
  // is sized for many frames and the reader runs at high priority.
  while (read_pos != write_pos && n < max) {
    out[n++] = rx_buf[read_pos];
    read_pos = (read_pos + 1) % kRxBufferSize;
  }
  return n;
}

void write(const std::uint8_t* data, std::size_t len) {
  for (std::size_t i = 0; i < len; ++i) {
    while (!(USART2->SR & USART_SR_TXE)) {
    }
    USART2->DR = data[i];
  }
  while (!(USART2->SR & USART_SR_TC)) {
  }
}

std::uint32_t overrun_count() {
  return overruns;
}

}  // namespace uart

extern "C" void DMA1_Stream5_IRQHandler() {
  const std::uint32_t flags = DMA1->HISR;
  DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
  if (flags & (DMA_HISR_HTIF5 | DMA_HISR_TCIF5)) uart::wake_reader_from_isr();
}

extern "C" void USART2_IRQHandler() {
  const std::uint32_t sr = USART2->SR;
  if (sr & (USART_SR_IDLE | USART_SR_ORE)) {
    // Clearing IDLE/ORE takes a read of SR followed by a read of DR (RM0390 30.6.1).
    (void)USART2->DR;
    if (sr & USART_SR_ORE) ++uart::overruns;
    uart::wake_reader_from_isr();
  }
}
