#include "board.h"

#include "stm32f4xx.h"

namespace board {

void init_clocks() {
  // Flash wait states must be raised *before* the clock goes up:
  // 2 WS covers 60-90 MHz at 2.7-3.6 V (RM0390, table 5).
  FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;

  // PLL from HSI (16 MHz): VCO in = 16/16 = 1 MHz, VCO out = 1 * 336 = 336 MHz,
  // SYSCLK = 336/4 = 84 MHz, PLLQ = 336/7 = 48 MHz (USB-ready), PLLR = 2 (unused).
  RCC->PLLCFGR = (16u << RCC_PLLCFGR_PLLM_Pos) | (336u << RCC_PLLCFGR_PLLN_Pos) |
                 (1u << RCC_PLLCFGR_PLLP_Pos) /* /4 */ | (7u << RCC_PLLCFGR_PLLQ_Pos) |
                 (2u << RCC_PLLCFGR_PLLR_Pos) | RCC_PLLCFGR_PLLSRC_HSI;
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY)) {
  }

  // AHB /1, APB1 /2 (max 45 MHz), APB2 /1, then switch SYSCLK to the PLL.
  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_SW)) |
              RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
  }
  SystemCoreClockUpdate();
}

void init_gpio() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
  (void)RCC->AHB1ENR;  // read back: the clock needs a cycle before the port responds

  // PA5: LED, push-pull output.
  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER5) | GPIO_MODER_MODER5_0;

  // PA2/PA3: USART2, alternate function 7.
  GPIOA->MODER =
      (GPIOA->MODER & ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3)) | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFFu << 8)) | (7u << 8) | (7u << 12);
  GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPDR3) | GPIO_PUPDR_PUPDR3_0;  // RX pull-up: idle high

  // PB3/PB4/PB5: SPI1, alternate function 5, high speed.
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5)) |
                 GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFFFu << 12)) | (5u << 12) | (5u << 16) | (5u << 20);
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5;

  // PB6: IMU chip select, output, idle high.
  GPIOB->BSRR = GPIO_BSRR_BS6;
  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER6) | GPIO_MODER_MODER6_0;
}

void led_toggle() {
  GPIOA->ODR ^= GPIO_ODR_OD5;
}

void cycle_counter_init() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

std::uint32_t cycles() {
  return DWT->CYCCNT;
}

}  // namespace board
