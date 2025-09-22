#include "stm32f4xx.h"
#include "clk_config.h"

#define TICKS_PER_MS 21000

//void configure_sysclk();
// 
//void configure_sysclk() {
//  RCC->CR |= RCC_CR_HSEON;                                               // enable the HSE clock peripheral
//  while(!(RCC->CR & RCC_CR_HSERDY));                                     // wait until HSE is fully ready/non-busy
//  
//  RCC->CR &= ~RCC_CR_PLLON;                                              // disable PLL before configuration 
//
//  // hardcoded to generate 168mhz (needs to be generalized)
//  RCC->PLLCFGR = 
//    RCC_PLLCFGR_PLLSRC_HSE |
//    RCC_PLLCFGR_PLLM_3     |                                             // PLLM (constrains VCO input between 1-2mhz)
//    RCC_PLLCFGR_PLLN_4     | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_8;    // PLLN (set to 336 ie. double the desired 168mhz)
//                                                                         // PLLP (final clock divisor: default to 0, feeds to SYSCLK)
//
//  RCC->CR |= RCC_CR_PLLON;                                               // enable PLL and wait until full ready/non-busy
//  while(!(RCC->CR & RCC_CR_PLLRDY));
//
//  RCC->CFGR |= RCC_CFGR_SW_PLL;                                          // switch SYSCLK to pll
//  while(((RCC->CFGR >> 2) & 0x03) != 2);                                 // wait until SYSCLK is fully ready (TODO: explain this logic)
//
//  FLASH->ACR = 5;                                                        // flash memory cannot repond fast enough to 168mhz. 
//                                                                         // delay CPU by 5 cycles between fetches 
//}

void enable_onboard_led() {
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;    // enabling this clock will enable power to gpio port A.
  GPIOA->MODER  |= GPIO_MODER_MODER5_0;    // sets pa5 to general purpose output mode
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;      // sets pa5 to push-pull output mode: equivalent to &= ~(1 << 5); 
  GPIOA->ODR    |= (1 << 5);               // sets the pa5 bit on the output data register (ODR) to turn on LED 
}

void toggle_led(uint8_t times, uint32_t ms) {
  while(times > 0) {
    GPIOA->ODR ^= (1 << 5);
    for(uint32_t i = 0; i < (ms * TICKS_PER_MS); i++) {
      __asm("nop");
    }

    --times;
  }
}

int main(void)
{
  enable_onboard_led();
  GPIOA->ODR &= ~(1 << 5);
  configure_sysclk();
  toggle_led(200, 100);
  while(1);
}

