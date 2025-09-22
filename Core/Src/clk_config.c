#include "stm32f4xx.h"
#include "clk_config.h"

void configure_sysclk() {
  FLASH->ACR = 5;                                                        // flash memory cannot repond fast enough to 168mhz. 
  RCC->CR |= RCC_CR_HSEON;                                               // enable the HSE clock peripheral
  while(!(RCC->CR & RCC_CR_HSERDY));                                     // wait until HSE is fully ready/non-busy
  
  RCC->CR &= ~RCC_CR_PLLON;                                              // disable PLL before configuration 

  // hardcoded to generate 168mhz (needs to be generalized)
  RCC->PLLCFGR = 
    RCC_PLLCFGR_PLLSRC_HSE |
    RCC_PLLCFGR_PLLM_3     |                                             // PLLM (constrains VCO input between 1-2mhz)
    RCC_PLLCFGR_PLLN_4     | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_8;    // PLLN (set to 336 ie. double the desired 168mhz)
                                                                         // PLLP (final clock divisor: default to 0, feeds to SYSCLK)

  RCC->CR |= RCC_CR_PLLON;                                               // enable PLL and wait until full ready/non-busy
  while(!(RCC->CR & RCC_CR_PLLRDY));

  RCC->CFGR |= RCC_CFGR_SW_PLL;                                          // switch SYSCLK to pll
  while(((RCC->CFGR >> 2) & 0x03) != 2);                                 // wait until SYSCLK is fully ready (TODO: explain this logic)
//
//                                                                         // delay CPU by 5 cycles between fetches 

}

