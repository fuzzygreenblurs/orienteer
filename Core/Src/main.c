#include "stm32f4xx.h"
#include "clk_config.h"

#define TICKS_PER_MS 21000

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

