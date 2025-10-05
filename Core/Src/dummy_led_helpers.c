#include "stm32f4xx.h"
#include "dummy_led_helpers.h"

void enable_onboard_led() {
  RCC->AHB1ENR |= (1 << 0);                // Enable GPIOA clock
  GPIOA->MODER  |= GPIO_MODER_MODER5_0;    // pa5 -> GP output mode
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;      // pa5 -> push-pull output mode: equivalent to &= ~(1 << 5);
  GPIOA->ODR    &= ~(1 << 5);               // sets pa5 on the output data register (ODR) to turn on LED
}

void delay(){
  for(volatile uint32_t i = 0; i < 500000; i++) {
    __asm("nop");
  }
}

void toggle_led(uint8_t times) {
  while(times > 0) {

    GPIOA->ODR |= (1 << 5);
    delay();
    
    GPIOA->ODR &= ~(1 << 5);
    delay();
    
    --times;
  }
}
