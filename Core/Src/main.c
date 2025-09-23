#include "stm32f4xx.h"
#include "clk_config.h"

#define TICKS_PER_MS 21000

void configure_uart() {
  RCC->AHB1ENR |= 1;
  RCC->APB1ENR |= 0x20000;

  GPIOA->AFR[0] &= ~0x0F00;
  GPIOA->AFR[0] |=  0x0700;
  GPIOA->MODER  &= ~0x0030;
  GPIOA->MODER  |=  0x0020;

  USART2->BRR  = 4375;  // 42MHz / 9600 = 4375 (when PLL enabled)
  USART2->CR1  = 0x0008;
  USART2->CR2  = 0x0000;
  USART2->CR3  = 0x0000;
  USART2->CR1 |= 0x2000;
}


//void configure_uart() {
//  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;     // powers GPIOA port peripheral (pin control, alt function routing)
//  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;    // powers USART2 peripheral logic
//                                           // enabling this clock will enable power to gpio port A.
//
//  GPIOA->AFR[0] &= ~0x0f00;
//  GPIOA->AFR[0] |= 0x0700;                  // set PA2 to AF7 (USART2, see datasheet, pg 57)
//    
//  // USART2 uses GPIOA: PA2 and PA3 pins under AF mode
//  GPIOA->MODER &= ~GPIO_MODER_MODER2;      // clear PA2 mode bits and set to AF mode (10)
//  GPIOA->MODER |= GPIO_MODER_MODER2_1;
//
//
//  USART2->BRR = 0x0683;                    // APB clock running at 42MHz
//                                           // desired baud rate is 115200: BRR = 42M/115200
//                                           // transmit/read a bit every 365 ticks, enforced by HW counter
//
//  USART2->CR1 = 0x0008;                   // enable tx, 8-bit data 
//  USART2->CR2 = 0x0000;                   // 1 stop bit 
//  USART2->CR3 = 0x0000;                   // no flow control
//  USART2->CR1 = 0X2000;                   // enable USART2 
//}

void uart_send_char(int c) {
  while(!(USART2->SR & 0x0080));     // wait until TX buffer is empty
  USART2->DR = (c & 0xFF);
}
void delay(int n) {
  for(; n > 0; n--) {
    for(volatile int i = 0; i < 2000; i++);
  }
}

void enable_onboard_led() {
  GPIOA->MODER  |= GPIO_MODER_MODER5_0;    // sets pa5 to general purpose output mode
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;      // sets pa5 to push-pull output mode: equivalent to &= ~(1 << 5);
  GPIOA->ODR    |= (1 << 5);               // sets the pa5 bit on the output data register (ODR) to turn on LED
}

void toggle_led(uint8_t times, uint32_t ms) {
  while(times > 0) {
    GPIOA->ODR ^= (1 << 5);
//    for(uint32_t i = 0; i < (ms * TICKS_PER_MS); i++) {
//      __asm("nop");
//    }

    --times;
  }
}

void uart_send_string(const char* str) {
  while(*str) {
    uart_send_char(*str++);
  }
}


int main(void)
{
  configure_sysclk();
  configure_uart();

  while(1){
    uart_send_string("hello_world, ");
    uart_send_string("my name is fuzzy\r\n");
    delay(1000);
  }
}

