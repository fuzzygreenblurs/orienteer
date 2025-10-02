#include "uart.h"

void configure_uart() {
  RCC->AHB1ENR |= 1;        // powers GPIOA port peripheral (pin control, alt function routing)
  RCC->APB1ENR |= 0x20000;  // powers USART2 peripheral logic

  GPIOA->AFR[0] &= ~0x0F00;
  GPIOA->AFR[0] |=  0x0700; // set PA2 to AF7 (USART2, see datasheet, pg 57)

  GPIOA->MODER  &= ~0x0030; 
  GPIOA->MODER  |=  0x0020; // clear PA2 mode bits and set to AF mode (10)

  USART2->BRR  = 365;       // 42MHz / 115200 = 365 (when PLL enabled)

  USART2->CR1  = 0x0008;    // enable tx, 8-bit data k
  USART2->CR2  = 0x0000;    // 1 stop bit 
  USART2->CR3  = 0x0000;    // no flow control
  USART2->CR1 |= 0x2000;    // enable USART2 
}

void uart_send_char(int c) {
  while(!(USART2->SR & 0x0080));     // wait until TX buffer is empty
  USART2->DR = (c & 0xFF);
}

void uart_send_string(const char* str) {
  while(*str) {
    uart_send_char(*str++);
  }
}
