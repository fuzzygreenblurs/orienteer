#include <stdbool.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "imu.h"

volatile uint8_t uart_tx_busy = 0;

void uart_init() {
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

void uart_init_dma(void) {
  DMA1_Stream6->CR &= ~(1 << 0);                  // disable DMA stream first to allow config changes
  while(DMA1_Stream6->CR & (1 << 0));             // RM0390 9.5.5: wait for stream to be disabled

  DMA1_Stream6->CR |= (4 << 25);                  // setup USART2_TX (chan 4): RM0390 Table 28 and 9.5.5
  DMA1_Stream6->CR |= (1 << 6);                   // mem-to-peripheral transfer direction
  DMA1_Stream6->CR |= (1 << 10);                  // MINC = 1: mem increment
  DMA1_Stream6->CR |= (1 << 4);                   // TCIE = 1: enable transfer complete INT
 
  DMA1_Stream6->PAR = (uint32_t)&USART2->DR;      // set peripheral addr (dest) to the usart data buffer 
          
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);              // raise interrupt upon completing mem-to-peripheral transfer
  USART2->CR3 |= (1 << 7);                        // enable USART2 DMA TX
}

void uart_transmit_attitude(void) {
  if(!uart_tx_busy) {
    uart_tx_busy = 1;

    DMA1_Stream6->CR &= ~(1 << 0);                    // disable DMA stream first to allow config changes
    while(DMA1_Stream6->CR & (1 << 0));               // RM0390 9.5.5: wait for stream to be disabled

    DMA1_Stream6->M0AR = (uint32_t)imu_get_current_attitude();  // source from current attitude
    DMA1_Stream6->NDTR = sizeof(attitude_t);          // DMA increments over entire reading
    DMA1_Stream6->CR |= (1 << 0);                      // enable DMA1:S6
  }
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

void uart_send_int(int32_t value) {
  char buffer[12];
  int i = 0;

  if(value < 0) {
    uart_send_char('-');
    value = -value;
  }

  // Convert to string (reverse order)
  do {
    buffer[i++] = '0' + (value % 10);
    value /= 10;
  } while(value > 0);

  // Send digits in correct order
  for(int j = i - 1; j >= 0; j--) {
    uart_send_char(buffer[j]);
  }
}

void uart_enqueue_estimate(void) {
  // TODO: Implement using imu API
}

bool is_buffer_empty(void) {
  // TODO: Implement using imu API
  return true;
}

void DMA1_Stream6_IRQHandler(void) {
  if(DMA1->HISR &  (1 << 21)) {                       // RM0390 9.5.2: transfer complete
    DMA1->HIFCR |= (1 << 21);                         // RM0390 9.5.2: clear transfer copmlete status bit
    uart_tx_busy = 0;
  }
  // TODO: Implement using imu API
}
