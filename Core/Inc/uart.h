#ifndef UART_H
#define UART_H

void configure_uart();
void uart_init();
void uart_init_dma();
void uart_transmit_attitude();
void uart_send_char(int c);
void uart_send_string(const char* str);
void uart_send_int(int32_t value);

#endif
