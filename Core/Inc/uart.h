#ifndef UART_H
#define UART_H

void configure_uart();
void uart_send_char(int c);
void uart_send_string(const char* str);

#endif
