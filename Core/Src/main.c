#include "stm32f4xx.h"
#include "clk_config.h"
#include "uart.h"
#include "i2c.h"
#include "dummy_led_helpers.h"

int main(void)
{
  configure_sysclk();
  configure_uart();
  configure_i2c();
 
  enable_onboard_led();
  toggle_led(200, 100); // toggle 200 times, ~100ms delay (no delay currently)

  // configure i2c + dma 

  while(1){
    uart_send_string("hw, my name is fuzzy\r\n");
    delay(1000);
  }
}

