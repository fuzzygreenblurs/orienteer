#include "stm32f4xx.h"
#include "clk_config.h"
#include "uart.h"
#include "i2c.h"
#include "imu.h"
#include "dummy_led_helpers.h"

int main(void)
{
//  configure_sysclk();

  // Initialize all peripherals
//  uart_init();
//  uart_init_dma();
//  i2c_init();
//  imu_init();  // This starts the 400Hz timer

  enable_onboard_led();
  toggle_led(3); // Brief LED indication

  i2c_init();
  if(i2c_ping(0x68)) {
    toggle_led(2);  // Found at 0x68

    // Read WHO_AM_I register (0x75)
    uint8_t who_am_i = i2c_read_reg(0x68, 0x75);

    if(who_am_i == 0x68) {
      toggle_led(1);  // Correct WHO_AM_I (0x68)
    } else {
      toggle_led(8);  // Wrong WHO_AM_I value
    }
  } else {
    toggle_led(4);  // Not found
  }
  // Everything now runs via interrupts:
  // Timer ISR (400Hz) -> I2C DMA -> DMA ISR -> EKF ISR (100Hz) -> UART DMA
  while(1){
    __WFI();  // Wait for interrupt - CPU idle
  }
}

