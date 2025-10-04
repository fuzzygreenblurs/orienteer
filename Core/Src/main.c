#include "stm32f4xx.h"
#include "clk_config.h"
#include "uart.h"
#include "i2c.h"
#include "imu.h"
#include "dummy_led_helpers.h"

int main(void)
{
  configure_sysclk();

  // Initialize all peripherals
  uart_init();
  uart_init_dma();
  i2c_init();
  imu_init();  // This starts the 400Hz timer

  enable_onboard_led();
  toggle_led(3, 100); // Brief LED indication

  // Everything now runs via interrupts:
  // Timer ISR (400Hz) -> I2C DMA -> DMA ISR -> EKF ISR (100Hz) -> UART DMA
  while(1){
    __WFI();  // Wait for interrupt - CPU idle
  }
}

