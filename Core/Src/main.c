#include "stm32f4xx.h"
#include "clk_config.h"
#include "uart.h"
#include "i2c.h"
#include "imu.h"
#include "mpu9250.h"
#include "dummy_led_helpers.h"

volatile uint32_t exti_count = 0;
volatile uint32_t dma_count = 0;
volatile uint32_t exti_dropped = 0;
volatile uint32_t ekf_count = 0;

// Simple buffer to hold one IMU reading (12 bytes: 6 accel + 6 gyro)
uint8_t imu_buffer[12];

void EXTI0_IRQHandler(void) {
  EXTI->PR |= (1 << 0);  // Clear pending flag
  exti_count++;

  // Check if DMA is still busy from previous read
  if(DMA1_Stream0->CR & (1 << 0)) {  // DMA enabled = busy
    exti_dropped++;
    return;
  }

  // Trigger I2C+DMA burst read of 12 bytes from register 0x3B
  i2c_read_burst(0x68, 0x3B, imu_buffer, 12);
}

void DMA1_Stream0_IRQHandler(void) {
  static uint8_t sample_counter = 0;

  if(DMA1->LISR & (1 << 5)) {  // Transfer complete
    DMA1->LIFCR |= (0x3F << 0);  // Clear all Stream0 flags

    dma_count++;

    // Disable DMA stream
    DMA1_Stream0->CR &= ~(1 << 0);

    // Send I2C STOP condition
    I2C1->CR1 |= (1 << 9);

    // Wait briefly for STOP to complete
    uint32_t timeout = 1000;
    while((I2C1->SR2 & (1 << 1)) && timeout > 0) timeout--;

    // Disable I2C DMA
    I2C1->CR2 &= ~(1 << 11);
    I2C1->CR2 &= ~(1 << 12);

    // Run estimator every 2nd sample (250Hz -> 125Hz EKF rate)
    sample_counter++;
    if(sample_counter >= 2) {
      sample_counter = 0;
      ekf_count++;

      // Run the estimator on the raw data
      imu_run_estimator(imu_buffer);
    }

    // Toggle LED to show DMA activity
    GPIOA->ODR ^= (1 << 5);
  }
}

int main(void)
{
  configure_sysclk();  // Enable PLL for 168MHz system clock
  uart_init();
  enable_onboard_led();

  uart_send_string("\r\n=== MPU9250 9-DOF Test ===\r\n");

  i2c_init();

  // Scan I2C bus first
  uart_send_string("\r\n=== Scanning I2C Bus ===\r\n");
  uint8_t devices_found = 0;
  for(uint8_t addr = 0x00; addr < 0x80; addr++) {
    if(i2c_ping(addr)) {
      uart_send_string("Device at 0x");
      uart_send_char((addr >> 4) < 10 ? '0' + (addr >> 4) : 'A' + (addr >> 4) - 10);
      uart_send_char((addr & 0xF) < 10 ? '0' + (addr & 0xF) : 'A' + (addr & 0xF) - 10);
      uart_send_string("\r\n");
      devices_found++;
    }
  }

  if(devices_found == 0) {
    uart_send_string("No I2C devices found!\r\n");
    uart_send_string("Check wiring:\r\n");
    uart_send_string("  SCL: PB6\r\n");
    uart_send_string("  SDA: PB7\r\n");
    uart_send_string("  VCC: 3.3V\r\n");
    uart_send_string("  GND: GND\r\n");
    toggle_led(8);
    while(1);
  }

  uart_send_string("Found ");
  uart_send_int(devices_found);
  uart_send_string(" device(s)\r\n\r\n");

  // Initialize MPU9250
  if(!mpu9250_init()) {
    uart_send_string("MPU9250 initialization failed!\r\n");
    uart_send_string("Check wiring and power.\r\n");
    toggle_led(8);
    while(1);  // Halt
  }

  toggle_led(2);
  uart_send_string("Starting 9-DOF data streaming...\r\n\r\n");

  // Main loop: read and print 9-DOF data at ~10Hz
  while(1) {
    mpu9250_data_t data;

    if(mpu9250_read_all(&data)) {
      // Print accelerometer (g)
      uart_send_string("A: ");
      uart_send_int((int32_t)(data.ax * 1000));  // Convert to milli-g
      uart_send_string(" ");
      uart_send_int((int32_t)(data.ay * 1000));
      uart_send_string(" ");
      uart_send_int((int32_t)(data.az * 1000));

      // Print gyroscope (deg/s)
      uart_send_string(" | G: ");
      uart_send_int((int32_t)data.gx);
      uart_send_string(" ");
      uart_send_int((int32_t)data.gy);
      uart_send_string(" ");
      uart_send_int((int32_t)data.gz);

      // Print magnetometer (uT)
      uart_send_string(" | M: ");
      uart_send_int((int32_t)data.mx);
      uart_send_string(" ");
      uart_send_int((int32_t)data.my);
      uart_send_string(" ");
      uart_send_int((int32_t)data.mz);

      uart_send_string("\r\n");

      // Toggle LED to show activity
      GPIOA->ODR ^= (1 << 5);
    } else {
      uart_send_string("Read error\r\n");
    }

    // Delay ~100ms (10Hz update rate)
    for(volatile uint32_t i = 0; i < 2000000; i++);
  }
}

