#include "stm32f4xx.h"
#include "clk_config.h"
#include "uart.h"
#include "i2c.h"
#include "imu.h"
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

  uart_send_string("\r\n=== MPU6050 Detection ===\r\n");

  i2c_init();
  uart_send_string("I2C initialized\r\n");

  if(i2c_ping(0x68)) {
    uart_send_string("MPU6050 found at 0x68\r\n");
    toggle_led(2);

    // Read WHO_AM_I register (0x75)
    uint8_t who_am_i = i2c_read_reg(0x68, 0x75);
    uart_send_string("WHO_AM_I: 0x");
    uart_send_char((who_am_i >> 4) < 10 ? '0' + (who_am_i >> 4) : 'A' + (who_am_i >> 4) - 10);
    uart_send_char((who_am_i & 0xF) < 10 ? '0' + (who_am_i & 0xF) : 'A' + (who_am_i & 0xF) - 10);
    uart_send_string("\r\n");

    if(who_am_i == 0x68) {
      uart_send_string("MPU6050 verified!\r\n");
      toggle_led(1);
    } else {
      uart_send_string("Wrong WHO_AM_I value\r\n");
      toggle_led(8);
    }
  } else {
    uart_send_string("MPU6050 not found\r\n");
    toggle_led(4);
  }

  uart_send_string("Ready to start interrupt-driven mode\r\n");

  // Configure MPU6050 for interrupt-driven mode
  uart_send_string("\r\nConfiguring MPU6050 interrupts...\r\n");

  // Give MPU6050 time to power up (100ms)
  for(volatile uint32_t i = 0; i < 2000000; i++);

  // 1. Wake up MPU6050 (clear sleep bit)
  i2c_write_reg(0x68, 0x6B, 0x00);
  for(volatile uint32_t i = 0; i < 200000; i++); // 10ms delay
  uart_send_string("MPU6050 awake\r\n");

  // 2. Set sample rate to 500Hz (1kHz / (1 + 1))
  i2c_write_reg(0x68, 0x19, 0x01);
  uart_send_string("Sample rate: 500Hz\r\n");

  // 3. Configure INT pin: active high, push-pull, clear on any read
  i2c_write_reg(0x68, 0x37, 0x10);
  uart_send_string("INT pin configured\r\n");

  // 4. Enable data ready interrupt
  i2c_write_reg(0x68, 0x38, 0x01);
  uart_send_string("Data ready interrupt enabled\r\n");

  uart_send_string("MPU6050 should now be generating interrupts on PA0\r\n");

  // Configure EXTI0 to catch MPU6050 interrupts
  uart_send_string("\r\nConfiguring EXTI0...\r\n");

  // 1. Configure PA0 as input
  RCC->AHB1ENR |= (1 << 0);        // Enable GPIOA clock
  GPIOA->MODER &= ~(3 << 0);       // PA0 as input (00)
  GPIOA->PUPDR &= ~(3 << 0);       // No pull-up/pull-down

  // 2. Connect EXTI0 to PA0
  RCC->APB2ENR |= (1 << 14);       // Enable SYSCFG clock
  SYSCFG->EXTICR[0] &= ~(0xF << 0); // Clear EXTI0 config
  SYSCFG->EXTICR[0] |= (0x0 << 0);  // Connect EXTI0 to PA0 (0x0 = GPIOA)

  // 3. Configure EXTI0 for rising edge trigger
  EXTI->IMR |= (1 << 0);           // Unmask EXTI0
  EXTI->RTSR |= (1 << 0);          // Rising edge trigger
  EXTI->FTSR &= ~(1 << 0);         // Disable falling edge

  // 4. Enable EXTI0 interrupt in NVIC
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_SetPriority(EXTI0_IRQn, 0); // Highest priority

  uart_send_string("EXTI0 configured and enabled\r\n");
  uart_send_string("System ready - interrupts should be firing\r\n\r\n");

  // Select estimator type
  imu_set_estimator(ESTIMATOR_COMPLEMENTARY);
  uart_send_string("Using COMPLEMENTARY filter\r\n\r\n");

  uint32_t last_ekf = 0;

  while(1){
    // Print attitude at ~10Hz (100ms intervals)
    for(volatile uint32_t i = 0; i < 200000; i++);

    if(ekf_count != last_ekf) {
      attitude_t* att = imu_get_current_attitude();

      uart_send_string("EKF:");
      uart_send_int(ekf_count);
      uart_send_string(" R:");
      uart_send_int((int32_t)att->roll);
      uart_send_string(" P:");
      uart_send_int((int32_t)att->pitch);
      uart_send_string(" Y:");
      uart_send_int((int32_t)att->yaw);
      uart_send_string("\r\n");

      last_ekf = ekf_count;
    }
  }
}

