#include <stdbool.h>
#include <math.h>
#include "stm32f4xx.h"
#include "i2c.h"
#include "uart.h"
#include "imu.h"
#include "mpu9250.h"

// Buffer constants defined in imu.h

static volatile uint8_t imu_raw[BUFFER_CAPACITY * SINGLE_IMU_READING_BYTES];
static volatile uint8_t imu_raw_write_idx = 0;
static volatile uint8_t imu_raw_sample_count = 0;

static volatile attitude_t imu_estimate[BUFFER_CAPACITY];
static volatile uint8_t imu_estimate_write_idx = 0;
static volatile uint8_t imu_estimate_read_idx = 0;

static volatile bool ekf_ready = 0;
static attitude_t current_attitude = {0};

void imu_init(void) {
  // generates the requisite 400Hz sampling ticker
     
  RCC->APB1ENR |= (1 << 0);                       // en TIM2  
  
  TIM2->PSC = 419;                                // prescaler: 42MHz/420 = 100Khz
  TIM2->ARR = 249;                                // auto-reload: 100Khz/250 = 400Hz

  TIM2->DIER |= (1 << 0);                         // enable update INT
  NVIC_EnableIRQ(TIM2_IRQn);                      // enable in NVIC
  NVIC_SetPriority(TIM2_IRQn, 0);                 // set highest priority

  TIM2->CR1 |= (1 << 0);                          // start timer 
}

void imu_canonicalize(uint8_t* raw_data) {
  // convert and reformat raw reading into metric values
  
  int16_t ax_bin = (raw_data[0]  << 8 | raw_data[1]);
  int16_t ay_bin = (raw_data[2]  << 8 | raw_data[3]);
  int16_t az_bin = (raw_data[4]  << 8 | raw_data[5]);

  int16_t wx_bin = (raw_data[6]  << 8 | raw_data[7]);
  int16_t wy_bin = (raw_data[8]  << 8 | raw_data[9]);
  int16_t wz_bin = (raw_data[10] << 8 | raw_data[11]);

  // Convert to float for attitude calculation
  float ax_g = (float)ax_bin / ACCEL_SCALE_RANGE;
  float ay_g = (float)ay_bin / ACCEL_SCALE_RANGE;
  float az_g = (float)az_bin / ACCEL_SCALE_RANGE;

  float wx_dps = (float)wx_bin / GYRO_SCALE_RANGE;
  float wy_dps = (float)wy_bin / GYRO_SCALE_RANGE;
  float wz_dps = (float)wz_bin / GYRO_SCALE_RANGE;

  (void)wx_dps; // TODO: Use in full EKF implementation
  (void)wy_dps; // TODO: Use in full EKF implementation

  // Simple attitude calculation (placeholder for real EKF)
  current_attitude.roll = atan2f(ay_g, az_g) * 180.0f / M_PI;
  current_attitude.pitch = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / M_PI;
  current_attitude.yaw += wz_dps * 0.01f; // Simple integration (100Hz rate)
}

// Buffer access API implementations
uint8_t* imu_get_raw_buffer(void) {
    return (uint8_t*)imu_raw;
}

uint8_t* imu_get_raw_write_position(void) {
    return (uint8_t*)&imu_raw[imu_raw_write_idx * SINGLE_IMU_READING_BYTES];
}

void imu_increment_write_index(void) {
    imu_raw_write_idx++;
    if(imu_raw_write_idx >= BUFFER_CAPACITY) imu_raw_write_idx = 0;
}

attitude_t* imu_get_estimate_buffer(void) {
    return (attitude_t*)imu_estimate;
}

uint8_t imu_get_estimate_write_index(void) {
    return imu_estimate_write_idx;
}

void imu_set_estimate_write_index(uint8_t idx) {
    imu_estimate_write_idx = idx;
}

void imu_process_latest(void) {
    // Get latest reading from ring buffer
    uint8_t read_idx = (imu_raw_write_idx == 0) ? 7 : imu_raw_write_idx - 1;
    volatile uint8_t* raw = &imu_raw[read_idx * SINGLE_IMU_READING_BYTES];

    // Process raw data and run EKF
    imu_canonicalize((uint8_t*)raw);

    // TODO: Add actual EKF/filter logic here
    // For now, just update timestamp
    current_attitude.timestamp = 0; // TODO: Get real timestamp
}

attitude_t* imu_get_current_attitude(void) {
    return &current_attitude;
}

void TIM2_IRQHandler(void) {
  // triggers reads from IMU based on set TIM2 frequency

  if(TIM2->SR & (1 << 0)) {                       // check and clear update INT flag(if set) 
    TIM2->SR &= ~(1 << 0);
  }

  i2c_read_burst(0x68, 0x3B, 12);                 // request IMU for a reading
}

void DMA1_Stream0_IRQHandler(void) {
  static uint8_t sample_counter = 0;

  if(DMA1->LISR &  (1<<5)) {                      // DMA "transfer complete" event raises TCIF0 int flag
    DMA1->LIFCR |= (1<<5);                        // clear TCIFO int flag

    //send NACK+STOP to end I2C transaction 
    I2C1->CR1 &= ~(1 << 10);                      // NACK bit
    I2C1->CR1 |=  (1 <<  9);                      // STOP bit

    //wrap pointers around FIFO
    imu_raw_write_idx++;  
    if(imu_raw_write_idx >= BUFFER_CAPACITY) imu_raw_write_idx = 0;
    sample_counter++;
  }

  // set read frequency to 1/4 of the write frequency (100hz)
  if(sample_counter >= 4) {                     
    sample_counter = 0;
    NVIC_SetPendingIRQ(EXTI0_IRQn);               // trigger attitude estimator read and process steps
    ekf_ready = 1;                                
  }
}

// EXTI0_IRQHandler moved to main.c for interrupt-driven approach
