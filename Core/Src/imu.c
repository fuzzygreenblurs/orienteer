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
static estimator_type_t current_estimator = ESTIMATOR_SIMPLE_TRIG;

void imu_set_estimator(estimator_type_t type) {
  current_estimator = type;
}

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

// Estimator implementations

static void estimator_simple_trig(uint8_t* raw_data) {
  // Extract raw sensor values (big-endian 16-bit)
  int16_t ax_bin = (raw_data[0] << 8) | raw_data[1];
  int16_t ay_bin = (raw_data[2] << 8) | raw_data[3];
  int16_t az_bin = (raw_data[4] << 8) | raw_data[5];

  int16_t wx_bin = (raw_data[6] << 8) | raw_data[7];
  int16_t wy_bin = (raw_data[8] << 8) | raw_data[9];
  int16_t wz_bin = (raw_data[10] << 8) | raw_data[11];

  // Convert to metric units (g's and deg/s)
  // MPU6050 default: ±2g range = 16384 LSB/g, ±250°/s = 131 LSB/°/s
  float ax_g = (float)ax_bin / 16384.0f;
  float ay_g = (float)ay_bin / 16384.0f;
  float az_g = (float)az_bin / 16384.0f;

  float wx_dps = (float)wx_bin / 131.0f;
  float wy_dps = (float)wy_bin / 131.0f;
  float wz_dps = (float)wz_bin / 131.0f;

  (void)wx_dps; (void)wy_dps; // Not used in simple trig

  // Simple trigonometric attitude calculation
  current_attitude.roll = atan2f(ay_g, az_g) * 180.0f / M_PI;
  current_attitude.pitch = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / M_PI;
  current_attitude.yaw += wz_dps * 0.008f; // Integrate yaw (125Hz = 0.008s)
  current_attitude.timestamp++;
}

static void estimator_complementary(uint8_t* raw_data) {
  // TODO: Implement complementary filter
  (void)raw_data;
  current_attitude.timestamp++;
}

static void estimator_ekf(uint8_t* raw_data) {
  // TODO: Implement EKF
  (void)raw_data;
  current_attitude.timestamp++;
}

static void estimator_ukf(uint8_t* raw_data) {
  // TODO: Implement UKF
  (void)raw_data;
  current_attitude.timestamp++;
}

void imu_run_estimator(uint8_t* raw_data) {
  switch(current_estimator) {
    case ESTIMATOR_SIMPLE_TRIG:
      estimator_simple_trig(raw_data);
      break;

    case ESTIMATOR_COMPLEMENTARY:
      estimator_complementary(raw_data);
      break;

    case ESTIMATOR_EKF:
      estimator_ekf(raw_data);
      break;

    case ESTIMATOR_UKF:
      estimator_ukf(raw_data);
      break;

    default:
      estimator_simple_trig(raw_data);
      break;
  }
}

// Old timer-based handlers - not used in interrupt-driven approach
// TIM2_IRQHandler, DMA1_Stream0_IRQHandler, and EXTI0_IRQHandler moved to main.c
