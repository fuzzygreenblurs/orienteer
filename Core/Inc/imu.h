#ifndef IMU_H
#define IMU_H

#include <stdint.h>
//#include "stdbool.h"

// 8 complete 12-byte (accel + gyro) binary readings
#define SINGLE_AXIS_READING_BYTES 2
#define MEASURED_AXES             3
#define MODALITIES                2
#define BUFFER_CAPACITY           8
#define ATTITUDE_BUFFER_CAPACITY 16
#define SINGLE_IMU_READING_BYTES  (MODALITIES * MEASURED_AXES * SINGLE_AXIS_READING_BYTES)

typedef struct {
  int16_t ax, ay, az;
  int16_t wx, wy, wz;
} imu_reading_t;

typedef struct {
  float roll, pitch, yaw;
  uint32_t timestamp;
} attitude_t;

// Estimator types
typedef enum {
  ESTIMATOR_SIMPLE_TRIG = 0,  // Simple atan2-based (placeholder)
  ESTIMATOR_COMPLEMENTARY,     // Complementary filter
  ESTIMATOR_EKF,               // Extended Kalman Filter
  ESTIMATOR_UKF                // Unscented Kalman Filter
} estimator_type_t;

// Buffer access API - no direct buffer access
uint8_t* imu_get_raw_buffer(void);
uint8_t* imu_get_raw_write_position(void);
void imu_increment_write_index(void);
attitude_t* imu_get_estimate_buffer(void);
uint8_t imu_get_estimate_write_index(void);
void imu_set_estimate_write_index(uint8_t idx);

void imu_init(void);
void imu_process_latest(void);
attitude_t* imu_get_current_attitude(void);

// Estimator framework
void imu_set_estimator(estimator_type_t type);
void imu_run_estimator(uint8_t* raw_data);

#endif
