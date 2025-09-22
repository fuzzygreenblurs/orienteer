#ifndef __MPU9250_H
#define __MPU9250_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// MPU9250 I2C Address (when AD0 is low)
#define MPU9250_ADDR 0x68 << 1  // Shifted for HAL I2C functions

// Key MPU9250 Register Addresses
#define MPU9250_WHO_AM_I        0x75
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_PWR_MGMT_2      0x6C
#define MPU9250_CONFIG          0x1A
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_ACCEL_CONFIG2   0x1D
#define MPU9250_SMPLRT_DIV      0x19

// Data registers (start of accelerometer data)
#define MPU9250_ACCEL_XOUT_H    0x3B
#define MPU9250_GYRO_XOUT_H     0x43
#define MPU9250_TEMP_OUT_H      0x41

// Expected WHO_AM_I response
#define MPU9250_WHO_AM_I_VAL    0x71

// Magnetometer (AK8963) I2C address
#define AK8963_ADDR             0x0C << 1
#define AK8963_WHO_AM_I         0x00
#define AK8963_ST1              0x02
#define AK8963_HXL              0x03
#define AK8963_CNTL             0x0A

// Data structures for raw sensor readings
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu9250_raw_data_t;

typedef struct {
    float x;
    float y;
    float z;
} mpu9250_float_data_t;

// Complete IMU data structure
typedef struct {
    mpu9250_raw_data_t accel_raw;
    mpu9250_raw_data_t gyro_raw;
    mpu9250_raw_data_t mag_raw;
    int16_t temp_raw;

    mpu9250_float_data_t accel_g;      // Acceleration in g's
    mpu9250_float_data_t gyro_dps;     // Angular velocity in degrees/sec
    mpu9250_float_data_t mag_ut;       // Magnetic field in microTesla
    float temp_c;                       // Temperature in Celsius

    uint32_t timestamp;                 // Timestamp from TIM2
    bool data_ready;                    // Flag for new data
} mpu9250_data_t;

// Configuration enums
typedef enum {
    MPU9250_ACCEL_FS_2G = 0,
    MPU9250_ACCEL_FS_4G,
    MPU9250_ACCEL_FS_8G,
    MPU9250_ACCEL_FS_16G
} mpu9250_accel_fs_t;

typedef enum {
    MPU9250_GYRO_FS_250DPS = 0,
    MPU9250_GYRO_FS_500DPS,
    MPU9250_GYRO_FS_1000DPS,
    MPU9250_GYRO_FS_2000DPS
} mpu9250_gyro_fs_t;

// Function prototypes
bool mpu9250_init(I2C_HandleTypeDef *hi2c);
bool mpu9250_who_am_i_check(I2C_HandleTypeDef *hi2c);
bool mpu9250_configure(I2C_HandleTypeDef *hi2c, mpu9250_accel_fs_t accel_fs, mpu9250_gyro_fs_t gyro_fs);
bool mpu9250_read_all_sensors(I2C_HandleTypeDef *hi2c, mpu9250_data_t *data);
bool mpu9250_read_raw_data(I2C_HandleTypeDef *hi2c, mpu9250_data_t *data);
void mpu9250_convert_raw_to_float(mpu9250_data_t *data, mpu9250_accel_fs_t accel_fs, mpu9250_gyro_fs_t gyro_fs);

// Magnetometer functions
bool ak8963_init(I2C_HandleTypeDef *hi2c);
bool ak8963_read_data(I2C_HandleTypeDef *hi2c, mpu9250_raw_data_t *mag_data);

#ifdef __cplusplus
}
#endif

#endif /* __MPU9250_H */