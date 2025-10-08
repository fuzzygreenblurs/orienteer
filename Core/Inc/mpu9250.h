#ifndef __MPU9250_H
#define __MPU9250_H

#include <stdint.h>
#include <stdbool.h>

// I2C Addresses
#define MPU9250_ADDR              0x68
#define AK8963_ADDR               0x0C

// MPU9250 Registers
#define MPU9250_WHO_AM_I          0x75
#define MPU9250_PWR_MGMT_1        0x6B
#define MPU9250_CONFIG            0x1A
#define MPU9250_SMPLRT_DIV        0x19
#define MPU9250_INT_PIN_CFG       0x37
#define MPU9250_ACCEL_XOUT_H      0x3B
#define MPU9250_GYRO_XOUT_H       0x43

// AK8963 Magnetometer Registers
#define AK8963_WHO_AM_I           0x00
#define AK8963_CNTL1              0x0A
#define AK8963_HXL                0x03
#define AK8963_ST2                0x09

// Scale factors
#define ACCEL_SCALE_RANGE         16384.0f   // +/-2g => 16,384 LSB/g
#define GYRO_SCALE_RANGE          131.0f     // +/-250 dps => 131 LSB/dps
#define MAG_SCALE_RANGE           0.15f      // 14-bit mode: 0.15 uT/LSB

// 9-DOF data structure
typedef struct {
    float ax, ay, az;        // Accelerometer (g)
    float gx, gy, gz;        // Gyroscope (deg/s)
    float mx, my, mz;        // Magnetometer (uT)
} mpu9250_data_t;

// Function declarations
bool mpu9250_init(void);
bool mpu9250_read_accel_gyro(int16_t* ax, int16_t* ay, int16_t* az,
                              int16_t* gx, int16_t* gy, int16_t* gz);
bool mpu9250_read_mag(int16_t* mx, int16_t* my, int16_t* mz);
bool mpu9250_read_all(mpu9250_data_t* data);

#endif

