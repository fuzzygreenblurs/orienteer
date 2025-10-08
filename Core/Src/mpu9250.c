#include "mpu9250.h"
#include "i2c.h"
#include "uart.h"

bool mpu9250_init(void) {
    uart_send_string("\r\n=== MPU9250 Initialization ===\r\n");

    // Check if MPU9250 is present
    if(!i2c_ping(MPU9250_ADDR)) {
        uart_send_string("MPU9250 not found at 0x68\r\n");
        return false;
    }

    // Read WHO_AM_I
    uint8_t who_am_i = i2c_read_reg(MPU9250_ADDR, MPU9250_WHO_AM_I);
    uart_send_string("WHO_AM_I: 0x");
    uart_send_char((who_am_i >> 4) < 10 ? '0' + (who_am_i >> 4) : 'A' + (who_am_i >> 4) - 10);
    uart_send_char((who_am_i & 0xF) < 10 ? '0' + (who_am_i & 0xF) : 'A' + (who_am_i & 0xF) - 10);
    uart_send_string("\r\n");

    if(who_am_i != 0x71 && who_am_i != 0x73) {
        uart_send_string("Wrong WHO_AM_I (expected 0x71 or 0x73)\r\n");
        return false;
    }

    uart_send_string("MPU9250 detected!\r\n");

    // Give MPU9250 time to power up (100ms)
    for(volatile uint32_t i = 0; i < 2000000; i++);

    // 1. Wake up MPU9250 (clear sleep bit)
    i2c_write_reg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00);
    for(volatile uint32_t i = 0; i < 200000; i++);
    uart_send_string("MPU9250 awake\r\n");

    // 2. Set DLPF to 41Hz (good balance for flight controller)
    i2c_write_reg(MPU9250_ADDR, MPU9250_CONFIG, 0x03);
    uart_send_string("DLPF set to 41Hz\r\n");

    // 3. Set sample rate to 500Hz (1kHz / (1 + 1))
    i2c_write_reg(MPU9250_ADDR, MPU9250_SMPLRT_DIV, 0x01);
    uart_send_string("Sample rate: 500Hz\r\n");

    // 4. Enable I2C bypass to access magnetometer
    i2c_write_reg(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x02);
    for(volatile uint32_t i = 0; i < 100000; i++);
    uart_send_string("I2C bypass enabled\r\n");

    // 5. Check if magnetometer is accessible
    if(!i2c_ping(AK8963_ADDR)) {
        uart_send_string("Magnetometer not found at 0x0C\r\n");
        return false;
    }

    // Read magnetometer WHO_AM_I
    uint8_t mag_id = i2c_read_reg(AK8963_ADDR, AK8963_WHO_AM_I);
    uart_send_string("Mag WHO_AM_I: 0x");
    uart_send_char((mag_id >> 4) < 10 ? '0' + (mag_id >> 4) : 'A' + (mag_id >> 4) - 10);
    uart_send_char((mag_id & 0xF) < 10 ? '0' + (mag_id & 0xF) : 'A' + (mag_id & 0xF) - 10);
    uart_send_string("\r\n");

    if(mag_id != 0x48) {
        uart_send_string("Wrong mag ID (expected 0x48)\r\n");
        return false;
    }

    // 6. Set magnetometer to continuous measurement mode 2 (100Hz, 16-bit)
    i2c_write_reg(AK8963_ADDR, AK8963_CNTL1, 0x16);
    for(volatile uint32_t i = 0; i < 100000; i++);
    uart_send_string("Magnetometer in continuous mode (100Hz)\r\n");

    uart_send_string("MPU9250 initialization complete!\r\n\r\n");
    return true;
}

bool mpu9250_read_accel_gyro(int16_t* ax, int16_t* ay, int16_t* az,
                              int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t data[14];

    // Read 14 bytes starting from ACCEL_XOUT_H using simple register reads
    // (i2c_read_burst with DMA might not work for this)
    for(int i = 0; i < 14; i++) {
        data[i] = i2c_read_reg(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H + i);
    }

    // Parse accelerometer (big-endian)
    *ax = (int16_t)((data[0] << 8) | data[1]);
    *ay = (int16_t)((data[2] << 8) | data[3]);
    *az = (int16_t)((data[4] << 8) | data[5]);
    // Skip temperature data[6:7]

    // Parse gyroscope (big-endian)
    *gx = (int16_t)((data[8] << 8) | data[9]);
    *gy = (int16_t)((data[10] << 8) | data[11]);
    *gz = (int16_t)((data[12] << 8) | data[13]);

    return true;
}

bool mpu9250_read_mag(int16_t* mx, int16_t* my, int16_t* mz) {
    uint8_t data[7];

    // Read 7 bytes starting from HXL using simple register reads
    for(int i = 0; i < 7; i++) {
        data[i] = i2c_read_reg(AK8963_ADDR, AK8963_HXL + i);
    }

    // Check if data is ready (ST2 bit 3 should be 0, bit 4 should be 0 for valid data)
    if(data[6] & 0x08) {
        // Magnetic sensor overflow
        return false;
    }

    // Parse magnetometer (little-endian, unlike MPU9250!)
    *mx = (int16_t)((data[1] << 8) | data[0]);
    *my = (int16_t)((data[3] << 8) | data[2]);
    *mz = (int16_t)((data[5] << 8) | data[4]);

    return true;
}

bool mpu9250_read_all(mpu9250_data_t* data) {
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

    // Read accel + gyro
    if(!mpu9250_read_accel_gyro(&ax, &ay, &az, &gx, &gy, &gz)) {
        return false;
    }

    // Read magnetometer
    if(!mpu9250_read_mag(&mx, &my, &mz)) {
        return false;
    }

    // Convert to physical units
    data->ax = (float)ax / ACCEL_SCALE_RANGE;
    data->ay = (float)ay / ACCEL_SCALE_RANGE;
    data->az = (float)az / ACCEL_SCALE_RANGE;

    data->gx = (float)gx / GYRO_SCALE_RANGE;
    data->gy = (float)gy / GYRO_SCALE_RANGE;
    data->gz = (float)gz / GYRO_SCALE_RANGE;

    data->mx = (float)mx * MAG_SCALE_RANGE;
    data->my = (float)my * MAG_SCALE_RANGE;
    data->mz = (float)mz * MAG_SCALE_RANGE;

    return true;
}
