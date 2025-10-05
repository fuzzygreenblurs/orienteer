#ifndef I2C_H
#define I2C_H

#include <stdbool.h>
#include <stdint.h>

void i2c_init();
void i2c_init_dma();
void i2c_en_clks();
void i2c_setup_pins();
void i2c_set_fast_mode();
bool i2c_ping(uint8_t addr);
uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr);

void i2c_read_burst(
  uint8_t dev_bus_addr,
  uint8_t dev_reg_addr,
  uint16_t num_bytes
);

uint8_t i2c_device_present(uint8_t dev_addr); // Returns 1 if device responds, 0 if not

#endif
