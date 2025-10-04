#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init();
void i2c_init_dma();
void i2c_en_clks();
void i2c_setup_pins();
void i2c_set_fast_mode();

void i2c_read_burst(
  uint8_t dev_bus_addr,
  uint8_t dev_reg_addr,
  uint16_t num_bytes
);

#endif
