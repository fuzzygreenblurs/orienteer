#include "stm32f4xx.h"
#include "i2c.h"
#include "imu.h"

bool i2c_ping(uint8_t addr) {
  I2C1->CR1 |= (1 << 8);                          // set START bit

  uint32_t timeout = 10000;
  while(!(I2C1->SR1 & (1 << 0)) && timeout > 0) { // wait for START condition to be set
    timeout--;
  }
  if(timeout == 0) return 0;                       // START failed

  I2C1->DR = addr << 1;                            // send 7 bit addr
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 1)) && timeout > 0) {  // wait for addr to be sent and ACK response
    timeout--;
  }

  bool dev_found = ( timeout > 0 ) ? 1 : 0;
  if(dev_found) (void)I2C1->SR2;                    // clear ADDR flag if device responds

  I2C1->CR1 |= (1 << 9);                            // always cleanup with STOP bit
  return dev_found;

}

uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr) {
  I2C1->CR1 |= (1 << 8);                  

  uint32_t timeout = 10000;
  while(!(I2C1->SR1 & (1 << 0)) && timeout > 0) { // wait for START condition to be set
    timeout--;
  }
  if(timeout == 0) return 0;                       // START failed

  I2C1->DR = (dev_addr << 1);                            // send 7 bit addr
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 1)) && timeout > 0) {  // wait for addr to be sent and ACK response
    timeout--;
  }
  if(timeout == 0) return 0;
  (void)I2C1->SR2;                                // clear ADDR flag
  
  I2C1->DR = reg_addr;                           
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 7)) && timeout > 0) {  // wait for TxE
    timeout--;
  }
  if(timeout == 0) return 0;

  // REPEATED START for read phase
  I2C1->CR1 |= (1 << 8);  // Set START bit again
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 0)) && timeout > 0) {
    timeout--;
  }
  if(timeout == 0) return 0;

  // Send device address in READ mode
  I2C1->DR = (dev_addr << 1) | 0x01;  // 0x01 sets read bit
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 1)) && timeout > 0) {
    timeout--;
  }
  if(timeout == 0) return 0;
  (void)I2C1->SR2;  // Clear ADDR flag

  // Disable ACK for single byte read and wait for data
  I2C1->CR1 &= ~(1 << 10);  // Clear ACK bit (send NACK)
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 6)) && timeout > 0) { // RxNE = bit 6
    timeout--;
  }
  if(timeout == 0) return 0;

  // Read data and send STOP
  uint8_t data = I2C1->DR;
  I2C1->CR1 |= (1 << 9);  // STOP condition
  return data;
}

void i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
  uint32_t timeout = 10000;

  // START condition
  I2C1->CR1 |= (1 << 8);
  while(!(I2C1->SR1 & (1 << 0)) && timeout > 0) timeout--;
  if(timeout == 0) return;

  // Send device address (write mode)
  I2C1->DR = dev_addr << 1;
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 1)) && timeout > 0) timeout--;
  if(timeout == 0) return;
  (void)I2C1->SR2; // Clear ADDR flag

  // Send register address
  I2C1->DR = reg_addr;
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 7)) && timeout > 0) timeout--; // Wait for TxE
  if(timeout == 0) return;

  // Send data value
  I2C1->DR = value;
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 7)) && timeout > 0) timeout--; // Wait for TxE
  if(timeout == 0) return;

  // STOP condition
  I2C1->CR1 |= (1 << 9);
}

void i2c_init() {
  // Disable i2c peripheral first
  I2C1->CR1 &= ~(1 << 0);

  // Enable clocks before bus recovery
  i2c_en_clks();

  // I2C bus recovery: manually clock out any stuck transaction
  // Temporarily configure pins as GPIO to bit-bang SCL
  GPIOB->MODER &= ~((3 << 12) | (3 << 14));  // Clear mode bits
  GPIOB->MODER |= (1 << 12) | (1 << 14);      // PB6/PB7 as GPIO output

  // Generate 9 clock pulses on SCL to release SDA
  for(int i = 0; i < 9; i++) {
    GPIOB->ODR &= ~(1 << 6);  // SCL low
    for(volatile int j = 0; j < 100; j++);
    GPIOB->ODR |= (1 << 6);   // SCL high
    for(volatile int j = 0; j < 100; j++);
  }

  // Software reset I2C peripheral
  I2C1->CR1 |= (1 << 15);  // SWRST
  for(volatile uint32_t i = 0; i < 1000; i++);
  I2C1->CR1 &= ~(1 << 15); // Clear SWRST

  // Reconfigure pins as I2C alternate function
  i2c_setup_pins();
  i2c_set_fast_mode();
  i2c_init_dma();

  // Enable I2C1
  I2C1->CR1 |= (1 << 0);

  // Give I2C time to stabilize
  for(volatile uint32_t i = 0; i < 10000; i++);
}

void i2c_init_dma() {
  RCC->AHB1ENR |= (1 << 21);                      // Enable DMA1 clock

  // Disable stream first
  DMA1_Stream0->CR &= ~(1 << 0);
  while(DMA1_Stream0->CR & (1 << 0));              // Wait until disabled

  // Clear all interrupt flags
  DMA1->LIFCR |= (0x3F << 0);                     // Clear all Stream0 flags

  // Configure DMA1 Stream0 Channel 1 for I2C1_RX
  DMA1_Stream0->CR = 0;                           // Reset control register
  DMA1_Stream0->CR |= (1 << 25);                  // Channel 1 (I2C1_RX)
  DMA1_Stream0->CR |= (0 << 6);                   // Peripheral-to-memory
  DMA1_Stream0->CR |= (1 << 10);                  // Memory increment
  DMA1_Stream0->CR |= (0 << 11);                  // Peripheral no increment
  DMA1_Stream0->CR |= (0 << 13);                  // Memory data size: byte
  DMA1_Stream0->CR |= (0 << 11);                  // Peripheral data size: byte
  DMA1_Stream0->CR |= (1 << 4);                   // Transfer complete interrupt
  DMA1_Stream0->CR |= (2 << 16);                  // High priority

  // Set peripheral address (I2C1 data register)
  DMA1_Stream0->PAR = (uint32_t)&I2C1->DR;

  // Use direct mode for I2C (FIFO disabled)
  DMA1_Stream0->FCR = 0;  // Direct mode enabled (default)

  // Enable DMA interrupt in NVIC
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  NVIC_SetPriority(DMA1_Stream0_IRQn, 1);
}


void i2c_read_burst(uint8_t dev_bus_addr, uint8_t dev_reg_addr, uint8_t* buffer, uint16_t num_bytes) {
  uint32_t timeout;

  // Disable and reconfigure DMA for this transfer
  DMA1_Stream0->CR &= ~(1 << 0);                          // Disable DMA
  timeout = 10000;
  while((DMA1_Stream0->CR & (1 << 0)) && timeout > 0) timeout--;
  if(timeout == 0) return; // DMA disable failed

  // Clear all DMA flags
  DMA1->LIFCR |= (0x3F << 0);                             // Clear all Stream0 flags

  // Reconfigure DMA addresses and count
  DMA1_Stream0->M0AR = (uint32_t)buffer;
  DMA1_Stream0->NDTR = num_bytes;

  // Reset and configure I2C for DMA reception
  I2C1->CR2 &= ~(1 << 11);                               // Disable DMA first
  I2C1->CR2 &= ~(1 << 12);                               // Clear LAST bit
  I2C1->CR1 |= (1 << 10);                                // Enable ACK

  if(num_bytes == 1) {
    I2C1->CR1 &= ~(1 << 10);                             // Disable ACK for single byte
  } else {
    I2C1->CR2 |= (1 << 12);                              // Set LAST bit for multi-byte
  }

  I2C1->CR2 |= (1 << 11);                                // Enable DMA requests

  // Start I2C communication - write phase
  I2C1->CR1 |= (1 << 8);                                 // Generate START
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 0)) && timeout > 0) timeout--;
  if(timeout == 0) return; // START failed

  // Send device address in write mode
  I2C1->DR = dev_bus_addr << 1;
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 1)) && timeout > 0) timeout--;
  if(timeout == 0) return; // ADDR failed
  (void)I2C1->SR2;                                        // Clear ADDR flag

  // Send register address
  I2C1->DR = dev_reg_addr;
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 7)) && timeout > 0) timeout--;
  if(timeout == 0) return; // TxE failed

  // Generate repeated START for read
  I2C1->CR1 |= (1 << 8);
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 0)) && timeout > 0) timeout--;
  if(timeout == 0) return; // Repeated START failed

  // Send device address in read mode
  I2C1->DR = (dev_bus_addr << 1) | 0x01;
  timeout = 10000;
  while(!(I2C1->SR1 & (1 << 1)) && timeout > 0) timeout--;
  if(timeout == 0) return; // ADDR failed

  // Enable DMA and clear ADDR to start reception
  DMA1_Stream0->CR |= (1 << 0);                           // Enable DMA
  (void)I2C1->SR2;                                        // Clear ADDR - starts DMA transfer
}

void i2c_en_clks() {
  // enable gpiob and i2c1 peripheral clocks (resp)
    // gpiob clk: powers GPIOA port peripheral (pin control, AF routing)
    // i2c1 clk : powers I2C1 peripheral logic
  RCC->AHB1ENR |= (1 << 1);  
  RCC->APB1ENR |= (1 << 21);
}

void i2c_setup_pins() {
  // set pins PB6/7 to AF mode
  GPIOB->MODER &= ~((3 << 12) | (3 << 14));
  GPIOB->MODER |= (2 << 12) | (2 << 14);
   
  // specify pin AFs to be AF4 (i2c1)
  GPIOB->AFR[0] &= ~((0xF << 24) | (0xF << 28));
  GPIOB->AFR[0] |= (0x4 << 24) | (0x4 << 28);

  // configure pins to be open-drain
  GPIOB->OTYPER |= (1 << 6) | (1 << 7);

  // enable pullups
  GPIOB->PUPDR &= ~((3 << 12) | (3 << 14));
  GPIOB->PUPDR |= (1 << 12) | (1 << 14);
}

void i2c_set_fast_mode() {
  /*
   TIMINGR:
    controls the I2C timing characteristics
    ex: how long clock high/low periods last, setup/hold times, etc.

    TIMINGR bit fields (32-bit register):
      PRESC  [31:28] : prescaler (divides APB1 clock)
      SCLDEL [23:20] : data setup time
      SDADEL [19:16] : data hold time
      SCLH    [15:8] : SCL high period
      SCLL     [7:0] : SCL low period

    breaking down 0x00503D5A:
      PRESC  = 0x0 (bits 31-28): No prescaling
      SCLDEL = 0x5 (bits 23-20): Setup time
      SDADEL = 0x0 (bits 19-16): Hold time
      SCLH   = 0x3D (bits 15-8): SCL high = 61 cycles
      SCLL   = 0x5A (bits 7-0): SCL low = 90 cycles

    example:
      target: 400kHz I2C from 42MHz APB1
      period: 42MHz / 400kHz = 105 APB1 cycles per I2C cycle
      split: ~61 cycles high + 90 cycles low = 151 total (includes overhead)
      setup/hold times: Meet I2C fast-mode specs 
  */
  I2C1->CCR = (1 << 15) | 35;   // Fast mode (bit 15) + 400kHz (42MHz / (3 × 35) ≈ 400kHz)
  I2C1->TRISE = 14;
  
  // I2C1->TIMINGR = 0x00503D5A;
}

/*
 * circular buffer approach:
 *    - timer based isr (400hz) causes cpu to trigger start + burst read from imu. 
 *    - imu pushes 12 bytes -> 12 rxnes -> 12 dma transfers, consisting of a reading to a circular buffer. 
 *    - at the end of each 12 byte transfer, the dma raises an interrupt that causes the cpu to send a NACK+STOP to the imu 
 *    - also, because this is timer based, the buffer is populated at exactly 400hz. 
 *        - thus, 4 readings are populated every 10ms. 
 *        - every 4th reading triggers a secondary interrupt (and ends the dma isr). 
 *    - the cpu then addresses this secondary interrupt, retreiving the 4th reading and running the EKF with it. 
 *    - assuming that the cpu can run this within 2.5ms, the cpu is then performing 1 estimate every 10ms, or running at100hz.

 * */


