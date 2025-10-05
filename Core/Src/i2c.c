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



void i2c_init() {
  // disable i2c peripheral
  I2C1->CR1 &= ~(1 << 0);
   
  i2c_en_clks();
  i2c_setup_pins();
  i2c_set_fast_mode();
  i2c_init_dma(); 
// enable I2C1 
  I2C1->CR1 |= (1 << 0);
}

void i2c_init_dma() {
  RCC->AHB1ENR |= (1 << 21);                      // power up DMA1 peripheral through DMA1EN bit
 
 // RM0390: 9.5.5 (pg 226)
  DMA1_Stream0->CR &= ~(1 << 0);                  // disable DMA stream first to allow config changes
  DMA1_Stream0->CR |= (1 << 25);                  // CHSEL = 001 (chan 1, I2C1_RX) -> RM0390: 9.3.4, table 28 
   
  DMA1_Stream0->PAR  = (uint32_t)&(I2C1->DR);     // TODO: stream0 source: peripheral addr (RM0390, 9.5.7)
  DMA1_Stream0->M0AR = (uint32_t)imu_get_raw_buffer(); // buffer start position
  DMA1_Stream0->NDTR = 12;                        // 8 packets x 12 bytes/packet: 6 each for accel, gyro
    
  DMA1_Stream0->CR  &= ~(3 <<  6);                // DIR  = 00: peripheral-to-memory
  DMA1_Stream0->CR  &= ~(1 <<  8);                // CIRC =  0: single transfer mode
  DMA1_Stream0->CR  |=  (1 << 10);                // MINC =  1: memory increment

  DMA1_Stream0->FCR |=  (1 <<  2);                // DMDIS = 1; (disable direct mode)
  DMA1_Stream0->FCR |=   1;                       // DMDIS = 1; (disable direct mode)
  DMA1_Stream0->CR  |=  (1 <<  4);                // TCIE =  1: transfer complete INT 
  DMA1_Stream0->CR  |=  (1 <<  0);                // enable DMA stream 
}


void i2c_read_burst(uint8_t dev_bus_addr, uint8_t dev_reg_addr, uint16_t num_bytes) {
  DMA1_Stream0->CR &= ~0x01;                      // disable DMA
  while(DMA1_Stream0->CR & (0x01));               // wait until disabled

  DMA1_Stream0->M0AR = (uint32_t)imu_get_raw_write_position();

  DMA1_Stream0->NDTR = 12;
  DMA1_Stream0->CR |= 0x01;                       // enable DMA
  I2C1->CR2 |= (1 << 11);                         // enable DMA for I2C1 (set DMAEN)

  while(I2C1->SR2 & (1 << 1));                   // wait until bus is idle
  
  // START
  I2C1->CR1 |= (1 << 8);                          // SW trigger to set HW START bit 
  while(!(I2C1->SR1 & 0x01));                     // wait until the start condition propagates

  // send dev and reg addrs
  I2C1->DR = dev_bus_addr;                        // ping the device on the bus  
  while(!(I2C1->SR1 & (1 << 1)));                 // wait until HW sets "dev found" ADDR flag upon matching target addr (RM0390 24.6.7)

  (void)I2C1->SR2;                                // dummy read from SR2 immediately after SR1 clears ADDR bit (RM0390 24.6.7)
                                                  //   - the ADDR bit confirms that the target device is found
                                                  //   - clearing the bit allows the protocol to continue to the "data phase"

  I2C1->DR = dev_reg_addr;
  while(I2C1->SR1 & (1 << 7));                    // wait for the TxE (transmitter register empty) flag
                                                  //   - byte written to I2C1->DR has been moved to shift register

  // REPEATED START for burst read  
  I2C1->CR1 |= (1 << 8);                          // repeated start to read full payload (num_bytes)
  while(!(I2C1->SR1 & 0x01));                     // wait until the start condition propagates
  
  I2C1->DR = (dev_bus_addr << 1) | 0x01;          // the LSB of the device addr specifies R/W (set for READ mode)
  while(!(I2C1->SR1 & (1 << 1)));                 // wait until HW sets "dev found" ADDR flag upon matching target addr (RM0390 24.6.7) 

  (void)I2C1->SR2;                                // clearing the ADDR bit triggers IMU to begin transmitting reading
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
  I2C1->CCR = 210;
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


