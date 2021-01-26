#ifndef I2C_MASTER_NO_INT__H__
#define I2C_MASTER_NO_INT__H__
// Header file for i2c_master_no_int.c
// helps implement I2C as a master w/o interrupts

void InitI2C0(void);
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
void I2CSendString(uint32_t slave_addr, char array[]);
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);

#endif
