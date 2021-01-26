// Standard C includes:
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

// Tivaware includes:
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

// Custom includes:
#include "i2c_master_no_int.h"
#include "LSM303D.h"

// Conversion factors for computing G's of accel, given some signed short:
// (available at https://www.pololu.com/file/0J703/LSM303D.pdf)
#define LSB_TO_G_PM2G 0.000061  // full-scale (FS) +/- 2G
#define LSB_TO_G_PM4G 0.000122  // FS +/- 4G
#define LSB_TO_G_PM6G 0.000183  // FS +/- 6G
#define LSB_TO_G_PM8G 0.000244  // FS +/- 8G
#define LSB_TO_G_PM16G 0.000732 // FS +/- 16G

void LSM303D_init(void) { // initialize LSM303D accelerometer
	InitI2C0(); // set up I2C0 at 100 kHz

  	// initialize XL (accelerometer):
  	// writing 0xA7 to CTRL_1 means:
    //  AODR[3:0]   = 0b1010    -> 1.6 kHz sample rate
    //  BDU         = 0b0       -> data is updated continuously
    //  AZEN        = 0b1       -> enable accelerometer z-axis
    //  AYEN        = 0b1       -> enable accelerometer y-axis
    //  AXEN        = 0b1       -> enable accelerometer x-axis
   	I2CSend(IMU_ADDR,2,CTRL_1,0xA7);

  	// Configure accelerometer anti-alias filter and full-scale range:
    // writing 0x20 to ctrl2 means:
    //  ABW[1:0     = 0b00      -> acceleration anti-alias filter BW = 773 Hz (default)
    //  AFS[2:0]    = 0b100     -> acceleration full-scale = +/- 16g (NOT default)
    //  AST         = 0b0       -> disable acceleration self-test (default)
    //  SIM         = 0b0       -> configure SPI to use 4-wire interface (default, irrelevant)
  	I2CSend(IMU_ADDR,2,CTRL_2,0x20);
}

unsigned char WhoAmI(void) {
    return I2CReceive(IMU_ADDR, WHOAMI);
}

unsigned char WhoAmI2(uint8_t address) {
    return I2CReceive(address, WHOAMI);
}

uint16_t ReadIMU(uint8_t reg)
{
    uint8_t accelDataL =  I2CReceive(IMU_ADDR, reg);
    uint8_t accelDataH =  I2CReceive(IMU_ADDR, reg+1);

    return ((accelDataH << 8) | accelDataL);
}

signed short getxXL(void) {
    return ReadIMU(X_XL_OUT_L);
}

float convxXL(void) {
    return getxXL()*LSB_TO_G_PM16G;
}

signed short getyXL(void) {
    return ReadIMU(Y_XL_OUT_L);
}

float convyXL(void) {
    return getyXL()*LSB_TO_G_PM16G;
}

signed short getzXL(void) {
    return ReadIMU(Z_XL_OUT_L);
}

float convzXL(void) {
    return getzXL()*LSB_TO_G_PM16G;
}
