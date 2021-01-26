#include "bno055.h"

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

//Create a structure to hold device info
struct bno055_t myBNO;

//Link the I2C driver functions to the API communication function pointer
myBNO.bus_read = BNO055_I2C_bus_read;
myBNO.bus_write = BNO055_I2C_bus_write;
myBNO.delay_msec = delay;

//Set the correct I2C address in the BNO055 API
myBNO.dev_addr = BNO055_I2C_ADDR1;
//myBNO.dev_addr = BNO055_I2C_ADDR2;

//API initialization
bno055_init(&myBNO)

//Change operation mode to IMU
bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS)
