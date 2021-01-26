#ifndef __LSM303D__H__
#define __LSM303D__H__
// Header file for LSM303D.c
// implements high-level accelerometer functions using I2C

// Important registers:
// (register map available at https://www.pololu.com/file/0J703/LSM303D.pdf)
#define IMU_ADDR    0x1D // 0x1D // I2C hardware address of LSM303D
#define WHOAMI      0x0F // useful register to verify communication with LSM303D
#define CTRL_1      0x20 // bit field for configuring accelerometer
#define CTRL_2      0x21 // accelerometer anti-alias filter and full-scale selection
#define X_XL_OUT_L  0x28 // x-axis accelerometer output, least significant byte
#define X_XL_OUT_M  0X29 // x-axis accelerometer output, most significant byte
#define Y_XL_OUT_L  0x2A // y-axis accelerometer output, least significant byte
#define Y_XL_OUT_M  0X2B // y-axis accelerometer output, most significant byte
#define Z_XL_OUT_L  0x2C // z-axis accelerometer output, least significant byte
#define Z_XL_OUT_M  0x2D // z-axis accelerometer output, most significant byte

#define LSM_ARRAY_LEN 14 // data array has 14 values

void LSM303D_init(void); // initialize LSM303D accelerometer
unsigned char WhoAmI(void); // use this to confirm communication with LSM6DS33 IMU
unsigned char WhoAmI2(uint8_t); // testing i2c adresses
uint16_t ReadIMU(uint8_t reg);
signed short getxXL(void); // convert x-acceleration LSB and MSB to signed short
float convxXL(void); // convert x-acceleration signed short to float (g's)
signed short getyXL(void); // convert y-acceleration LSB and MSB to signed short
float convyXL(void); // convert y-acceleration signed short to float (g's)
signed short getzXL(void); // convert z-acceleration LSB and MSB to signed short
float convzXL(void); // convert z-acceleration signed short to float (g's)

#endif
