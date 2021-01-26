#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
//#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


void sendTorqueCommand(uint16_t torque)
{
    uint8_t LSB = torque && 0xFF;
    uint8_t MSB = torque >> 8;
    char buffer[3] = {LSB,MSB,0};
    UARTwrite(buffer,3);
}

uint16_t getSensorState()
{
    uint8_t terminator=0xFF;
    while(1)
    {
        uint8_t encoder_pos_LSB = UARTgetc();
        uint8_t encoder_pos_MSB = UARTgetc();
        terminator = UARTgetc();
        if (terminator == 0)
        {
            return encoder_pos_LSB + (encoder_pos_MSB << 8);
        }
        }
    }


