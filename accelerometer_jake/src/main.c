// code adapted from
// https://eewiki.net/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL#I2CCommunicationwiththeTITivaTM4C123GXL-ExampleofReadingdatafromaFreescaleMMA7455L3-axisAccelerometer

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
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

// custom includes:
#include "i2c_master_no_int.h"
#include "LSM303D.h"

void InitI2C0(void);
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
void I2CSendString(uint32_t slave_addr, char array[]);
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);
void LSM6DS33_init(void); // initialize LSM6DS33 IMU

void InitConsole(void) // set up UART0 to use as a console
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);


    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void SimpleDelay(void)
{
    //
    // Delay cycles for 1 second
    //
    SysCtlDelay(16000000 / 3);
}

int main(void)
{
    //volatile uint32_t ui32Loop;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);


    // initialize the console on UART0:
    InitConsole();

    UARTprintf("Hello!\n");

    //initialize I2C module 0
    InitI2C0();
    UARTprintf("I2C0 initialized!\n");

    int16_t Ax = 0;
    int16_t Ay = 0;
    int16_t Az = 0;
    uint8_t whoiam = 0;
    uint8_t counter = 0;
    uint8_t address = 0;

    LSM303D_init();
    UARTprintf("LSM303D accelerometer initialized!\n");

    while(1)
    {
    UARTprintf("Send Command:");

    switch(UARTscanf("%d")) {

        case 1:
        UARTprintf("10\n");
        break;

        case 2:
        UARTprintf("20\n");
        break;

    }
      //
      // Turn on the LED.
      //
     // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

      //Ax = getxXL();
     // Ay = getyXL();
     // Az = getzXL();
     // whoiam = WhoAmI();

     // UARTprintf("WhoAmI: %d, X: %d, Y: %d, Z: %d\n",whoiam,Ax,Ay,Az); //X: %d\tY: %d\tZ: %d\n",whoiam,Ax,Ay,Az);

      //address++;
     // counter++;

     SimpleDelay();

      //
      // Turn off the LED.
      //
     // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

      //UARTprintf("Hello world!\n");

      SimpleDelay();

    }

    return 0;
}
