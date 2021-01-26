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
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "inc/estop.h"





#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif



void configure_UART() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}



int main() {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    configure_UART();

    SysCtlPeripheralEnable(TIMER_PERIPH);
    SysCtlPeripheralEnable(LED_PERIPH);

    SysCtlDelay(3);

    init_button_int();
    enable_button_int();
    init_timer_int();
    enable_timer_int();

    while(1) {
        if (stop == true) {
                GPIOPinWrite(LED_BASE, RED_LED, RED_LED);
        }
    }
}
