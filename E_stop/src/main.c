#include <stdint.h>
#include <stdbool.h>

#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.c"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"

#define LED_PERIPH SYSCTL_PERIPH_GPIOF
#define LED_BASE GPIO_PORTF_BASE
#define RED_LED GPIO_PIN_1
#define BLUE_LED GPIO_PIN_2

#define BUTTON_BASE GPIO_PORTF_BASE
#define BUTTON GPIO_PIN_4
#define BUTTON_INT INT_GPIOF
#define BUTTON_INT_PIN GPIO_INT_PIN_4

#define TIMER_PERIPH SYSCTL_PERIPH_TIMER0
#define TIMER_BASE TIMER0_BASE
#define TIMER TIMER_A
#define TIMER_INT INT_TIMER0A
#define TIMER_TIMEOUT_INT TIMER_TIMA_TIMEOUT

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif

static volatile bool stop = false;

void configure_UART() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

void button_ISR() {
    uint32_t status = GPIOIntStatus(BUTTON_BASE, true);

    if(status & BUTTON_INT_PIN) {
        stop = true;
    }

    GPIOIntClear(BUTTON_BASE, BUTTON);
}

void timer_ISR() {
    if (stop == true) {
        GPIOPinWrite(LED_BASE, BLUE_LED, BLUE_LED);
    }

    TimerIntClear(TIMER_BASE, TIMER_TIMEOUT_INT);
}

int Hz_to_counts(int Hz) {
    return (int) 120000000/Hz;
}

int main() {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    configure_UART();

    SysCtlPeripheralEnable(LED_PERIPH);
    SysCtlPeripheralEnable(TIMER_PERIPH);

    SysCtlDelay(3);

    GPIOPinTypeGPIOInput(BUTTON_BASE, BUTTON);
    GPIOPinTypeGPIOOutput(LED_BASE, RED_LED | BLUE_LED);
    GPIOPadConfigSet(BUTTON_BASE, BUTTON, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntDisable(BUTTON_BASE, BUTTON);
    GPIOIntClear(BUTTON_BASE, BUTTON);
    GPIOIntTypeSet(BUTTON_BASE, BUTTON, GPIO_FALLING_EDGE);
    GPIOIntRegister(BUTTON_BASE, button_ISR);
    IntPrioritySet(BUTTON_INT, 0);
    GPIOIntEnable(BUTTON_BASE, BUTTON_INT_PIN);

    TimerConfigure(TIMER_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER_BASE, TIMER, Hz_to_counts(100));
    TimerEnable(TIMER_BASE, TIMER);

    TimerIntDisable(TIMER_BASE, TIMER_TIMEOUT_INT);
    TimerIntClear(TIMER_BASE, TIMER_TIMEOUT_INT);
    TimerIntRegister(TIMER_BASE, TIMER, timer_ISR);
    IntPrioritySet(TIMER_INT, 64);
    TimerIntEnable(TIMER_BASE, TIMER_TIMEOUT_INT);

    while(1) {
        if (stop == true) {
            GPIOPinWrite(LED_BASE, RED_LED, RED_LED);
        }
    }
}
