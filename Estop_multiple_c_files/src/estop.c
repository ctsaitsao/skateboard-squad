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
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/estop.h"

volatile bool stop=false;

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
void init_button_int(){
    GPIOPinTypeGPIOInput(BUTTON_BASE, BUTTON);
    GPIOPinTypeGPIOOutput(LED_BASE, RED_LED | BLUE_LED);
    GPIOPadConfigSet(BUTTON_BASE, BUTTON, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}
void enable_button_int(){
    GPIOIntDisable(BUTTON_BASE, BUTTON);
    GPIOIntClear(BUTTON_BASE, BUTTON);
    GPIOIntTypeSet(BUTTON_BASE, BUTTON, GPIO_FALLING_EDGE);
    GPIOIntRegister(BUTTON_BASE, button_ISR);
    IntPrioritySet(BUTTON_INT, 0);
    GPIOIntEnable(BUTTON_BASE, BUTTON_INT_PIN);
}
void init_timer_int(){
    TimerConfigure(TIMER_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER_BASE, TIMER, Hz_to_counts(100));
    TimerEnable(TIMER_BASE, TIMER);
}


void enable_timer_int(){
    TimerIntDisable(TIMER_BASE, TIMER_TIMEOUT_INT);
    TimerIntClear(TIMER_BASE, TIMER_TIMEOUT_INT);
    TimerIntRegister(TIMER_BASE, TIMER, timer_ISR);
    IntPrioritySet(TIMER_INT, 64);
    TimerIntEnable(TIMER_BASE, TIMER_TIMEOUT_INT);
}



