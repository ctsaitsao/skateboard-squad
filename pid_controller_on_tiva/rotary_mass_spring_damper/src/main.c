#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif

void ConfigureUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(0, 115200, 16000000);
}

uint16_t getSensorState(void) {
    uint8_t terminator = 0xFF;

    while(true) {
        uint8_t encoderPositionLSB = UARTgetc();
        uint8_t encoderPositionMSB = UARTgetc();
        terminator = UARTgetc();

        if(terminator == 0) {
            return encoderPositionLSB + (encoderPositionMSB << 8);
        }
    }
}

void sendTorqueCommand(uint16_t torque) {
    char commandArray[] = {torque & 0xFF, torque >> 8, 0};
    UARTwrite(commandArray, 3);
}

int main(void) {
    FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    ConfigureUART();

    // Send a single zero-torque command to prevent "hang" condition in Simulink:
    sendTorqueCommand(0);

    const float Kp = 22;
    const float Kd = 28;
    const float Ki = 4;

    int16_t error = 0;
    float errorSum = 0;
    uint16_t prevError = 0;

    int16_t encoderPosition = 0;
    int16_t setpoint = 1023;
    int16_t controlEffort = 0;
    int16_t errorCeiling = 5000;

    float dt = 0.01;

    while(1)
    {
        encoderPosition = getSensorState();    // getSensorState() needs data but Simulink settings are set to blocking (it needs data first), so this is why we need to send zero-torque command before while loop
        error = setpoint - encoderPosition;
        controlEffort = error*Kp + (error - prevError)*Kd + errorSum*Ki;    // units of mNm
        prevError = error;
        if (errorSum <= errorCeiling) {
            errorSum += error*dt;
        }
        sendTorqueCommand(controlEffort);
    }
}
