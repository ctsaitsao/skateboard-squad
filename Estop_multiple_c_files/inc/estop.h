/*
 * estop.h
 *
 *  Created on: May 7, 2020
 *      Author: jbera
 */

#ifndef INC_ESTOP_H_
#define INC_ESTOP_H_

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

extern volatile bool stop;

void button_ISR();
void timer_ISR();

int Hz_to_counts(int Hz);


void init_button_int();
void enable_button_int();
void init_timer_int();
void enable_timer_int();


#endif /* INC_ESTOP_H_ */
