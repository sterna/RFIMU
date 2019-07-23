/*
 *	sw.h
 *
 *	Created on: 5 okt 2014
 *		Author: Sterna
 */
#ifndef SW_H_
#define SW_H_

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

//Used for swithc on discovery board. Not used here
#define SW (GPIOA->IDR & (1<<0))

//Special case for RFIMU
#define SW1_PORT			GPIOA
#define SW1_PORT_REG 		SW1_PORT->IDR
#define SW1_PIN 6

#define SW2_PORT			GPIOB
#define SW2_PORT_REG 		SW2_PORT->IDR
#define SW2_PIN 0

#define SW1_4_PORT			GPIOB
#define SW1_4_PORT_REG 		SW1_4_PORT->IDR

#define SW3_PIN	8
#define SW4_PIN	9

#define SW5_7_PORT			GPIOC
#define SW5_7_PORT_REG 		SW5_7_PORT->IDR

#define SW5_PIN	13
#define SW6_PIN	14
#define SW7_PIN	15

#define SW8_PORT			GPIOB
#define SW8_PORT_REG 		SW8_PORT->IDR
#define SW8_PIN 10

#define SW1 (!(SW1_PORT_REG &(1<<SW1_PIN)))
#define SW2 (!(SW2_PORT_REG &(1<<SW2_PIN)))
#define SW3 (!(SW1_4_PORT_REG &(1<<SW3_PIN)))
#define SW4 (!(SW1_4_PORT_REG &(1<<SW4_PIN)))
#define SW5 (!(SW5_7_PORT_REG &(1<<SW5_PIN)))
#define SW6 (!(SW5_7_PORT_REG &(1<<SW6_PIN)))
#define SW7 (!(SW5_7_PORT_REG &(1<<SW7_PIN)))
#define SW8 (!(SW8_PORT_REG &(1<<SW8_PIN)))

#define SW_NOF_SWITCHES	2

//The interval at which the debounce function is run (in ms)
#define SW_DEBOUNCE_PERIOD 33
#define SW_DEBOUNCE_COUNTER_TOP 3

void swInit();
void swDebounceTask();

bool swGetState(uint8_t sw);
bool swGetRisingEdge(uint8_t sw);
bool swGetFallingEdge(uint8_t sw);

#endif /* SW_H_ */
