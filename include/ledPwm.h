/*
 *	ledPwm.h
 *
 *	Created on: 6 feb. 2016
 *		Author: root
 */

#ifndef LEDPWM_H_
#define LEDPWM_H_

#include "stm32f10x.h"
#include <stdbool.h>
#include "time.h"
#include "utils.h"


//Only one channel, alla are the same)
/*
 * Using the led driver shield:
 * CH1_R, G, B = A7, B0, B1 -> TIM3_CH2, CH3, CH4
 * CH2_R, G, B = A1, A2, A3 -> TIM2_CH2, CH3, CH4
 * CH3_R, G, B = B6, B7, B8 -> TIM4_CH1, CH2, CH3
 * CH4_R, G, B = B9, A0, A6 -> TIM4_CH4, TIM2_CH1, TIM3_CH1
 * CH5_R, G, B, W = A10, A8, A11, A9 -> TIM1_CH3, CH1, CH4, CH2 (This should maybe be changed, so that W->R, R->G, G->B, W->B)
 */

#define B1_PWM	TIM3->CCR2
#define G1_PWM	TIM3->CCR3
#define R1_PWM	TIM3->CCR4

#define B2_PWM	TIM2->CCR2
#define G2_PWM	TIM2->CCR3
#define R2_PWM	TIM2->CCR4

#define B3_PWM	TIM4->CCR1
#define G3_PWM	TIM4->CCR2
#define R3_PWM	TIM4->CCR3

#define B4_PWM	TIM4->CCR4
#define G4_PWM	TIM2->CCR1
#define R4_PWM	TIM3->CCR1

//These ones were changed, the legend in the schematics is commented below
#define G5_PWM	TIM1->CCR4	//org R
#define B5_PWM	TIM1->CCR1	//org G
#define W5_PWM	TIM1->CCR3	//org B
#define R5_PWM	TIM1->CCR2	//org W

//The time interval that the led fade function should be called at.
#define FADE_DELAY_TIME_MS	50

//The number of LED channels
#define LED_PWM_NOF_CHANNELS 5

typedef struct
{
	uint32_t r_min;
	uint32_t r_max;
	uint32_t g_min;
	uint32_t g_max;
	uint32_t b_min;
	uint32_t b_max;
}led_fade_setting_t;

typedef struct
{
	uint16_t r;
	uint16_t g;
	uint16_t b;
	uint16_t r_rate;
	uint16_t g_rate;
	uint16_t b_rate;
	int8_t dir;
	uint32_t cycles;	//
	led_fade_setting_t conf;
	bool active;
}led_fade_state_t;

void ledPwmInit();
void ledPwmRunFade(led_fade_setting_t* s);
void ledPwmUpdateColours(uint16_t r,uint16_t g,uint16_t b, uint8_t chan);
void ledPwmClear(uint8_t chan);

void ledFadeSetup(led_fade_setting_t* s, uint8_t ch);
void ledFadeRunIteration(uint8_t ch);
void ledFadeSetActive(uint8_t ch, bool setting);
void ledFadeRestart();
bool ledFadeGetState(uint8_t ch);



#endif /* LEDPWM_H_ */
