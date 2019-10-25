/*
 * onboardLedCtrl.c
 *
 * 	Controls the onboard LED
 */

/*
*********************************************************************************
*	Include files
*********************************************************************************
*/
#include "onboardLedCtrl.h"

#include "stm32f10x.h"
#include "string.h"
#include "utils.h"

/*
*********************************************************************************
*	Defined constants
*********************************************************************************
*/

#define RED_LOW_BYTE 		0
#define RED_HIGH_BYTE 		1
#define GREEN_LOW_BYTE 		2
#define GREEN_HIGH_BYTE 	3
#define BLUE_LOW_BYTE 		4
#define BLUE_HIGH_BYTE 		5

#define PAYLOAD_SIZE		6

#define R_PIN_NUM	3
#define R_PIN		utilGetGPIOPin(R_PIN_NUM)
#define G_PIN_NUM	2
#define G_PIN		utilGetGPIOPin(G_PIN_NUM)
#define B_PIN_NUM	1
#define B_PIN		utilGetGPIOPin(B_PIN_NUM)
#define RGB_PORT	GPIOA	//This only works if all are on the same port, which they most likely are

#define RGB_TIMER	TIM2
#define R_CHAN	4
#define G_CHAN	3
#define B_CHAN	2

/*
*********************************************************************************
*	Type definitions & enums
*********************************************************************************
*/

/*
*********************************************************************************
*	Variables
*********************************************************************************
*/
static S_RGB_LED currentLed;

/*
*********************************************************************************
*	Functions
*********************************************************************************
*/

/*
 *********************************************************************************
 *  External Functions
 *********************************************************************************
 */

/*
 * Init the control of the onboard LED
 */
void onboardLedCtrlInit(void)
{
	//Init LED PWM
	GPIO_InitTypeDef 		GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIMBaseInitStruct;
	TIM_OCInitTypeDef 		TIMOCInitStruct;
	uint16_t PrescalerValue = 0;

	SystemCoreClockUpdate();
	PrescalerValue = (uint16_t) (SystemCoreClock / 1000000) - 1; // Prescaler of Timer 2 will run on 1Mhz

	// Setup GPIO used for RGB led
	utilSetClockGPIO(RGB_PORT,ENABLE);
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin 	= R_PIN | G_PIN | B_PIN;
	GPIO_Init(RGB_PORT,&GPIO_InitStruct);

	// Enable RGB_TIMER clock
	// CPU clock runs on 72 MHz
	utilSetClockTIM(RGB_TIMER,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	// Setup timer to 1Khz Prescaler/period (1.000.000 / 1000)
	TIMBaseInitStruct.TIM_Period 		= 1000;
	TIMBaseInitStruct.TIM_Prescaler 	= PrescalerValue;
	TIMBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIMBaseInitStruct.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIMBaseInitStruct);

	// Same setting for all timer channels
	TIM_OCStructInit(&TIMOCInitStruct);
	TIMOCInitStruct.TIM_OCMode 		= TIM_OCMode_PWM1;
	TIMOCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIMOCInitStruct.TIM_OCPolarity 	= TIM_OCPolarity_High;
	TIMOCInitStruct.TIM_Pulse 		= 0;

	//Inits the timer PWMs (all channels have the same setting)
	utilTIMOCInit(RGB_TIMER,R_CHAN,&TIMOCInitStruct);
	utilTIMOCInit(RGB_TIMER,G_CHAN,&TIMOCInitStruct);
	utilTIMOCInit(RGB_TIMER,B_CHAN,&TIMOCInitStruct);

	//Init LEDs as white, awaiting commands
	S_RGB_LED led;
	led.red = 0;
	led.green =0;
	led.blue = 0;
	onboardLedCtrlWriteColours(led);

	TIM_Cmd(RGB_TIMER,ENABLE);
}


/*
 * Write colours to PWM
 */
void onboardLedCtrlWriteColours(S_RGB_LED led)
{
	utilTIMOCWrite(RGB_TIMER,R_CHAN,led.red);
	utilTIMOCWrite(RGB_TIMER,G_CHAN,led.green);
	utilTIMOCWrite(RGB_TIMER,B_CHAN,led.blue);
}

/*
 *
 */
void onboardLedCtrlUpdateColors(uint8_t colors[])
{
	S_RGB_LED led;

	led.red    	=  colors[RED_LOW_BYTE];
	led.red   	|=(colors[RED_HIGH_BYTE] << 8);
	led.green  	=  colors[GREEN_LOW_BYTE];
	led.green 	|=(colors[GREEN_HIGH_BYTE] << 8);
	led.blue    =  colors[BLUE_LOW_BYTE];
	led.blue	|=(colors[BLUE_HIGH_BYTE] << 8);
	currentLed = led;
	onboardLedCtrlWriteColours(led);
}

/*
 *********************************************************************************
 *  Internal Functions
 *********************************************************************************
 */



