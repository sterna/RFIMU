/*
 *	adc.h
 *
 *	Created on: Jan 2, 2018
 *		Author: Sterna
 */
#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "utils.h"


#define BAT1_ADC_CHAN		9
#define BAT1_ADC_PIN		1
#define BAT1_ADC_PORT		GPIOB
#define BAT1_INDEX_NUM		0

#define IR1_ADC_CHAN		6
#define IR1_ADC_PIN			6
#define IR1_ADC_PORT		GPIOA
#define IR1_INDEX_NUM		1

#define IR2_ADC_CHAN		8
#define IR2_ADC_PIN			0
#define IR2_ADC_PORT		GPIOB
#define IR2_INDEX_NUM		2

#define ADC1_DR_Address    	((uint32_t)ADC1_BASE+0x4C)
#define MILLIVOLT_PER_BIT	3300/4096
//The resistors for the voltage dividers for the batteries, in ohms
#define BAT_VOLT_DIV_R1		10000
#define BAT_VOLT_DIV_R2		10000
//The ratio for batteri voltage calculator
//Note that the ( in the beginning is omitted. This is to be able to use integer math. To get the voltage use: Vbat=(VbatMeas*BAT_VOLT_DIV_RATIO_INV
#define BAT_VOLT_DIV_RATIO_INV	(BAT_VOLT_DIV_R1+BAT_VOLT_DIV_R2))/BAT_VOLT_DIV_R2

//The number of channels used for the ADC
#define ADC_NOF_CHANNELS 3
//The number of samples for the filtered ADC value
#define ADC_FILTER_SIZE 8

uint16_t adcGetBatVolt();
uint16_t adcGetIRSens(uint8_t ch);
void adcInit();


#endif /* ADC_H_ */
