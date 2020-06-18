/*
 * APA102Conf.h
 *
 *  Created on: 11 May 2019
 *      Author: Sterna
 */

/*
 * This file defines the hardware conf to use in the apa102 library
 */

/*
 * This conf is for the RFIMU board
 */

#ifndef APA102CONF_H_
#define APA102CONF_H_

#include "utils.h"

//The maximum numbers of LEDs per strip
#define APA_MAX_NOF_LEDS 	300
//The number of strips (all these are handled in a hardcoded way)
#define APA_NOF_STRIPS		 1
//Keyword to use when trying to adress all strips (might not work for all functions)
#define APA_ALL_STRIPS 		255
//The max value for the colour scaling settings
#define APA_SCALE_MAX			128
//THe maximum number of strip scaling used
#define APA_SCALE_MAX_SEGMENTS	1
//Enables/desiables sacling
#define APA_ENABLE_SCALING

//Hardware conf
//Hardware settings for strip 1
#define APA_SPI				SPI1
#define APA_SPI_DR			(SPI1_BASE+0x0C)
#define APA_DMA_CH 			DMA1_Channel3
#define APA_DMA_TC_FLAG		DMA1_FLAG_TC3
#define APA_DMA_TE_FLAG		DMA1_FLAG_TE3
#define APA_DMA_IRQ			DMA1_Channel3_IRQHandler
#define APA_DMA_IRQn		DMA1_Channel3_IRQn
//Speed setting for the peripheral. What to write depends on the type of peripheral
#define APA_SPEED_SETTING	SPI_BaudRatePrescaler_16
//Ports, pins and remap for strip 1
#define APA_MOSI_PIN 		7
#define APA_MOSI_PORT		GPIOA
#define APA_SCK_PIN 		5
#define APA_SCK_PORT		GPIOA
//Set REMAP_CONFIG to correct remap, if needed. Set to 0 if remap is not needed //GPIO_Remap_SPI1
#define APA_REMAP_CONFIG	0

//Start stop for the first scale in this strip
//Note: Any LEDs not within any scale range will not be scaled
//Settings for scaling strip1,segment1
#define APA_SCALE1_START	0
#define APA_SCALE1_STOP		140
//Scale factors for colours. Value is 0 to 128	(to optimize scaling performance)
#define APA_SCALE1_R		128
#define APA_SCALE1_G		90
#define APA_SCALE1_B		70
#define APA_SCALE1			{APA_SCALE1_START,APA_SCALE1_STOP,APA_SCALE1_R,APA_SCALE1_G,APA_SCALE1_B}

//Settings for scaling strip1,segment2
#define APA_SCALE2_START	40
#define APA_SCALE2_STOP		80
//Scale factors for colours. Value is 0 to 128	(to optimize scaling performance)
#define APA_SCALE2_R		30
#define APA_SCALE2_G		128
#define APA_SCALE2_B		128
#define APA_SCALE2			{APA_SCALE2_START,APA_SCALE2_STOP,APA_SCALE2_R,APA_SCALE2_G,APA_SCALE2_B}

//Settings for scaling strip1,segment2
#define APA_SCALE3_START	81
#define APA_SCALE3_STOP		100
//Scale factors for colours. Value is 0 to 128	(to optimize scaling performance)
#define APA_SCALE3_R		128
#define APA_SCALE3_G		128
#define APA_SCALE3_B		30
#define APA_SCALE3			{APA_SCALE3_START,APA_SCALE3_STOP,APA_SCALE3_R,APA_SCALE3_G,APA_SCALE3_B}

//#define APA_SCALE			{APA_SCALE1,APA_SCALE2,APA_SCALE3}
#define APA_SCALE			{APA_SCALE1}



//Hardware settings for strip 2 (not used for this board)
#define APA2_SPI			SPI2
#define APA2_SPI_DR			(SPI2_BASE+0x0C)
#define APA2_DMA_CH 		DMA1_Channel5
#define APA2_DMA_TC_FLAG	DMA1_FLAG_TC5
#define APA2_DMA_TE_FLAG	DMA1_FLAG_TE5
#define APA2_DMA_IRQ		DMA1_Channel5_IRQHandler
#define APA2_DMA_IRQn		DMA1_Channel5_IRQn
//Speed setting for the peripheral. What to write depends on the type of peripheral
#define APA2_SPEED_SETTING	SPI_BaudRatePrescaler_16
//Ports, pins and remap for strip 2
#define APA2_MOSI_PIN 		15
#define APA2_MOSI_PORT		GPIOB
#define APA2_SCK_PIN 		13
#define APA2_SCK_PORT		GPIOB
//Set REMAP_CONFIG to correct remap, if needed. Set to 0 if remap is not needed
#define APA2_REMAP_CONFIG	0

//Hardware settings for strip 3 (not used for this board)
#define APA3_SPI			USART2
#define APA3_SPI_DR			(USART2_BASE+0x04)
#define APA3_DMA_CH 		DMA1_Channel7
#define APA3_DMA_TC_FLAG	DMA1_FLAG_TC7
#define APA3_DMA_TE_FLAG	DMA1_FLAG_TE7
#define APA3_DMA_IRQ		DMA1_Channel7_IRQHandler
#define APA3_DMA_IRQn		DMA1_Channel7_IRQn
//Speed setting for the peripheral. What to write depends on the type of peripheral
#define APA3_SPEED_SETTING	1125000
//Ports, pins and remap for strip 3
#define APA3_MOSI_PIN 		2
#define APA3_MOSI_PORT		GPIOA
#define APA3_SCK_PIN 		4
#define APA3_SCK_PORT		GPIOA
//Set REMAP_CONFIG to correct remap, if needed. Set to 0 if remap is not needed
#define APA3_REMAP_CONFIG	0


//Here, the totality of the led correction is documented
#define APA_SCALE_ASSIGN {APA_SCALE}

#endif /* APA102CONF_H_ */
