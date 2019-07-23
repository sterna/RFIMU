/*
 * APA102Conf.h
 *
 *  Created on: 11 May 2019
 *      Author: Sterna
 */

/*
 * This file defines the hardware conf to use in the apa102 library
 */

#ifndef APA102CONF_H_
#define APA102CONF_H_

#include "utils.h"

//The maximum numbers of LEDs per strip
#define APA_MAX_NOF_LEDS 500
//The number of strips (all these are handled in a hardcoded way)
#define APA_NOF_STRIPS 3
//Keyword to use when trying to adress all strips (might not work for all functions)
#define APA_ALL_STRIPS 255

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

//Hardware settings for strip 2
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

//Hardware settings for strip 3
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


#endif /* APA102CONF_H_ */
