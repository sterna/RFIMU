/*
 *	adc.c
 *
 *	Created on: Jan 2, 2018
 *		Author: Sterna
 */

#include "adc.h"

//Various arrays for ADC values
static volatile uint16_t adcFilteredVals[ADC_NOF_CHANNELS];	//Here you can read the filtered ADC values
static volatile uint32_t adcRawTotal[ADC_NOF_CHANNELS];		//Internal buffer used for the filter
static volatile uint16_t adcRawBuffer[ADC_NOF_CHANNELS];	//The DMA places samples here

/*
 * Inits all things for the ADC
 * The ADC measures the following things:
 * 	- Battery voltage on three channels
 */
void adcInit()
{

	GPIO_InitTypeDef 		GPIO_InitStruct;
	NVIC_InitTypeDef 		NVIC_InitStructure;
	ADC_InitTypeDef 		ADC_InitStructure;
	DMA_InitTypeDef 		DMA_InitStructure;

	// ADCCLK = PCLK2/4
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);


	// Enable peripheral clocks
	utilSetClockDMA(DMA1_Channel1,ENABLE);
	utilSetClockGPIO(BAT1_ADC_PORT,ENABLE);
	utilSetClockADC(ADC1,ENABLE);

	// Setup GPIO
	GPIO_InitStruct.GPIO_Pin 	= _BV(BAT1_ADC_PIN);// || _BV(BAT2_ADC_PIN) || _BV(BAT3_ADC_PIN);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AIN;
	GPIO_Init(BAT1_ADC_PORT, &GPIO_InitStruct);

	// DMA1 channel1 configuration (we almost always use ADC1, and its DMAChannel is 1)
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr 		= (uint32_t)(&adcRawBuffer[0]);
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize 			= ADC_NOF_CHANNELS;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_High;
	DMA_InitStructure.DMA_M2M 					= DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	// Enable DMA1 channel1
	DMA_Cmd(DMA1_Channel1, ENABLE);

	// ADC1 configuration
	ADC_InitStructure.ADC_Mode 					= ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode 			= ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode 	= ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv 		= ADC_ExternalTrigConv_None;	//This means only triggered by software
	ADC_InitStructure.ADC_DataAlign 			= ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel 			= ADC_NOF_CHANNELS;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_DMACmd(ADC1, ENABLE);

	//ADC_TempSensorVrefintCmd(ENABLE);
	// ADC1 regular channels configuration
	ADC_RegularChannelConfig(ADC1, BAT1_ADC_CHAN, BAT1_INDEX_NUM+1, ADC_SampleTime_239Cycles5);

	// Configure and enable ADC interrupt
	NVIC_InitStructure.NVIC_IRQChannel 						= ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable ADC2 EOC interrupt
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Enable ADC1 reset calibration register
	ADC_ResetCalibration(ADC1);
	// Check the end of ADC1 reset calibration register
	while(ADC_GetResetCalibrationStatus(ADC1));

	// Start ADC1 calibration
	ADC_StartCalibration(ADC1);
	// Check the end of ADC1 calibration
	while(ADC_GetCalibrationStatus(ADC1));

	// Start ADC1 Software Conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}


/*
 * ADC IRQ handler
 * Will be called whenever a conversion is completed
 */
void ADC1_2_IRQHandler()
{
	static uint8_t sampleNr=0;
	for(uint8_t i=0;i<ADC_NOF_CHANNELS;i++)
	{
		adcRawTotal[i]+=adcRawBuffer[i];
	}
	sampleNr++;
	if(sampleNr>=ADC_FILTER_SIZE)
	{
		sampleNr=0;
		for(uint8_t i=0;i<ADC_NOF_CHANNELS;i++)
		{
			adcFilteredVals[i]=adcRawTotal[i]/ADC_FILTER_SIZE;
			adcRawTotal[i]=0;
		}
	}
	//Start next conversion
	ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
}

/*
 * Returns the voltage of the batteri in mV
 */
uint16_t adcGetBatVolt(uint8_t channel)
{
	uint32_t millivolt=adcRawBuffer[BAT1_INDEX_NUM+channel-1]*MILLIVOLT_PER_BIT;
	return (uint16_t)((millivolt*BAT_VOLT_DIV_RATIO_INV);
}
