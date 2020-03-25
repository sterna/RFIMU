/*
 * uart1.c
 *
 *  Created on: 22 jan 2013
 *      Author: peter
 */
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "uart.h"
#include "utils.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

static void uartPutc(uint8_t ch);

volatile unsigned char uartLastByte=0;
volatile char uartTxStringBuffer[UART_TX_BUFFER_SIZE];
volatile unsigned char uartDMAGotError = 0;
char* DMABuffer=0;
unsigned char UartDMABusy=0;



//Used to signal the rest of the program when a new message is received and such.
//When a message has been received, this is set to UART_STATUS_MSG_RECEIVED.
//When the message has been processed, it should be set UART_STATUS_WAIT_FOR_CMD
volatile unsigned char uartStatus=0;
volatile unsigned char uartCurrentCmd=0;
volatile unsigned char uartRxBuffer[20];

void uart1Init(unsigned long baud)
{

	//Default baud is 115200
	if (!baud)
	{
		baud=115200;
	}
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//This is not entirely correct...
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
	//Might not actually be needed
	//GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,1);
	//GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF);

	//Will collide with WS2812 for this project, ignore DMA
	uartInitDMA();

	// Setup the USART1 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//Enable interrupt on uart rx register not empty
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);

	USART_Cmd(USART1, ENABLE);

	//Setup xprintf etc
	xdev_out(uartPutc);
	xdev_in(uartGetc);

}

/*
 * Initializes the DMA for the uart
 */
void uartInitDMA()
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef nvicInitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);


	//Currently, DMA is not used for Uart Rx, since it would be harder to dynamically change the message structure
	/*//USART1 rx event DMA setup
	//Data goes from uart to memory every time a new byte of data is received
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_BASE+4;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uartRxStringBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = UART_RX_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);*/

	//USART1 tx event DMA setup
	// Data goes from memory to uart every time a byte of data has been sent
	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(USART1_BASE+4);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uartTxStringBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = UART_TX_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	//Configure the interrupt for DMA transfer.
	//Not very high prio, since we are not in a hurry to find out that the DMA has finished
	nvicInitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	nvicInitStructure.NVIC_IRQChannelSubPriority = 1;
	nvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicInitStructure);

	//Enable interrupt on transfer complete and transfer error
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC | DMA_IT_TE,ENABLE);

}

static void uartPutc(uint8_t ch) {
	USART_SendData(USART1, (uint8_t) ch);
	//Loop until the end of transmission
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
	}
}

/*
 * Fetch one byte from USART1 using busy wait
 */
unsigned char uartGetc()
{
	//Wait until byte is received
	while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)){}
	return (unsigned char)(USART1->DR);
}

void uartSendString(char* s)
{
	while(*s)
	{
		uartPutc(*s++);
	}
}

/*
 * Transmits data one the form ",<key>:<value>"
 * Note: Key must be a null-terminated string
 * Key + string must not longer than 20 characters be longer than
 */
void uartSendKeyVal(char* key, int32_t val, bool newline)
{
	//Wait until DMA is free
	while(uartGetDMAStatus()){}
	uint8_t c=0;
	c=xsprintf(uartTxStringBuffer,",%s:%d",key,val);
	if(newline)
	{
		uartTxStringBuffer[c++]='\n';
		uartTxStringBuffer[c]='\0';
	}
	uartSendStringDMA(uartTxStringBuffer,c,0);
}

/*
 * Sends a series of comma-separated values
 * Messages will be truncated to 50 chars
 */
void uartSendCSV(int32_t* vals,uint8_t n, bool newline)
{
	//Wait until DMA is free
	while(uartGetDMAStatus()){}
	uint8_t c=0;
	char* ptr=uartTxStringBuffer;
	uint8_t i=0;
	while(n-->0 && c<UART_TX_BUFFER_SIZE)
	{
		c+=xsprintf((char*)(ptr),"%d,",vals[i]);
		ptr=(char*)(uartTxStringBuffer+c);
		i++;
	}
	if(newline)
	{
		uartTxStringBuffer[c++]='\n';
		uartTxStringBuffer[c]='\0';
	}
	uartSendStringDMA(uartTxStringBuffer,c,false);
}

/*
 * Indicates if an uart DMA transfer is in progress
 */
unsigned char uartGetDMAStatus()
{
	return (UartDMABusy || !USART_GetFlagStatus(USART1, USART_FLAG_TC));
}

/*
 * Sends a string with uart1 using the DMA.
 * This loads the processor much less when sending large amounts of data compared to the standard busy wait
 * If this function is called when the DMA is active, that call is ignored and an error it returns false
 * @param sendString: Contains the null-terminated string to be sent
 * @param length: Specifies the length of the transfer.
 * 	if set to 0, the function calculates it, at the cost of some speed
 * @param copy: Specifies if the function should copy the string into its own buffer. This is to avoid cases where the source buffer is changed
 * Note: for some reason, the copy function does not quite work. If you can, avoid using it
 */
unsigned char uartSendStringDMA(char* sendString,unsigned short length, unsigned char copy)
{
	//If a DMA transfer is active, skip this transfer request
	if(!UartDMABusy && sendString)
	{
		DMA_Cmd(DMA1_Channel4,DISABLE);
		//If length is 0, count the number of characters to be sent
		if(length==0)
		{
			char* tempString=sendString;
			while(*tempString++)
			{
				length++;
			}
		}
		DMA_SetCurrDataCounter(DMA1_Channel4,length);
		//If copy is specified, allocate some memory and copy the data into that buffer
		if(copy)
		{
			DMABuffer = (calloc(length,sizeof(char)));
			memcpy(DMABuffer,sendString,length);
			DMA1_Channel4->CMAR = (uint32_t)DMABuffer;	//Tell DMA where to get the data
		}
		else
		{
			DMA1_Channel4->CMAR = (uint32_t)sendString;
		}
		//After this command, data will be sent automagically through the uart
		//When DMA is done, an interrupt is generated
		UartDMABusy=1;
		DMA_Cmd(DMA1_Channel4,ENABLE);
		return 1;
	}
	else
	{
		return 0;
	}
}



/*
 * Used to emulate putc, but use the DMA instead
 */
void uartPutcDMA(unsigned char c)
{
#define UART_WAIT_FOR_MSG 0
#define UART_BUFFERING_MSG 1
	static unsigned char state=UART_WAIT_FOR_MSG;
	static unsigned short len;
	switch (state)
	{
		case UART_WAIT_FOR_MSG:
			if(c)
			{
				state=UART_BUFFERING_MSG;
				len=0;
				uartTxStringBuffer[len++]=c;
			}
		break;
		case UART_BUFFERING_MSG:
			uartTxStringBuffer[len++]=c;
			if(c == '\0') //End of string, send data
			{
				uartSendStringDMA((char*)(uartTxStringBuffer),len,1);
				state=UART_WAIT_FOR_MSG;
			}
		break;
	}
}

void USART1_IRQHandler()
{
	static unsigned char dataCounter=0;
	unsigned short tmpLen=0;
	if(USART1->SR & USART_SR_RXNE)
	{
		uartLastByte = USART1->DR; //Reading data clears RXNE flag

		if (uartStatus == UART_STATUS_WAIT_FOR_CMD)
		{
			switch(uartLastByte)
			{
			//For testing purposes (like, are you alive?)
			case 'a':
				uartSendString("Hello!\n");
				break;
			case 'd':	//Test to see if the DMA is working
				uartSendStringDMA("DMA test transfer\n",0,0);
				break;
			case 'T':	//Used for various testing purposes
				uartStatus=UART_STATUS_CMD_RECEIVED;
				uartCurrentCmd=UART_CMD_TEST;
				break;
			case 'P':	//Set P in motor pid regulator. Value is a string converted to a floting point
				uartCurrentCmd = UART_CMD_SET_P;
				uartStatus = UART_STATUS_DURING_RECEPTION;
				dataCounter=0;	//Set to 0 to use when receiving a string
				break;
			case 'I':	//Set P in motor pid regulator. Value is a string converted to a floting point
				uartCurrentCmd = UART_CMD_SET_I;
				uartStatus = UART_STATUS_DURING_RECEPTION;
				dataCounter=0;	//Set to 0 to use when receiving a string
				break;
			case 'D':	//Set P in motor pid regulator. Value is a string converted to a floting point
				uartCurrentCmd = UART_CMD_SET_D;
				uartStatus = UART_STATUS_DURING_RECEPTION;
				dataCounter=0;	//Set to 0 to use when receiving a string
				break;
			case 'G':
				uartCurrentCmd = UART_CMD_GET_DATA;
				uartStatus = UART_STATUS_CMD_RECEIVED;
				//tmpLen=xsprintf(uartTxStringBuffer,"P: %s\nI: %s\nD: %s\n",motorRegPString,motorRegIString,motorRegDString);
				uartSendStringDMA((char*)uartTxStringBuffer,tmpLen,0);
				break;
			case 'L':	//Set left motor RPM
				uartCurrentCmd = UART_CMD_SET_LEFT_MOTOR_RPM;
				uartStatus = UART_STATUS_DURING_RECEPTION;
				dataCounter = 0; //Set to 0 to use when receiving a string
				break;
			case 'F':	//Save to flash
				break;
			}
		}
		else if(uartStatus == UART_STATUS_DURING_RECEPTION)
		{
			switch (uartCurrentCmd)
			{
				case UART_CMD_SET_P:
					uartRxBuffer[dataCounter]=uartLastByte;
					if(uartLastByte=='\n' || uartLastByte=='\0')
					{
						uartRxBuffer[dataCounter]='\0';
						uartStatus=UART_STATUS_CMD_RECEIVED;
					}
					dataCounter++;
					break;
				case UART_CMD_SET_I:
					uartRxBuffer[dataCounter]=uartLastByte;
					if(uartLastByte=='\n' || uartLastByte=='\0')
					{
						uartRxBuffer[dataCounter]='\0';
						uartStatus=UART_STATUS_CMD_RECEIVED;
					}
					dataCounter++;
					break;
				case UART_CMD_SET_D:
					uartRxBuffer[dataCounter]=uartLastByte;
					if(uartLastByte=='\n' || uartLastByte=='\0')
					{
						uartRxBuffer[dataCounter]='\0';
						uartStatus=UART_STATUS_CMD_RECEIVED;
					}
					dataCounter++;
					break;
				case UART_CMD_SET_LEFT_MOTOR_RPM:

					uartRxBuffer[dataCounter]=uartLastByte;
					if(uartLastByte=='\n' || uartLastByte=='\0')
					{
						uartRxBuffer[dataCounter]='\0';
						uartStatus=UART_STATUS_WAIT_FOR_CMD;
					}
					dataCounter++;
					break;
			}
		}
	} //End of check commands
	else
	{
		//This should never happen, since RXNE is the only source allowed to give interrupt
	}
}


/*
 * UART DMA interrupt handler
 */
void DMA1_Channel4_IRQHandler()
{
	//DMA error
	if(DMA1->ISR & DMA1_FLAG_TE4)
	{
		DMA1->IFCR = DMA1_FLAG_TE4;
		uartDMAGotError = 1;
	}
	else if(DMA1->ISR & DMA1_FLAG_TC4)
	{
		DMA1->IFCR = DMA1_FLAG_TC4;
		free(DMABuffer);
		DMABuffer=0;
		UartDMABusy=0;
	}
	DMA_Cmd(DMA1_Channel4,DISABLE);
}
