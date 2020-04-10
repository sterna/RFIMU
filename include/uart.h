/*
 * uart.h
 *
 *  Created on: 22 jan 2013
 *      Author: peter
 */

#ifndef UART_H_
#define UART_H_

#include <stdbool.h>
#include <stdint.h>
#include "xprintf.h"

void uart1Init(unsigned long baud);
void uartInitDMA();
void uartPutcDMA(unsigned char c);
unsigned char uartGetc();
void uartSendString(char* s);
unsigned char uartSendStringDMA(char* sendString,unsigned short length, unsigned char copy);
unsigned char uartGetDMAStatus();
void uartSendKeyVal(char* key, int32_t val, bool newline);
void uartSendCSV(int32_t* vals,uint8_t n, bool newline);

extern volatile unsigned char motorRegPString[10];
extern volatile unsigned char motorRegIString[10];
extern volatile unsigned char motorRegDString[10];

#define UART_TX_BUFFER_SIZE 100

extern volatile char uartTxStringBuffer[UART_TX_BUFFER_SIZE];

//This contains the last byte received by uart
extern volatile unsigned char uartLastByte;

extern volatile unsigned char uartStatus;
extern volatile unsigned char uartRxBuffer[20];
extern volatile unsigned char uartCurrentCmd;


//Uart commands
enum
{
	UART_CMD_SET_P=0,
	UART_CMD_SET_I,
	UART_CMD_SET_D,
	UART_CMD_TEST,
	UART_CMD_GET_DATA,
	UART_CMD_SET_LEFT_MOTOR_RPM,
	UART_CMD_SET_RIGHT_MOTOR_RPM,
	UART_CMD_NOF_COMMANDS
};

//Uart statuses
enum
{
	UART_STATUS_WAIT_FOR_CMD =0,
	UART_STATUS_DURING_RECEPTION,
	UART_STATUS_DATA_RECEIVED,
	UART_STATUS_CMD_RECEIVED
};

#endif /* UART_H_ */
