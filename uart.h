/******************************************************************************
 *
 * Module: UART
 *
 * File Name: uart.h
 *
 * Description: Header file for the UART AVR driver
 *
 * Author: Modified for VFDLS Project
 *
 *******************************************************************************/

#ifndef UART_H_
#define UART_H_

#include "micro_config.h"
#include "std_types.h"
#include "common_macros.h"

/*******************************************************************************
 *                         Types Declaration                                   *
 *******************************************************************************/

typedef enum {
    UART_5_BIT = 0,
    UART_6_BIT = 1,
    UART_7_BIT = 2,
    UART_8_BIT = 3,
    UART_9_BIT = 7
} UART_BitDataType;

typedef enum {
    UART_NO_PARITY = 0,
    UART_EVEN_PARITY = 2,
    UART_ODD_PARITY = 3
} UART_ParityType;

typedef enum {
    UART_1_STOP_BIT = 0,
    UART_2_STOP_BIT = 1
} UART_StopBitType;

typedef uint32 UART_BaudRateType;

typedef struct {
    UART_BitDataType bit_data;
    UART_ParityType parity;
    UART_StopBitType stop_bit;
    UART_BaudRateType baud_rate;
} UART_ConfigType;

/*******************************************************************************
 *                      Functions Prototypes                                   *
 *******************************************************************************/

/*
 * Description: Function to initialize the UART driver with configuration structure
 */
void UART_init(const UART_ConfigType* Config_Ptr);

/*
 * Description: Function to send byte through UART
 */
void UART_sendByte(const uint8 data);

/*
 * Description: Function to receive byte through UART
 */
uint8 UART_recieveByte(void);

/*
 * Description: Function to send string through UART
 */
void UART_sendString(const uint8 *Str);

/*
 * Description: Function to receive string through UART (until #)
 */
void UART_receiveString(uint8 *Str);

#endif /* UART_H_ */
