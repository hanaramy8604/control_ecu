/******************************************************************************
 *
 * Module: UART
 *
 * File Name: uart.c
 *
 * Description: Source file for the UART AVR driver
 *
 * Author: Modified for VFDLS Project
 *
 *******************************************************************************/

#include "uart.h"

/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/

void UART_init(const UART_ConfigType* Config_Ptr)
{
    uint16 ubrr_value = 0;

    /* U2X = 1 for double transmission speed */
    UCSRA = (1 << U2X);

    /* Enable UART Tx and Rx */
    UCSRB = (1 << RXEN) | (1 << TXEN);

    /* UCSRC Register Configuration:
     * URSEL = 1 to write to UCSRC
     * UMSEL = 0 for Asynchronous mode
     */
    UCSRC = (1 << URSEL);

    /* Configure Parity Mode (UPM1:0) */
    UCSRC = (UCSRC & 0xCF) | ((Config_Ptr->parity) << UPM0);

    /* Configure Stop Bit (USBS) */
    UCSRC = (UCSRC & 0xF7) | ((Config_Ptr->stop_bit) << USBS);

    /* Configure Character Size (UCSZ1:0 in UCSRC, UCSZ2 in UCSRB) */
    if (Config_Ptr->bit_data == UART_9_BIT) {
        /* 9-bit mode: UCSZ2=1, UCSZ1=1, UCSZ0=1 */
        UCSRB |= (1 << UCSZ2);
        UCSRC |= (1 << UCSZ1) | (1 << UCSZ0);
    } else {
        /* 5/6/7/8-bit mode: UCSZ2=0, UCSZ1:0 from bit_data */
        UCSRB &= ~(1 << UCSZ2);
        UCSRC = (UCSRC & 0xF9) | ((Config_Ptr->bit_data) << UCSZ0);
    }

    /* Calculate baud rate register value
     * UBRR = (F_CPU / (8 * BAUD_RATE)) - 1  [When U2X = 1]
     */
    ubrr_value = (uint16)(((F_CPU / (Config_Ptr->baud_rate * 8UL))) - 1);

    /* Set baud rate registers */
    UBRRH = (uint8)(ubrr_value >> 8);
    UBRRL = (uint8)ubrr_value;
}

void UART_sendByte(const uint8 data)
{
    /* Wait until the Tx buffer (UDR) is empty (UDRE flag is set) */
    while(BIT_IS_CLEAR(UCSRA, UDRE));

    /* Put the data in UDR register to send it */
    UDR = data;
}

uint8 UART_recieveByte(void)
{
    /* Wait until data is received (RXC flag is set) */
    while(BIT_IS_CLEAR(UCSRA, RXC));

    /* Read and return the received data from UDR */
    return UDR;
}

void UART_sendString(const uint8 *Str)
{
    uint8 i = 0;

    /* Send each character until null terminator */
    while(Str[i] != '\0')
    {
        UART_sendByte(Str[i]);
        i++;
    }
}

void UART_receiveString(uint8 *Str)
{
    uint8 i = 0;

    /* Receive characters until '#' is received */
    Str[i] = UART_recieveByte();

    while(Str[i] != '#')
    {
        i++;
        Str[i] = UART_recieveByte();
    }

    /* Replace '#' with null terminator */
    Str[i] = '\0';
}
