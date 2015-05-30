 /* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>

/** Reads a character from UART.
Execution is blocked until the UART peripheral detects that a character has been received.
\return cr Received character.
*/
uint8_t uart_get(void);


/** Sends a character to UART.
Execution is blocked until the UART peripheral reports that a character has been sent.
@param[in] cr Character to send.
*/
void uart_put(uint8_t cr);

/** Sends a string to UART.
Execution is blocked until the UART peripheral reports that all characters to have been sent.
Maximum string length is 254 characters including null character in the end.
@param[in] str Null terminated string to send.
*/
void uart_putstring(const uint8_t *str);

/** Configures UART to use 38400 baud rate.
@param[in] txd_pin_number Pin number used for UART TXD
@param[in] rxd_pin_number Pin number used for UART RXD
*/
void uart_config(uint8_t txd_pin_number, uint8_t rxd_pin_number);

#endif
