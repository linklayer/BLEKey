/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 * @defgroup cherry8x16_example_main main.c
 * @{
 * @ingroup cherry8x16_example
 *
 * @brief Cherry8x16 Keyboard Application main file.
 *
 * This file contains the source code for an application using the Cherry8x16 Keyboard.
 */

#include "cherry8x16.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "simple_uart.h"
#include "nrf_gpio.h"

#define USE_UART // Comment out to use pins 8-15 for LEDs. Otherwise UART will be used to transmit keystrokes.

/** @def  SIMPLE_UART_TXD_PIN_NUMBER
 * Pin number to use as UART TXD. LEDs for modifier key presses and UART cannot be used at the same time as both use SIMPLE_UART_TXD_PIN_NUMBER.
 * Comment out UART code if using LEDs for modifier keys (and comment out the LED code if using UART). */
#define SIMPLE_UART_TXD_PIN_NUMBER (24U)

/** @def  SIMPLE_UART_RXD_PIN_NUMBER
 * Pin number to use as UART RXD. LEDs for modifier key presses and UART cannot be used at the same time as both use SIMPLE_UART_RXD_PIN_NUMBER.
 * Comment out UART code if using LEDs for modifier keys (and comment out the LED code if using UART). */
#define SIMPLE_UART_RXD_PIN_NUMBER (25U)

/** @var static const uint8_t volatile * const matrix_row_port
 * Pointer to GPIO memory used as a row port. No pointer alignment required for pointing to 8-bit values in this memory region.
 */
static const uint8_t volatile * const matrix_row_port = ((uint8_t*)(&NRF_GPIO->IN)) + 2;

/** @var static uint16_t * const matrix_column_port
 * Pointer to GPIO memory used as a column port. Half-word alignment is required for pointing to 16-bit values in this memory region.
 */
static uint16_t * const matrix_column_port = ((uint16_t*)(&NRF_GPIO->OUT));

#define KEYPAD_NUM_OF_COLUMNS 16 //!< Number of columns in the keyboard matrix.
#define KEYPAD_NUM_OF_ROWS    8  //!< Number of rows in the keyboard matrix.

#ifdef USE_UART


/** @var static const uint8_t usb_hid2_ascii_look_up
 * Table containing the mapping between the USB HID codes and ASCII values for most of the keys.
 */
static const uint8_t usb_hid2_ascii_look_up[KEYPAD_NUM_OF_COLUMNS*KEYPAD_NUM_OF_ROWS] =
{
  0x00, 0x00, 0x00, 0x00, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46,
  0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50,
  0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A,
  0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30,
  0x0D, 0x1B, 0x08, 0x09, 0x20, 0x2D, 0x3D, 0x5B, 0x5D, 0x00,
  0x00, 0x3B, 0x27, 0x00, 0x2C, 0x2E, 0x2F, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


/** @brief Function for sending a series of characters (string) to UART.
 * Execution is blocked until UART peripheral reports a character has been sent.
 * @param[in] str Null terminated array of characters to send.
 */
static void uart_puthidstring(const char *str)
{
    uint_fast8_t i  = 0;
    char         ch = str[i++];
    while (ch != '\0')
    {
        simple_uart_put(usb_hid2_ascii_look_up[(uint8_t)ch]);
        ch = str[i++];
    }
}
#endif


/**
 * @brief Function for application main entry.
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
    const uint8_t *key_packet;
    uint8_t        key_packet_size;

#ifdef USE_UART
    simple_uart_config(0, SIMPLE_UART_TXD_PIN_NUMBER, 0, SIMPLE_UART_RXD_PIN_NUMBER, false); // Hardware flow control not used in this example.
#else
    // Configure pins 24-30 for LEDs. Note that pin 31 is not connected.
    nrf_gpio_range_cfg_output(24, 30);
#endif

    // Enable pulldowns on row port, see matrix_row_port.
    nrf_gpio_range_cfg_input(16, 23, NRF_GPIO_PIN_PULLDOWN);

    // Column pin configuration, see matrix_column_port.
    nrf_gpio_range_cfg_output(0, 15);

    if (cherry8x16_init(matrix_row_port, matrix_column_port, CHERRY8x16_DEFAULT_KEY_LOOKUP_MATRIX) != CHERRY8x16_OK)
    {
#ifdef USE_UART
        simple_uart_putstring((const uint8_t *)"Init failed.");
#else
        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT3, 0x55);
#endif
        while (true) 
        {
            // Do nothing.
        }
    }

    while(true)
    {
        if (cherry8x16_new_packet(&key_packet, &key_packet_size))
        {
#ifdef USE_UART
            // Send the whole key packet over UART.
            uart_puthidstring((char *)&key_packet[KEY_PACKET_KEY_INDEX]);
#else
            // Show modifier key state using the LEDs. Note LED's use GPIO pins from 8 to 15.
        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT3, key_packet[KEY_PACKET_MODIFIER_KEY_INDEX]);
#endif
        }
        nrf_delay_ms(25);
    }
}

/** @} */
