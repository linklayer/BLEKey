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
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */
 
#include "simple_uart.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "boards.h"

//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define ERROR_PIN           (LED_0) /**< gpio pin number to show error if loopback is enabled. */
#define MAX_TEST_DATA_BYTES (15U)   /**< max number of test bytes to be used for tx and rx. */

#ifndef ENABLE_LOOPBACK_TEST

/** @brief Function for sending 'Exit!' string to UART. 
 * 
 *  @note Execution is blocked until UART peripheral detects all characters have been sent.
 */
static __INLINE void uart_quit()
{
    simple_uart_putstring((const uint8_t *)" \n\rExit!\n\r");
}


/** @brief Function for sending 'Start:' string to UART. 
 *  @details Execution is blocked until UART peripheral detects all characters have been sent.
 */
static __INLINE void uart_start()
{
    simple_uart_putstring((const uint8_t *)" \n\rStart: ");
}
#else

/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 *  This function is called if any of the nRF6350 functions fail.
 */
static void show_error(void)
{
    nrf_gpio_pin_write(ERROR_PIN, 1);
    while(true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back. 
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t *tx_data = (uint8_t *)("\n\rLOOPBACK_TEST");
    uint8_t  rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        bool status;
        simple_uart_put(tx_data[i]);
        status = simple_uart_get_with_timeout(2, &rx_data);

        if ((rx_data != tx_data[i]) || (!status))
        {
            show_error();
        }
    }
    return; 
}
#endif


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
    
    nrf_gpio_cfg_output(ERROR_PIN); // ERROR_PIN configured as output.

#ifndef ENABLE_LOOPBACK_TEST
    uart_start();
    while (true)
    {
        uint8_t cr = simple_uart_get();
        simple_uart_put(cr);

        if (cr == 'q' || cr == 'Q')
        {
            uart_quit();
            while (true)
            {
                // Do nothing.
            }
        }
    }
#else
  // This part of the example is just for testing the loopback .
  while(true)
  {
    uart_loopback_test();
  }
#endif
}
/** @} */
