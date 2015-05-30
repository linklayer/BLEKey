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
*
* @defgroup nrf_dev_button_radio_rx_example_main main.c
* @{
* @ingroup nrf_dev_button_radio_rx_example
*
* @brief Radio Receiver Example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO peripheral.
*
*/
#include <stdint.h>
#include <stdbool.h>
#include "radio_config.h"
#include "nrf.h"
#include "boards.h"
#include "simple_uart.h"

static uint8_t volatile packet[4];  ///< Received packet buffer

void init(void)
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

   /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
    }

    // Set radio configuration parameters
    radio_configure();

    // Set payload pointer
    NRF_RADIO->PACKETPTR = (uint32_t)packet;

    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
}

/**
 * @brief Function for application main entry.
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
    init();
    simple_uart_putstring((const uint8_t *)"Press Button 0 or Button 1 on the transmitter\n\r");
    while(true)
    {
        NRF_RADIO->EVENTS_READY = 0U;
        // Enable radio and wait for ready
        NRF_RADIO->TASKS_RXEN = 1U;
        while(NRF_RADIO->EVENTS_READY == 0U)
        {
        }
        NRF_RADIO->EVENTS_END = 0U;
        // Start listening and wait for address received event
        NRF_RADIO->TASKS_START = 1U;
        // Wait for end of packet
        while(NRF_RADIO->EVENTS_END == 0U)
        {
        }
        // Write received data to LED0 and LED1 on CRC match
        if (NRF_RADIO->CRCSTATUS == 1U)
        {
          switch(packet[0])
          {
              case 1:
                  simple_uart_putstring((const uint8_t *)"Button 0 pressed\n\r");
                  break;

              case 2:
                  simple_uart_putstring((const uint8_t *)"Button 1 pressed\n\r");
                  break;

              case 3:
                  simple_uart_putstring((const uint8_t *)"Both buttons pressed\n\r");
                  break;
          }
        }
        NRF_RADIO->EVENTS_DISABLED = 0U;
        // Disable radio
        NRF_RADIO->TASKS_DISABLE = 1U;
        while (NRF_RADIO->EVENTS_DISABLED == 0U)
        {
        }
    }
}

/**
 *@}
 **/
