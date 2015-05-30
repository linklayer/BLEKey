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
* @defgroup nrf_dev_radio_rx_example_main main.c
* @{
* @ingroup nrf_dev_radio_rx_example
*
* @brief Radio Receiver Example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO to receive.
*
* @image html example_board_setup_a.jpg "Use board setup A for this example."
*/
#include "radio_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

static uint8_t volatile packet[PACKET_PAYLOAD_MAXSIZE];  /**< Received packet buffer. */

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        // Do nothing.
    }
  
    // Set Port 1 as output.
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    // Set radio configuration parameters.
    radio_configure();

    nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0x55);

    while (true)
    {
        // Set payload pointer.
        NRF_RADIO->PACKETPTR    = (uint32_t)packet;
        NRF_RADIO->EVENTS_READY = 0U;        
        NRF_RADIO->TASKS_RXEN   = 1U; // Enable radio.

        while(NRF_RADIO->EVENTS_READY == 0U) // Wait for an event to be ready.
        {
            // Do nothing.
        }

        NRF_RADIO->EVENTS_END  = 0U;        
        NRF_RADIO->TASKS_START = 1U; // Start listening and wait for address received event.

        // Wait for the end of the packet.
        while (NRF_RADIO->EVENTS_END == 0U)
        {
            // Do nothing.
        }

        // Write received data to port 1 on CRC match.
        if (NRF_RADIO->CRCSTATUS == 1U)
        {
            nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, packet[0]);
        }

        NRF_RADIO->EVENTS_DISABLED = 0U;    
        NRF_RADIO->TASKS_DISABLE   = 1U;  // Disable the radio.

        while (NRF_RADIO->EVENTS_DISABLED == 0U)
        {
            // Do nothing.
        }
    }   

}
/** @} */
