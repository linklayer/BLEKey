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
* @defgroup nrf_dev_radio_tx_example_main main.c
* @{
* @ingroup nrf_dev_radio_tx_example
*
* @brief Radio Transmitter Example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO to transmit.
*
* @image html example_board_setup_a.jpg "Use board setup A for this example."
*/
#include "nrf_gpio.h"
#include <stdint.h>
#include <stdbool.h>
#include "nrf_delay.h"
#include "radio_config.h"
#include "boards.h"


static uint8_t packet[PACKET_PAYLOAD_MAXSIZE];  /**< Packet to transmit. */

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

    // Set Port 0 as input.
    nrf_gpio_range_cfg_input(BUTTON_START, BUTTON_STOP, BUTTON_PULL);

    // Set Port 1 as output.
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    // Set radio configuration parameters.
    radio_configure();

    // Set payload pointer.
    NRF_RADIO->PACKETPTR = (uint32_t)packet;

    while (true)
    {
        // Read Data to send, button signals are default high, and low when pressed.
        packet[0]               = ~(nrf_gpio_port_read(NRF_GPIO_PORT_SELECT_PORT0));  // Write GPIO to payload byte 0.
        NRF_RADIO->EVENTS_READY = 0U;
        NRF_RADIO->TASKS_TXEN   = 1; // Enable radio and wait for ready.

        while (NRF_RADIO->EVENTS_READY == 0U)
        {
            // Do nothing.
        }

        // Start transmission.
        NRF_RADIO->TASKS_START = 1U;
        NRF_RADIO->EVENTS_END  = 0U;
    
        while (NRF_RADIO->EVENTS_END == 0U) // Wait for end of the transmission packet.
        {
            // Do nothing.
        }

        // Write sent data to port 1.
        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, packet[0]);

        NRF_RADIO->EVENTS_DISABLED = 0U;
        NRF_RADIO->TASKS_DISABLE   = 1U; // Disable the radio.

        while (NRF_RADIO->EVENTS_DISABLED == 0U)
        {
            // Do nothing.
        }
    }
}
/** @} */
