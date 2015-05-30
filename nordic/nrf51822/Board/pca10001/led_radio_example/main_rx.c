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
* @defgroup nrf_dev_led_radio_rx_example_main main.c
* @{
* @ingroup nrf_dev_led_radio_rx_example
* @brief Radio Receiver example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO peripheral. 
*
*/
#include <stdint.h>
#include <stdbool.h>
#include "radio_config.h"
#include "nrf_gpio.h"
#include "boards.h"

static uint8_t volatile packet[4];  ///< Received packet buffer

void init(void)
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
    }

    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    // Set radio configuration parameters
    radio_configure();
}

/**
 * @brief Function for application main entry.
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
    init();

    while (true)
    {
        // Set payload pointer
        NRF_RADIO->PACKETPTR    = (uint32_t)packet;
        NRF_RADIO->EVENTS_READY = 0U;
        // Enable radio and wait for ready
        NRF_RADIO->TASKS_RXEN = 1U;
        while (NRF_RADIO->EVENTS_READY == 0U)
        {
        }
        NRF_RADIO->EVENTS_END = 0U;
        // Start listening and wait for address received event
        NRF_RADIO->TASKS_START = 1U;
        // Wait for end of packet
        while (NRF_RADIO->EVENTS_END == 0U)
        {
        }
        // Write received data to LED0 and LED1 on CRC match
        if (NRF_RADIO->CRCSTATUS == 1U)
        {
            switch (packet[0])
            {
                case '0':
                    nrf_gpio_pin_set(LED_0);
                    nrf_gpio_pin_clear(LED_1);
                    break;

                case '1':
                    nrf_gpio_pin_set(LED_1);
                    nrf_gpio_pin_clear(LED_0);
                    break;
            }
        }
        NRF_RADIO->EVENTS_DISABLED = 0U;
        // Disable radio
        NRF_RADIO->TASKS_DISABLE = 1U;
        while(NRF_RADIO->EVENTS_DISABLED == 0U)
        {
        }
    }
}

/**
 *@}
 **/
