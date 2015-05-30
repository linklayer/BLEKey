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
 * @defgroup rng_example_main main.c
 * @{
 * @ingroup rng_example
 * @brief Random Number Generator Example Application main file.
 *
 * @details
 * This file contains the source code for a sample application using the Random Number Generator.
 * The following PAN workarounds must be taken into consideration (all but number 24 are demonstrated):
 * - PAN_028 rev2.0A anomaly 21 - RNG: Generated random value is reset when VALRDY event is cleared.
 * - PAN_028 rev2.0A anomaly 22 - RNG: RNG does not generate a new number after the current number generated
 * - PAN_028 rev2.0A anomaly 23 - RNG: STOP task clears the VALUE register
 * - PAN_028 rev2.0A anomaly 24 - RNG: The STOP task cannot be assigned to a PPI channel.
 *
 * In short the PANS add the following restrictions: 
 * - The random number must be read before VALRDY is cleared.
 * - EVENTS_VALRDY must be cleared to start generating a new value
 * - read VALUE before triggering STOP task.
 * - The random number generator has to be stopped by writing to the STOP task register.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */


#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"


/** @brief Function for main application entry.
 */
int main(void)
{
    // Configure pins 8-15 (port 1) for LEDs as outputs.
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT1, 0XFF);

    NRF_RNG->TASKS_START = 1; // start the RNG peripheral.
    while (true)
    {
        // Clear the VALRDY EVENT.
        NRF_RNG->EVENTS_VALRDY = 0;

        // Wait until the value ready event is generated.
        while (NRF_RNG->EVENTS_VALRDY == 0)
        {
            // Do nothing.
        }

        // Set output according to the random value.
        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)NRF_RNG->VALUE);
        nrf_delay_ms(100);
    }
}
/** @} */
