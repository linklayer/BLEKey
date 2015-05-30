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
* @defgroup twi_master_example_main main.c
* @{
* @ingroup twi_master_example
*
* @brief TWI Master Example Application main file.
*
* This file contains the source code for a sample application using TWI and external sensors. 
*
*/

#include "twi_master.h"
#include <stdbool.h>
#include <stdint.h>
#include "ds1624.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "synaptics_touchpad.h"

#define DS1624_ADDRESS      0x07    /**< Bits [2:0] describing how pins A2, A1 and A0 are wired. */
#define TOUCHPAD_ADDRESS    0x20    /**< Touchpad TWI address in bits [6:0]. */

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Delay for touchpad power up 
    nrf_delay_ms(400);

    bool touchpad_init_succeeded;
    bool ds1624_init_succeeded;
    bool m_conversion_in_progress = false;

    nrf_gpio_port_dir_set(NRF_GPIO_PORT_SELECT_PORT1, NRF_GPIO_PORT_DIR_OUTPUT);

    if (!twi_master_init())
    {
        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0x55);
        while (true) 
        {
            // Do nothing.
        }
    }

    touchpad_init_succeeded     = touchpad_init(TOUCHPAD_ADDRESS);
    ds1624_init_succeeded       = ds1624_init(DS1624_ADDRESS);

    // If both failed to initialized, halt here.
    if (!touchpad_init_succeeded && !ds1624_init_succeeded)
    {
        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0x5F);
        while (true) 
        {
            // Do nothing.
        }
    }

    while(true)
    {
        if (ds1624_init_succeeded)
        {
            if (!m_conversion_in_progress) 
            {
                m_conversion_in_progress = ds1624_start_temp_conversion();
            }
            else
            {
            // Succeeded.
                if (ds1624_is_temp_conversion_done())
                {
                    m_conversion_in_progress = false;
                    int8_t temperature;
                    int8_t temperature_fraction;
      
                    if (ds1624_temp_read(&temperature, &temperature_fraction))
                    {
                        nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT1, 0x7F);
                        nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)temperature);
                    }
                }
            }
        }

        if (touchpad_init_succeeded)
        {       
            uint8_t touchpad_button_status; // read button status.
            if (touchpad_read_register(TOUCHPAD_BUTTON_STATUS, &touchpad_button_status))
            {
                // There's a active low button on the side of the touchpad, check the state and light up a LED if it's pressed.
                nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT1, 0x80);
                if (!(touchpad_button_status & 0x01))
                {
                    nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT1, 0x80);
                }
            }
        }
    }
}
/** @} */
