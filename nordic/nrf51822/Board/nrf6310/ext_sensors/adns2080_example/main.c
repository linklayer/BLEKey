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
* @defgroup adns2080_example_main main.c
* @{
* @ingroup adns2080_example
*
* @brief ADNS2080 Mouse Sensor Driver Application main file.
*
*  This file contains the source code for an application using ADNS2080 mouse sensor driver.
*
*/

#include <stdbool.h>
#include <stdint.h>

#include "adns2080.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"

/* File sdio_config.h contains pin configuration for SDIO clock and data. It is included by sdio.c. */

#define MOTION_INTERRUPT_PIN_NUMBER (26) /**< Pin number used for ADNS2080 motion interrupt. If you change this, you must change the pin configuration in the main function. */
#define MOUSE_MOVEMENT_THRESHOLD    (10) /**< Deadzone for mouse movement before LEDs are lit. */

static int16_t m_delta_x = 0; /**< Variable to store mouse X-axis movement deltas. */
static int16_t m_delta_y = 0; /**< Variable to store mouse Y-axis movement deltas. */

static bool volatile motion_interrupt_detected = false; /**< If set, motion interrupt has occurred. Clear after reading. */


/** @brief Function for initializing the GPIO Tasks/Events peripheral.
*/
static void gpiote_init(void)
{
    // Configure GPIOTE channel 0 to generate event when MOTION_INTERRUPT_PIN_NUMBER goes from Low to High
    nrf_gpiote_event_config(0, MOTION_INTERRUPT_PIN_NUMBER, NRF_GPIOTE_POLARITY_LOTOHI);

    // Enable interrupt for NRF_GPIOTE->EVENTS_IN[0] event
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
}


/** @brief Function for handling the GPIOTE interrupt.
* Triggered on motion interrupt pin input low-to-high transition.
*/
void GPIOTE_IRQHandler(void)
{
    motion_interrupt_detected = true;    
    NRF_GPIOTE->EVENTS_IN[0]  = 0; // Event causing the interrupt must be cleared
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Configure pins 8-15 (Port1) as outputs
    nrf_gpio_range_cfg_output(8, 15);
    nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT1, 0XFF);

    // Configure motion interrupt pin
    nrf_gpio_cfg_input(MOTION_INTERRUPT_PIN_NUMBER, NRF_GPIO_PIN_PULLDOWN);

    gpiote_init();

    if (adns2080_init() != ADNS2080_OK)
    {
        // ADNS2080 init failed, set rightmost LED on.
        nrf_gpio_pin_write(15, 1);
        while (true)
        {
            //  Do nothing.
        }
    }

    // By default, ADNS2080 motion interrupt output is active low, edge sensitive; make it active high.
    if (adns2080_motion_interrupt_set(ADNS2080_MOTION_OUTPUT_POLARITY_HIGH, ADNS2080_MOTION_OUTPUT_SENSITIVITY_LEVEL) != ADNS2080_OK)
    {
        nrf_gpio_pin_write(14, 1);
        while (true)
        {
            // Do nothing.
        }
    }

    // Read out movement to clear ADNS2080 interrupt flags.
    if (adns2080_is_motion_detected())
    {
        int16_t dummy;
        adns2080_movement_read(&dummy, &dummy);
    }

    // Enable GPIOTE interrupt in Nested Vector Interrupt Controller.
    NVIC_EnableIRQ(GPIOTE_IRQn);

    // Enable global interrupts.
    __enable_irq();

    while(true)
    {
        if (motion_interrupt_detected)
        {
            // Toggle pin 12 to indicate that we've detected motion interrupt.
            nrf_gpio_pin_toggle(12);
            
            motion_interrupt_detected = false;

            // On our Nordic reference design PCB, the chip orientation is not the same as in the ADNS2080
            // datasheet diagram, so X and Y axis are reversed. This is corrected by passing the pointer
            // parameters in reversed order. */
            adns2080_movement_read(&m_delta_y, &m_delta_x);

            // If movement delta_x is above the threshold, the LEDs light up accordingly. When the mouse is moved
            // to the left, LEDs 1 through 4 are lit and when the mouse is moved right, LEDs 5 through 8 are lit.
            if (m_delta_x > MOUSE_MOVEMENT_THRESHOLD)
            {
                nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0xF0);
            }
            else if (m_delta_x < (-MOUSE_MOVEMENT_THRESHOLD))
            {
                nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0x0F);
            }
            else
            {
                nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT1, 0xFF);
            }
        }
    }
}
/** @} */
