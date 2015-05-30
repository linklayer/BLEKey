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
* @defgroup gpiote_example_main main.c
* @{
* @ingroup nrf_gpiote_example
* @brief GPIOTE Example Application main file.
*
* This file contains the source code for a sample application using GPIOTE. 
*
* @image html example_board_setup_a.jpg "Use board setup A for this example."
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"


#define GPIO_OUTPUT_PIN_NUMBER LED_0  /**< Pin number for PWM output. */
#define GPIOTE_CHANNEL_NUMBER  0      /**< GPIOTE channel number. */


/** @brief Function for initializing the GPIO Tasks and Events peripheral.
*/
static void gpiote_init(void)
{
    // Configure GPIO_OUTPUT_PIN_NUMBER as an output.
    nrf_gpio_cfg_output(GPIO_OUTPUT_PIN_NUMBER);

    // Configure GPIOTE_CHANNEL_NUMBER to toggle the GPIO pin state with input.
    // @note Only one GPIOTE task can be coupled to an output pin.
    nrf_gpiote_task_config(GPIOTE_CHANNEL_NUMBER, GPIO_OUTPUT_PIN_NUMBER, \
                           NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
}

/** @brief Function for initializing Programmable Peripheral Interconnect (PPI) peripheral.
*   The PPI is needed to convert the timer event into a task.
 */
static void ppi_init(void)
{
    // Configure PPI channel 0 to toggle GPIO_OUTPUT_PIN on every TIMER0 COMPARE[0] match (200 ms)
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER0->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_CHANNEL_NUMBER];

    // Enable PPI channel 0
    NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos);
}


/** @brief Function for initializing the Timer 0 peripheral.
 */
static void timer0_init(void)
{
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        // Do nothing.
    }

    // Clear TIMER0
    NRF_TIMER0->TASKS_CLEAR = 1;

    // Configure TIMER0 for compare[0] event every 200 ms.
    NRF_TIMER0->PRESCALER = 4;                // Prescaler 4 results in 1 tick equals 1 microsecond.
    NRF_TIMER0->CC[0]     = 200*1000UL;       // 1 tick equals 1µ , multiply by 1000 for ms value.
    NRF_TIMER0->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->BITMODE   = TIMER_BITMODE_BITMODE_24Bit;
    NRF_TIMER0->SHORTS    = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    gpiote_init();                // Configure a GPIO to toggle on a GPIOTE task.
    timer0_init();                // Use TIMER0 to generate events every 200 ms.
    ppi_init();                   // Use a PPI channel to connect the event to the task automatically.

    NRF_TIMER0->TASKS_START = 1;  // Start event generation.

    while (true)
    {
        // Do Nothing - GPIO can be toggled without software intervention.
    }
}
/** @} */
