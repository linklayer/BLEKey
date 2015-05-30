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
 * @defgroup debouncer_example_main main.c
 * @{
 * @ingroup debouncer_example
 * @brief Button Debounce Example Application main file.
 *
 * This file contains the source code for a sample application using button debouncing.
 * @image html example_board_setup_a.jpg "Use board setup A for this example. "
 */

#include "lib_debounce.h"
#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nrf.h"
#include "nrf_gpio.h"


#define DEBOUNCE_TIME_IN_MS          (50U)                                  /**< Debounce timer in milliseconds */
#define DEBOUNCE_INPUT_SAMPLING_FREQ (60U)                                  /**< Input sampling frequency in Hertz */
#define TIMER0_PRESCALER             (9UL)                                  /**< Timer 0 prescaler */
#define TIMER0_CLOCK                 (SystemCoreClock >> TIMER0_PRESCALER)  /**< Timer clock frequency */
#define MS_TO_TIMER0_TICKS(ms)       ((1000000UL * ms) / (TIMER0_CLOCK))    /**< Converts milliseconds to timer ticks */
#define MAX_BUTTONS                  (8U)                                   /**< Maximum number of buttons in use */

static uint_fast16_t timer0_cc0_period;          /**< Period between debouncer input reads. */
static               deb_t button[MAX_BUTTONS];  /**< Debounced button state holder */


/** @brief Interrupt handler function for the Timer 0 peripheral.
 */
void TIMER0_IRQHandler(void)
{
    if ((NRF_TIMER0->EVENTS_COMPARE[0] != 0) && \
        ((NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
    {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        NRF_TIMER0->CC[0]            += timer0_cc0_period;
        
        for (uint_fast8_t button_index = 0; button_index < MAX_BUTTONS; button_index++)
        {
            debounce(nrf_gpio_pin_read(button_index), &button[button_index]);
        }
    }
}


/** @brief Function for initializing Timer 0.
 */
static void timer0_init(void)
{
    // Set the timer in Timer Mode
    NRF_TIMER0->MODE      = TIMER_MODE_MODE_Timer; 
    NRF_TIMER0->PRESCALER = TIMER0_PRESCALER;
    
    // 24-bit mode
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;  

    // Enable interrupt for COMPARE[0]
    NRF_TIMER0->INTENSET    = (1UL << TIMER_INTENSET_COMPARE0_Pos);
    NRF_TIMER0->CC[0]       = timer0_cc0_period;
    NRF_TIMER0->TASKS_START = 1; // Start clocks
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // DEBOUNCE_INPUT_SAMPLING_FREQ is in hertz, Multiply by 1000 to get milliseconds
    timer0_cc0_period = MS_TO_TIMER0_TICKS((1000 * 1 / DEBOUNCE_INPUT_SAMPLING_FREQ));
    timer0_init();

    // Enable Interrupt for the timer in the core
    NVIC_EnableIRQ(TIMER0_IRQn); 
    __enable_irq();

    // Configure pins 0-7 as inputs
    nrf_gpio_range_cfg_input(BUTTON_START, BUTTON_STOP, BUTTON_PULL);

    // Configure pins 8-15 as outputs
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    NRF_GPIO->OUT = 0UL;

    for (uint_fast8_t button_index = 0; button_index < MAX_BUTTONS; button_index++)
    {
        debounce_init(&button[button_index], DEBOUNCE_TIME_IN_MS, DEBOUNCE_INPUT_SAMPLING_FREQ);
    }

    while (true)
    {
        for (uint_fast8_t button_index = 0; button_index < MAX_BUTTONS; button_index++)
        {
            // Map buttons to leds
            nrf_gpio_pin_write((LED_START + button_index), !button[button_index].output_state);
        }
    }
}
/** @} */
