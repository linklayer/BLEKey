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
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 * 
 * @brief  PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"

#define PWM_OUTPUT_PIN_NUMBER (LED_0)  /**< Pin number for PWM output. */

#define MAX_SAMPLE_LEVELS (256UL)  /**< Maximum number of sample levels. */
#define TIMER_PRESCALERS  6U       /**< Prescaler setting for timer. */

/** @brief Function for getting the next sample.
 *  @return sample_value computed sample.
 */
static __INLINE uint32_t next_sample_get(void)
{
    static uint32_t sample_value = 8;
    
    // Read button input.
    sample_value = (~(nrf_gpio_port_read(NRF_GPIO_PORT_SELECT_PORT0)) & 0x000000FFUL);
  
    // This is to avoid having two CC events happen at the same time,
    // CC1 will always create an event on 0 so CC0 and CC2 should not.
    if (sample_value == 0) 
    {
        sample_value = 8;
    }

    return (uint32_t)sample_value;
}


/** @brief Function for handling timer 2 peripheral interrupts.
 */
void TIMER2_IRQHandler(void)
{
    static bool cc0_turn = false; /**< Keeps track of which CC register to be used. */

    if ((NRF_TIMER2->EVENTS_COMPARE[1] != 0) && 
       ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
    {
        // Sets the next CC1 value
        NRF_TIMER2->EVENTS_COMPARE[1] = 0;
        NRF_TIMER2->CC[1]             = (NRF_TIMER2->CC[1] + MAX_SAMPLE_LEVELS);
    
        // Every other interrupt CC0 and CC2 will be set to their next values.
        uint32_t next_sample = next_sample_get();

        if (cc0_turn)
        {
            NRF_TIMER2->CC[0] = NRF_TIMER2->CC[1] + next_sample;
        }
        else
        {
            NRF_TIMER2->CC[2] = NRF_TIMER2->CC[1] + next_sample;
        }
        // Next turn the other CC will get its value.
        cc0_turn = !cc0_turn;
    }
}


/** @brief Function for initializing the Timer 2 peripheral.
 */
static void timer2_init(void)
{
    // Start 16 MHz crystal oscillator .
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        //Do nothing.
    }

    NRF_TIMER2->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER2->PRESCALER = TIMER_PRESCALERS;

    // Clears the timer, sets it to 0.
    NRF_TIMER2->TASKS_CLEAR = 1;

    // Load the initial values to TIMER2 CC registers.
    NRF_TIMER2->CC[0] = MAX_SAMPLE_LEVELS + next_sample_get();
    NRF_TIMER2->CC[1] = MAX_SAMPLE_LEVELS;

    // CC2 will be set on the first CC1 interrupt.
    NRF_TIMER2->CC[2] = 0;

    // Interrupt setup.
    NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
}


/** @brief Function for initializing the GPIO Tasks/Events peripheral.
 */
static void gpiote_init(void)
{
    // Connect GPIO input buffers and configure PWM_OUTPUT_PIN_NUMBER as an output.
    nrf_gpio_range_cfg_input(BUTTON_START, BUTTON_STOP, BUTTON_PULL);
    nrf_gpio_cfg_output(PWM_OUTPUT_PIN_NUMBER);

    nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT0, 0xFF);

    // Configure GPIOTE channel 0 to toggle the PWM pin state
    // @note Only one GPIOTE task can be connected to an output pin.
    nrf_gpiote_task_config(0, PWM_OUTPUT_PIN_NUMBER, \
                           NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
}


/** @brief Function for initializing the Programmable Peripheral Interconnect peripheral.
 */
static void ppi_init(void)
{
    // Configure PPI channel 0 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[0] match.
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

    // Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[1] match.
    NRF_PPI->CH[1].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[1];
    NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];
    
    // Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[2] match.
    NRF_PPI->CH[2].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[2];
    NRF_PPI->CH[2].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];
    
    // Enable PPI channels 0-2.
    NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos)
                    | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos)
                    | (PPI_CHEN_CH2_Enabled << PPI_CHEN_CH2_Pos);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    gpiote_init();
    ppi_init();
    timer2_init();

    // Enabling constant latency as indicated by PAN 11 "HFCLK: Base current with HFCLK 
    // running is too high" found at Product Anomaly document found at
    // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
    //
    // @note This example does not go to low power mode therefore constant latency is not needed.
    //       However this setting will ensure correct behaviour when routing TIMER events through 
    //       PPI (shown in this example) and low power mode simultaneously.
    NRF_POWER->TASKS_CONSTLAT = 1;

    // Enable interrupt on Timer 2.
    NVIC_EnableIRQ(TIMER2_IRQn);
    __enable_irq();

    // Start the timer.
    NRF_TIMER2->TASKS_START = 1;

    while (true)
    {
        // Do nothing.
    }
}

/** @} */
