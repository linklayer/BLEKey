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
 * @defgroup pwm_analyzer_example_main main.c
 * @{
 * @ingroup pwm_analyzer_example
 * @brief PWM Analyzer Example Application main file.
 * 
 * This file contains the source code for a sample application using PWM 
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"

#define INPUT_PIN_NUMBER (BUTTON_0)    /**< Pin number for the input. */
#define DUTY_CYCLE_SCALE_VALUE (256UL) /**< Defines the upper limit of the duty cycle value. */

static void timer1_init(void);
static void gpiote_init(void);
static void ppi_init(void);

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    gpiote_init();
    ppi_init();
    timer1_init();

    NVIC_EnableIRQ(GPIOTE_IRQn);
    __enable_irq();

    while (true)
    {
        // Do nothing.
    }
}


/** @brief: Function for initializing the Timer 1 peripheral.
*/
static void timer1_init(void)
{
    // Start 16 MHz crystal oscillator
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    // Wait for the external oscillator to start up
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        // Do nothing.
    }

    NRF_TIMER1->MODE        = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->PRESCALER   = 4;
    NRF_TIMER1->BITMODE     = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER1->TASKS_CLEAR = 1;
    NRF_TIMER1->TASKS_START = 1; // Start clocks
}


/** @brief Function for handling the GPIOTE interrupt.
* Triggered on input Low-to-high transition.
*/
void GPIOTE_IRQHandler(void)
{
    static uint32_t prev_cc1 = 0;
    uint32_t        curr_cc1;
    uint32_t        cycle_duration;
    uint32_t        duty_cycle;
    uint32_t        active_time;
    
    curr_cc1 = NRF_TIMER1->CC[1];
    
    // Cycle duration is the difference between two low-to-high transitions.
    cycle_duration = (curr_cc1 - prev_cc1); 
    
    active_time = cycle_duration - ((curr_cc1 - NRF_TIMER1->CC[0]));

    if (cycle_duration != 0)
    {
        duty_cycle = (DUTY_CYCLE_SCALE_VALUE * active_time) / cycle_duration;
        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)duty_cycle);
    }
    else
    {
        // Do nothing.
    }

    // Clear the event causing the interrupt.
    NRF_GPIOTE->EVENTS_IN[1] = 0;    
    prev_cc1                 = curr_cc1;
}


/** @brief Function for initializing the GPIO Tasks/Events peripheral.
*/
static void gpiote_init(void)
{
    // Configure port1 (pins 8-15) as outputs for showing duty cycle.
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    nrf_gpio_cfg_sense_input(INPUT_PIN_NUMBER, BUTTON_PULL, NRF_GPIO_PIN_SENSE_LOW);

    // Enable interrupt on input 1 event.
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN1_Enabled << GPIOTE_INTENSET_IN1_Pos);

    // Configure GPIOTE channel 0 to generate event on input pin high-to-low transition.
    // Note that we can connect multiple GPIOTE events to a single input pin.
    nrf_gpiote_event_config(0, INPUT_PIN_NUMBER, NRF_GPIOTE_POLARITY_HITOLO);

    // Configure GPIOTE channel 1 to generate event on input pin low-to-high transition.
    // Note that we can connect multiple GPIOTE events to a single input pin.
    nrf_gpiote_event_config(1, INPUT_PIN_NUMBER, NRF_GPIOTE_POLARITY_LOTOHI);
}


/** @brief Function for initializing the PPI peripheral.
*/
static void ppi_init(void)
{
    // Configure PPI channel 0 to capture Timer 1 value into  the CC[0] register.
    // This is achieved when GPIOTE detects Low-to-High transition on pin INPUT_PIN_NUMBER.
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TIMER1->TASKS_CAPTURE[0];

    // Configure PPI channel 1 to capture Timer 1 value into CC[1] register.
    // This is achieved when GPIOTE detects High-to-Low transition on pin INPUT_PIN_NUMBER.
    NRF_PPI->CH[1].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[1];
    NRF_PPI->CH[1].TEP = (uint32_t)&NRF_TIMER1->TASKS_CAPTURE[1];

    // Enable only PPI channels 0 and 1.
    NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos)
                  | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);
}


/** @} */
