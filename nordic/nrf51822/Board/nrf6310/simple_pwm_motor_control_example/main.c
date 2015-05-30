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
* @defgroup simple_motor_control_main main.c
* @{
* @ingroup simple_motor_control
* @brief Simple Motor Control Example Application main file.
*
* This file contains the source code for a sample application using PWM that could control a motor.
*
* @image html example_board_setup_a.jpg "Use board setup A for this example."
*
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "boards.h"

#define PWM_OUTPUT_PIN_NUMBER (LED_0) /**< Pin number for PWM output. */
#define TIMER_PRESCALER       (4)     /**< Prescaler setting for timers. */
#define LED_INTENSITY_HIGH    (224U)  /**< High intensity. */
#define LED_INTENSITY_LOW     (32U)   /**< Low intensity. */
#define LED_OFF               (1U)    /**< Led off. */
#define LED_INTENSITY_HALF    (128U)  /**< Half intensity. Used to calculate timer parameters. */


/** @brief Function for setting the PWM duty cycle.
 *  @param[in] new_setting New duty cycle, where 128 results in 50% duty cycle.
 */
static void pwm_set(uint8_t new_setting)
{
    NRF_TIMER2->CC[0] = new_setting;
}


/** @brief Function for initializing the Timer 2 peripheral.
 */
static void timer2_init(void)
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
  
    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)       
    {
        // Do nothing.
    }
  
    NRF_TIMER2->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->PRESCALER = 4;

    /* Load initial values to Timer 2 CC registers */
    /* Set initial CC0 value to anything > 1 */
    NRF_TIMER2->CC[0] = LED_INTENSITY_LOW;
    NRF_TIMER2->CC[1] = (LED_INTENSITY_HALF*2);

    /* Set up interrupt for CC2 */
    /* This interrupt is to force the change of CC0 to happen when it is safe */
    /* A safe time is after the highest possible CC0 value, but before the lowest one. */
    NRF_TIMER2->CC[2]    = LED_INTENSITY_HIGH;
    NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE2_Enabled << TIMER_INTENSET_COMPARE2_Pos;

    /* Create an Event-Task shortcut to clear TIMER2 on COMPARE[1] event. */
    NRF_TIMER2->SHORTS = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos);
}


/** @brief Function for initializing the GPIO peripheral.
*/
static void gpiote_init(void)
{
    NRF_GPIO->OUT    = 0x00000000UL;
    NRF_GPIO->DIRSET = 0x0000FF00UL;
    NRF_GPIO->DIRCLR = 0x000000FFUL;

    /* Configuring Button 0 as input */
    nrf_gpio_cfg_input(BUTTON_0, BUTTON_PULL);

    /* Configuring Button 1 as input. */
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    nrf_gpio_cfg_input(BUTTON_1, BUTTON_PULL);

    /* Configuring Pin PWM_OUTPUT_PIN_NUMBER as output to be used for the PWM waveform. */
    nrf_gpio_cfg_output(PWM_OUTPUT_PIN_NUMBER);
  
    /* Configure GPIOTE channel 0 to toggle the PWM pin state. */
    /* Note that we can only connect one GPIOTE task to one output pin. */
    nrf_gpiote_task_config(0, PWM_OUTPUT_PIN_NUMBER, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
}


/** @brief Function for initializing the PPI peripheral.
*/
static void ppi_init(void)
{
    /* Configure PPI channel 0 to toggle PWM_OUTPUT_PIN on every Timer 2 COMPARE[0] match. */
    NRF_PPI->CH[0].EEP  = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP  = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

    /* Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every Timer 2 COMPARE[1] match. */
    NRF_PPI->CH[1].EEP  = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[1];
    NRF_PPI->CH[1].TEP  = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

    /* Enable only PPI channels 0 and 1. */
    NRF_PPI->CHEN       = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos) |
                          (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);
}


/** @brief Function for handling the Timer 2 interrupt.
 */
void TIMER2_IRQHandler(void)
{
    // Clear interrupt.
    if ((NRF_TIMER2->EVENTS_COMPARE[2] == 1) && 
        (NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE2_Msk))
    {
        NRF_TIMER2->EVENTS_COMPARE[2] = 0;
    }
    // Process buttons.
    if (nrf_gpio_pin_read(BUTTON_1) == 0)
    {
        pwm_set(LED_INTENSITY_HIGH);
    }
    else if (nrf_gpio_pin_read(BUTTON_0) == 0)
    {
        pwm_set(LED_INTENSITY_LOW);
    }
    else
    {
        pwm_set(LED_OFF);
    }
}


/** @brief Function for main application entry.
 */
int main(void)
{  
    gpiote_init();
    ppi_init();
    timer2_init();

    // Enable interrupt on Timer
    NVIC_EnableIRQ(TIMER2_IRQn);
    __enable_irq();

    // Enabling constant latency as indicated by PAN 11 "HFCLK: Base current with HFCLK 
    // running is too high" found at Product Anomaly document found at
    // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
    //
    // @note This example does not go to low power mode therefore constant latency is not needed.
    //       However this setting will ensure correct behaviour when routing TIMER events through 
    //       PPI (shown in this example) and low power mode simultaneously.
    NRF_POWER->TASKS_CONSTLAT = 1;

    // Start the timer.
    NRF_TIMER2->TASKS_START = 1;
    while (true)
    {
        // Do nothing.
    }
}
/** @} */
