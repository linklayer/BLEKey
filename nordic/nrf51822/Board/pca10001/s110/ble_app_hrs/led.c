/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup ble_sdk_app_hrs_eval_led led.c
 * @{
 * @ingroup ble_sdk_app_hrs_eval
 * @brief LED control for the HRS example application
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "boards.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "led.h"
#include "app_util.h"

#define ADVERTISING_LED_PIN_NO               LED_0                                     /**< Is on when device is advertising. */

#define PPI_CHAN0_TO_TOGGLE_LED              0                                         /**< The PPI Channel that connects CC0 compare event to the GPIOTE Task that toggles the Advertising LED. */
#define GPIOTE_CHAN_FOR_LED_TASK             3                                         /**< The GPIOTE Channel used to perform write operation on the Advertising LED pin. */
#define TIMER_PRESCALER                      9                                         /**< Prescaler setting for timer. */
#define CAPTURE_COMPARE_0_VALUE              0x1E84                                    /**< Capture compare value that corresponds to 250 ms. */

/** @brief Function for Timer1 initialization.
 *
 * @details This function will initialise Timer 1 peripheral. This timer is used only to
 *          generate capture compare events that toggle the advertising LED state.
 */
static void timer1_init(void)
{
    // Configure timer
    NRF_TIMER1->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER1->PRESCALER = TIMER_PRESCALER;

    // Clear the timer
    NRF_TIMER1->TASKS_CLEAR = 1;

    // Load the value to TIMER1 CC0 register. The following value is calculated to generate
    // a 2 Hz waveform that will serve as input to the LED
    NRF_TIMER1->CC[0] = CAPTURE_COMPARE_0_VALUE;

    // Make the Capture Compare 0 event to clear the timer. This will restart the timer.
    NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

    // There is no need to setup NRF_TIMER1->INTENSET register because the application do not need
    // to wake up the CPU on Timer interrupts.
}


/** @brief Function for the PPI initialization.
 *
 * @details This function will initialise Programmable Peripheral Interconnect peripheral. It will
 *          configure the PPI channels as follows -
 *              PPI Channel 0 - Connecting CC0 Compare event to GPIOTE Task to toggle the LED state
 *          This configuration will feed a PWM input to the LED thereby making it flash in an
 *          interval that is dependent on the TIMER configuration.
 */
static void ppi_init(void)
{
    uint32_t err_code;

    // Configure PPI channel 0 to toggle ADVERTISING_LED_PIN_NO on every TIMER1 COMPARE[0] match
    err_code = sd_ppi_channel_assign(PPI_CHAN0_TO_TOGGLE_LED,
                                     &(NRF_TIMER1->EVENTS_COMPARE[0]),
                                     &(NRF_GPIOTE->TASKS_OUT[GPIOTE_CHAN_FOR_LED_TASK]));
    APP_ERROR_CHECK(err_code);

    // Enable PPI channel 0
    err_code = sd_ppi_channel_enable_set(PPI_CHEN_CH0_Msk);
    APP_ERROR_CHECK(err_code);
}

static void gpiote_init(void)
{
    // Configure the GPIOTE Task to toggle the LED state.
    nrf_gpiote_task_config(GPIOTE_CHAN_FOR_LED_TASK,
                           ADVERTISING_LED_PIN_NO,
                           NRF_GPIOTE_POLARITY_TOGGLE,
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
}

void led_start(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    
    ppi_init();
    timer1_init();
    gpiote_init();

    NRF_TIMER1->TASKS_START = 1;
}

void led_stop(void)
{
    // Disable the GPIOTE_CHAN_FOR_LED_TASK. This is because when an task has been configured
    // to operate on a pin, the pin can only be written from GPIOTE module. Attempting to write a
    // pin (using nrf_gpio_pin_clear() below for example) as a normal GPIO pin will have no effect.
    NRF_GPIOTE->CONFIG[GPIOTE_CHAN_FOR_LED_TASK] =
                                          (GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos);

    NRF_TIMER1->TASKS_STOP = 1;

    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
}

/**
 * @}
 */
