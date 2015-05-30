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
* @defgroup ppi_example_main main.c
* @{
* @ingroup ppi_example
* @brief PPI Example Application main file.
*
* This file contains the source code for a sample application using PPI to communicate between timers.
*
* @image html example_board_setup_a.jpg "Use board setup A for this example."
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

/** @brief Function for initializing the PPI peripheral.
*/
static void ppi_init(void)
{
    // Configure PPI channel 0 to stop Timer 0 counter at TIMER1 COMPARE[0] match, which is every even number of seconds.
    NRF_PPI->CH[0].EEP = (uint32_t)(&NRF_TIMER1->EVENTS_COMPARE[0]);
    NRF_PPI->CH[0].TEP = (uint32_t)(&NRF_TIMER0->TASKS_STOP);

    // Configure PPI channel 1 to start timer0 counter at TIMER2 COMPARE[0] match, which is every odd number of seconds.
    NRF_PPI->CH[1].EEP = (uint32_t)(&NRF_TIMER2->EVENTS_COMPARE[0]);
    NRF_PPI->CH[1].TEP = (uint32_t)(&NRF_TIMER0->TASKS_START);

    // Enable only PPI channels 0 and 1.
    NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos) | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);
}


/** @brief Function for Timer 0 initialization, which will be started and stopped by timer1 and timer2 using PPI.
*/
static void timer0_init(void)
{
    NRF_TIMER0->MODE    = TIMER_MODE_MODE_Counter;      // Set the timer in counter Mode.
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;  // 24-bit mode.
}


/** @brief Function for Timer 1 initialization.
 *  @details Initializes Timer 1 peripheral, creates event and interrupt every 2 seconds,
 *           by configuring CC[0] to timer overflow value, we create events at even number of seconds
 *           for example, events are created at 2,4,6 ... seconds. This event can be used to stop Timer 0
 *           with Timer1->Event_Compare[0] triggering Timer 0 TASK_STOP through PPI.
*/
static void timer1_init(void)
{
    // Configure Timer 1 to overflow every 2 seconds.
    // SysClk = 16 Mhz
    // BITMODE = 16 bit
    // PRESCALER = 9
    // The overflow occurs every 0xFFFF/(SysClk/2^PRESCALER).
    // = 65535/31250 = 2.097 sec
    NRF_TIMER1->BITMODE   = (TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos);
    NRF_TIMER1->PRESCALER = 9;
    NRF_TIMER1->SHORTS    = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);

    // Trigger interrupt for compare[0] event.
    NRF_TIMER1->MODE  = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->CC[0] = 0xFFFFUL;  // Match at even number of seconds
}


/** @brief Function for Timer 2 initialization.
 *  @details Initializes Timer 2 peripheral, creates event and interrupt every 2 seconds
 *           by configuring CC[0] to half of timer overflow value. Events are created at odd number of seconds.
 *           For example, events are created at 1,3,5,... seconds. This event can be used to start Timer 0
 *           with Timer2->Event_Compare[0] triggering Timer 0 TASK_START through PPI.
*/
static void timer2_init(void)
{
    // Generate interrupt/event when half of time before the timer overflows has past, that is at 1,3,5,7... seconds from start.
    // SysClk = 16Mhz
    // BITMODE = 16 bit
    // PRESCALER = 9
    // now the overflow occurs every 0xFFFF/(SysClk/2^PRESCALER)
    // = 65535/31250 = 2.097 sec */
    NRF_TIMER2->BITMODE   = (TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos);
    NRF_TIMER2->PRESCALER = 9;
    NRF_TIMER2->SHORTS    = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);

    // Trigger interrupt for compare[0] event.
    NRF_TIMER2->MODE  = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->CC[0] = 0x7FFFUL;  // Match at odd number of seconds.
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    timer0_init();  // Timer used to blink the LEDs.
    timer1_init();  // Timer to generate events on even number of seconds.
    timer2_init();  // Timer to generate events on odd number of seconds.
    ppi_init();     // PPI to redirect the event to timer start/stop tasks.

    // Enabling constant latency as indicated by PAN 11 "HFCLK: Base current with HFCLK 
    // running is too high" found at Product Anomaly document found at
    // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
    //
    // @note This example does not go to low power mode therefore constant latency is not needed.
    //       However this setting will ensure correct behaviour when routing TIMER events through 
    //       PPI (shown in this example) and low power mode simultaneously.
    NRF_POWER->TASKS_CONSTLAT = 1;
    
    // Start clock.
    NRF_TIMER1->TASKS_START = 1;
    NRF_TIMER2->TASKS_START = 1;

    // Configure pins 8-15 as outputs.
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    // Loop and increment the timer count value and capture value into LEDs. @note counter is only incremented between TASK_START and TASK_STOP.
    while (true)
    {
        /* increment the counter */
        NRF_TIMER0->TASKS_COUNT      = 1;
        NRF_TIMER0->TASKS_CAPTURE[0] = 1;

        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)NRF_TIMER0->CC[0]);

        nrf_delay_ms(100);
    }
}
/** @} */
