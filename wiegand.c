#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf51.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "retarget.h"
#include "wiegand.h"

// wiegand data pins
#define DATA0_IN 0
#define DATA1_IN 1

#define TIMER_DELAY 3000 // Timer is set at 1Mhz, 3000 ticks = 3ms
#define MAX_BITS 100

volatile uint8_t data_bits[MAX_BITS];
volatile uint8_t bit_count = 0;
volatile bool data_incoming = false;
volatile bool data_ready = false;
volatile bool timer_started = false;
volatile uint32_t timerStop;

void wiegand_init(void)
{
    retarget_init(); // retarget printf to UART pins 9(tx) and 11(rx)
    printf("Initializing wiegand shit...");


    // Set up GPIO and pin interrupts
    nrf_gpio_cfg_sense_input(DATA0_IN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(DATA1_IN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);

    // Set the GPIOTE PORT event as interrupt source, and enable interrupts for GPIOTE
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_EnableIRQ(GPIOTE_IRQn);

    // set up timer 2
    // adapted from https://github.com/NordicSemiconductor/nrf51-TIMER-examples/blob/master/timer_example_timer_mode/main.c
    // and https://devzone.nordicsemi.com/question/6278/setting-timer2-interval/
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Timer mode
    NRF_TIMER2->TASKS_CLEAR = 1;               // clear the task first to be usable for later
    NRF_TIMER2->PRESCALER = 4;                 //Set prescaler. Higher number gives slower timer. Prescaler = 0 gives 16MHz timer
    NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;      //Set counter to 16 bit resolution
    NRF_TIMER2->CC[0] = TIMER_DELAY;                        //Set value for TIMER2 compare register 0
    // Enable interrupt on Timer 2 for CC[0]
    NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    NVIC_EnableIRQ(TIMER2_IRQn);

}

/*
  static uint8_t pin_read(uint8_t pin_number)
  {
  // Borrowed from:
  // http://developer.mbed.org/teams/Nordic-Semiconductor/code/nRF51822/docs/3794dc9540f0/nrf__gpio_8h_source.html
  return ((NRF_GPIO->IN >> pin_number) & 1UL);
  }
*/

void wiegand_task(void)
{
    printf("timer %d, datardy %d, bitcount %d\r\n", timer_started, data_ready, bit_count);

    if (data_incoming && !timer_started) {
        NRF_TIMER2->TASKS_START = 1;    // Start TIMER2
        timer_started = true;
    }

    if (data_ready) {
        NRF_TIMER2->TASKS_STOP = 1;     // Stop the clock
        if (bit_count > 1)   // avoid garbage data at startup.
        {
            printf("Read %d bits: ", bit_count);
            for (uint8_t i=0; i<bit_count; i++)
            {
                printf("%d", data_bits[i]);
            }

        }

        data_incoming = false;
        timer_started = false;
        bit_count = 0;
        data_ready = false;

        // clears the old data
        for (uint8_t i=0; i<MAX_BITS; i++)
        {
            data_bits[i] = 0;
        }
    }
}

void TIMER2_IRQHandler(void)
{
    if (NRF_TIMER2->EVENTS_COMPARE[0])
    {
        data_ready = true;
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;           // Clear compare register 0 event
        NRF_TIMER2->TASKS_CAPTURE[1] = 1;
        NRF_TIMER2->CC[0] = (NRF_TIMER2->CC[1] + TIMER_DELAY);
    }
}

void GPIOTE_IRQHandler(void) {
    // This handler will be run after wakeup from system ON (GPIO wakeup)
    if(NRF_GPIOTE->EVENTS_PORT)
    {
        volatile uint32_t port_status = NRF_GPIO->IN;
        NRF_GPIOTE->EVENTS_PORT = 0;    // Clear event
        // If DATA1 is low assign it, otherwise leave it at 0 and move on.
        if (!(port_status >> DATA1_IN & 1UL)) {
            data_bits[bit_count] = 1;
            printf("1");
        } else {
            data_bits[bit_count] = 0;
            printf("0");
        }
        ata_incoming = true;
        NRF_TIMER2->TASKS_CAPTURE[1] = 1;   // trigger CAPTURE task
        NRF_TIMER2->CC[0] = (NRF_TIMER2->CC[1] + TIMER_DELAY); // Reset timer
        bit_count++;
    }
}
