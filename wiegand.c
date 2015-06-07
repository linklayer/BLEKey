#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h" // added
//#include "nrf51.h"
//#include "nrf_gpiote.h" //added
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "retarget.h"
#include "wiegand.h"
#include "nrf_sdm.h" // added and now it builds!
#include "ble_wiegand.h"

// wiegand data pins
#define DATA0_IN 0
#define DATA1_IN 1
#define DATA0_CTL 2
#define DATA1_CTL 3

#define TIMER_DELAY 3000 // Timer is set at 1Mhz, 3000 ticks = 3ms
#define MAX_BITS 100
#define MAX_LEN 44

volatile uint64_t card_data = 0;
volatile uint8_t data_bits[MAX_BITS];
volatile uint8_t bit_count = 0;
volatile bool data_incoming = false;
volatile bool data_ready = false;
volatile bool timer_started = false;
volatile bool card_fubar = false;       // set if BLE screws up an incoming card
volatile uint32_t timerStop;

static struct wiegand_ctx *p_ctx;

static const uint16_t padding[19] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0x3, 0x5, 0x9, 0x11, 0x21,
    0x41, 0x81, 0x101, 0x201, 0x401, 0x801
};

void wiegand_init(struct wiegand_ctx *ctx)
{
    p_ctx = ctx;

    retarget_init(); // retarget printf to UART pins 9(tx) and 11(rx)
    printf("Initializing wiegand shit...");

    // Set the control lines as outputs and pull them low
    nrf_gpio_cfg_output(DATA0_CTL);
    nrf_gpio_cfg_output(DATA1_CTL);
    nrf_gpio_pin_clear(DATA0_CTL);
    nrf_gpio_pin_clear(DATA1_CTL);

    printf("Pin interrupts...");
    // Set up GPIO and pin interrupts
    nrf_gpio_cfg_sense_input(DATA0_IN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(DATA1_IN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
    // Set the GPIOTE PORT event as interrupt source, and enable interrupts for GPIOTE
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    //NVIC_EnableIRQ(GPIOTE_IRQn);
    sd_nvic_SetPriority(GPIOTE_IRQn, 1);
    sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);
    sd_nvic_EnableIRQ(GPIOTE_IRQn);

    printf("Timers...");
    // set up timer 2
    // adapted from https://github.com/NordicSemiconductor/nrf51-TIMER-examples/blob/master/timer_example_timer_mode/main.c
    // and https://devzone.nordicsemi.com/question/6278/setting-timer2-interval/
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Timer mode
    NRF_TIMER2->TASKS_CLEAR = 1;               // clear the task first to be usable for later
    NRF_TIMER2->PRESCALER = 4;       //Set prescaler. Higher number gives slower timer. Prescaler = 0 gives 16MHz timer
    NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;      //Set counter to 16 bit resolution
    NRF_TIMER2->CC[0] = TIMER_DELAY;                        //Set value for TIMER2 compare register 0
    // Enable interrupt on Timer 2 for CC[0]
    NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    //NVIC_EnableIRQ(TIMER2_IRQn);
    sd_nvic_ClearPendingIRQ(TIMER2_IRQn);
    sd_nvic_SetPriority(TIMER2_IRQn, 3);
    sd_nvic_EnableIRQ(TIMER2_IRQn);

    printf("done\r\n");
}

void add_card(uint64_t *data, uint8_t len) {
    // shift cards over
    memcpy(&(p_ctx->card_store[1]), p_ctx->card_store,
           sizeof(p_ctx->card_store) - sizeof(struct card));

    // add card to store
    p_ctx->card_store[0].bit_len = len;
    memcpy(p_ctx->card_store[0].data, data, (len/8+1));

}

void wiegand_task(void)
{
    if (data_incoming && !timer_started) {
        NRF_TIMER2->TASKS_START = 1;    // Start TIMER2
        timer_started = true;
    }

    if (data_ready) {
        if (bit_count > 1 && !card_fubar)   // avoid garbage data at startup.
        {
            uint8_t pad_len = (MAX_LEN - bit_count);
            printf("Read %d bits: ", bit_count);

            uint64_t card_val = padding[pad_len];
            card_val <<= bit_count;
            card_val |= card_data;

            for (uint8_t i=0; i<bit_count; i++)
            {
                printf("%d", data_bits[i]);
            }
            printf( " 0x%llx \r\n", card_val);
	    // add card to struct for BLE transmission
	    add_card(&card_val, bit_count);

        }

        data_incoming = false;
        timer_started = false;
        bit_count = 0;
        data_ready = false;
        card_data = 0;
        card_fubar = false;

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
        NRF_TIMER2->TASKS_STOP = 1;     // Stop the timer
        NRF_TIMER2->TASKS_CAPTURE[1] = 1;
        NRF_TIMER2->CC[0] = (NRF_TIMER2->CC[1] + TIMER_DELAY);

    }
}

void GPIOTE_IRQHandler(void)
{
    // This handler will be run after wakeup from system ON (GPIO wakeup)
    uint32_t port_status = NRF_GPIO->IN;
    NRF_GPIOTE->EVENTS_PORT = 0;    // Clear event
    card_data <<= 1;
    if (!(port_status >> DATA1_IN & 1UL)) {
        data_bits[bit_count] = 1;
        card_data |= 1;
    } else if (!(port_status >> DATA0_IN & 1UL)) {
        data_bits[bit_count] = 0;
    } else {
        // port status lost thanks to BLE delaying read.
        data_bits[bit_count] = 9;
        card_fubar = true;
    }
    data_incoming = true;
    NRF_TIMER2->TASKS_CAPTURE[1] = 1;   // trigger CAPTURE task
    NRF_TIMER2->CC[0] = (NRF_TIMER2->CC[1] + TIMER_DELAY); // Reset timer
    bit_count++;
}
