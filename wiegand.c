#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
#define DATA1_IN 7
#define DATA0_CTL 2
#define DATA1_CTL 3

// macro to grab bit n from a unit64_t
#define GETBIT(x,n) ((x >> n)&1ULL)

#define TIMER_DELAY 3000 // Timer is set at 1Mhz, 3000 ticks = 3ms
#define MAX_LEN 44

uint64_t last_card = 0xDEADBEEF;        // unpadded last card for ease of re-transmission
uint8_t last_size = 32;                 // number of bits in last card
uint64_t proxmark_fmt = 0;				// proxmark formatted card
uint32_t num_reads = 0;					// number of cards read by BLEKey
volatile uint64_t card_data = 0;        // incoming wiegand data stored here
volatile uint8_t bit_count = 0;         // number of bits in the incoming card
volatile bool data_incoming = false;    // true when data starts coming in
volatile bool data_ready = false;       // set when timer interrupt fires
volatile bool timer_started = false;    // if recv wiegand
volatile bool card_fubar = false;       // set if BLE screws up an incoming card
volatile bool start_tx = false;         // triggers sending of wiegand data

static Wiegand_ctx *p_ctx;				// Struct to store card data

// static value that needs to be prepended to HID Prox cards
static const uint16_t padding[19] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0x3, 0x5, 0x9, 0x11, 0x21,
    0x41, 0x81, 0x101, 0x201, 0x401, 0x801
};

void wiegand_init(Wiegand_ctx *ctx)
{
	p_ctx = ctx;

	retarget_init(); // retarget printf to UART pins 9(tx) and 11(rx)
    printf("Initializing wiegand shit...");

    // Set the Wiegand control lines as outputs and pull them low
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

void add_card(uint64_t *data, uint8_t len)
{
	// add card to store
    p_ctx->card_store[num_reads].bit_len = len;
	// zero out old data to avoid garbage data from longer cards
	memset(p_ctx->card_store[num_reads].data, 0, CARD_DATA_LEN);
	memcpy(p_ctx->card_store[num_reads].data, data, CARD_DATA_LEN);
}


/*
 * Utility function for starting Wiegand transmission
 */

void send_wiegand(void)
{
    start_tx = true;
}


/*
 * Sends data out on the Wiegand lines
 */

void tx_wiegand(uint64_t data, uint8_t size)
{
    for (uint8_t i = last_size; i-- > 0;)
        {
            // wiegand pulses should be ~40us, there should be ~2.025ms
            // between each pulse...correcting for fubar nrf delay here.
            uint8_t bit = GETBIT(last_card, i);
            bit ? nrf_gpio_pin_set(DATA1_CTL) : nrf_gpio_pin_set(DATA0_CTL);
            nrf_delay_us(26);
            bit ? nrf_gpio_pin_clear(DATA1_CTL) : nrf_gpio_pin_clear(DATA0_CTL);
            nrf_delay_us(1380);
        }
}

/*
 * Adds the padding (or preamble) bits to card data so it's ready
 * to be copied with a Proxmark or similar.
 */
uint64_t pad_card(uint64_t data, uint8_t size)
{
	uint8_t pad_len = (MAX_LEN - size);
    uint64_t card_val = padding[pad_len];
	card_val <<= size;
    card_val |= data;
    return card_val;
}

void wiegand_task(void)
{
    if (start_tx && !timer_started) {
        uint32_t ret;
        uint8_t foo;
        // this turns off application interrupts (not softdevice)
        ret = sd_nvic_critical_region_enter(&foo);
        if (ret == NRF_SUCCESS)
        {
            // send the data on the Wiegands
            tx_wiegand(last_card, last_size);
            printf("Tx %d bits: 0x%llx, Wiegandses pwned!\r\n", last_size, pad_card(last_card, last_size));
        }
        sd_nvic_critical_region_exit(foo);
        start_tx = false;
    }

    if (data_incoming && !timer_started) {
        NRF_TIMER2->TASKS_START = 1;    // Start TIMER2
        timer_started = true;
    }

    if (data_ready) {
        if (bit_count > 1 && !card_fubar)   // avoid garbage data at startup.
        {
            // store the card's information for replay later
            last_card = card_data;
            last_size = bit_count;
            // print debug information to the serial terminal
            printf("%ld. Rx %d bits: ", num_reads, bit_count);
            for (uint8_t i = last_size; i-- > 0;)
            {
                printf("%lld", GETBIT(last_card, i));
            }
            proxmark_fmt = pad_card(card_data, bit_count);
            printf( " Raw: 0x%llx Padded: 0x%llx\r\n", card_data, proxmark_fmt);
            // add card to struct for BLE transmission
            add_card(&proxmark_fmt, bit_count);
        	num_reads++;
		}

        //reset vars for next read
        data_incoming = false;
        timer_started = false;
        data_ready = false;
        card_fubar = false;
        bit_count = 0;
        card_data = 0;
		proxmark_fmt = 0;
    }
}

void TIMER2_IRQHandler(void)
{
    if (NRF_TIMER2->EVENTS_COMPARE[0])
    {
        data_ready = true;
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;           // Clear compare register 0 event
        NRF_TIMER2->TASKS_STOP = 1;     // Stop the timer
        // this code here is probably screwed up, it will get fixed when
		// I convert the tx_wiegand to timer based code to make it more accurate. 
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
        card_data |= 1;
    } else if (!(port_status >> DATA0_IN & 1UL)) {
		// 0 is already assigned, don't need to do anything
    } else {
        // port status lost thanks to BLE delaying read.
        card_fubar = true;
    }
    data_incoming = true;
    NRF_TIMER2->TASKS_CAPTURE[1] = 1;   // trigger CAPTURE task
    NRF_TIMER2->CC[0] = (NRF_TIMER2->CC[1] + TIMER_DELAY); // Reset timer
    bit_count++;
}
