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
 * @defgroup nRF6350_radio_configuration_example_main main.c
 * @{
 * @ingroup nRF6350_radio_configuration_example
 * @brief nRF6350 Radio Configuration Example Application main file.
 * 
 * This file contains the source code for a sample application using nRF6350.
 *
 */
#include <stdio.h>
#include "nrf.h"
#include "nrf6350.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"

static uint8_t packet[256];                                      /**< Data packet. */
static bool    sweep              = false;                       /**< Sweep enabled. */
static uint8_t mode_              = RADIO_MODE_MODE_Nrf_2Mbit;   /**< Transfer mode. */
static uint8_t data_rate_         = 0;                           /**< Transfer data rate. */
static uint8_t txpower_menu_      = RADIO_TXPOWER_TXPOWER_0dBm;  /**< TX power in dBm. */
static uint8_t channel_           = 0;                           /**< This is used in sweep and varies from channel_start_ to channel_end_. */
static uint8_t channel_start_     = 0;                           /**< Starting channel number. */
static uint8_t channel_end_       = 80;                          /**< End channel number. */
static uint8_t delayms_           = 10;                          /**< Delay in ms (0ms - 99ms). */
static uint8_t js_state_last      = JS_BUTTON_NONE;              /**< Previous Joystick state. */
static uint8_t js_state           = JS_BUTTON_NONE;              /**< Current Joystick state. */
static bool    sweep_tx_          = false;                       /**< Boolean for TX Sweep. */
static bool    in_menu            = true;                                 /**< Browsing menu if true and false while editing some value (defaults to true at power up). */
static uint8_t current_menu_pos   = 0;                               /**< Current menu option selected (defaults to first one in the list). */
static uint8_t current_scroll_pos = 0;                                     /**< Current scroll position in the selection text (defaults to first character). */


#define MAX_MENU_OPTIONS             (11UL)                             /**< Number of options in the main menu of the test */
#define MAX_CHARACTERS_PER_LINE      (16UL)                             /**< Maximum characters that are visible in one line on nRF6350 display */
#define MAX_CHARECTER_IN_MENU_OPTION (60UL)                             /**< Maximum string length of display options text */
#define MAX_CURSOR_POS_IN_STRING     (MAX_CHARECTER_IN_MENU_OPTION - \
                                     MAX_CHARACTERS_PER_LINE)           /**< Maximum cursor position(string position that points to first letter in first line of display) on display */
#define ERROR_PIN                    (LED_0)                            /**< Pin that is active high when there is an error in this example */

/*
 * Output power is
      0-->  +4 dBm
      1-->  0 dBm
      2-->  -4 dBm
      3-->  -8 dBm
      4-->  -12 dBm
      5-->  -16 dBm
      6-->  -20 dBm
      7-->  -40 dBm
 */
static uint8_t txpower_ = RADIO_TXPOWER_TXPOWER_0dBm;

/* Do not change the order, it depends on menu order. */
static uint32_t *p_variables[MAX_MENU_OPTIONS] =
{
    (uint32_t *)&channel_start_,
    (uint32_t *)&channel_end_,
    (uint32_t *)0,
    (uint32_t *)&delayms_,
    (uint32_t *)0,
    (uint32_t *)&data_rate_,
    (uint32_t *)0,
    (uint32_t *)&txpower_menu_,
    (uint32_t *)0,
    (uint32_t *)0,
    (uint32_t *)0,
};

/* Text for menu options, fixed length strings. */
static const char *help_menu[MAX_MENU_OPTIONS] =
{
    "Enter start channel for Sweep/Channel for constant carrier",
    "Enter end channel for Sweep                               ",
    "Start TX carrier                                          ",
    "Enter delay on each channel (1ms-99ms)                    ",
    "Cancel Sweep/Carrier                                      ",
    "Enter data rate('0'=250 Kb/s, '1'=1 Mb/s and '2'=2 Mb/s)  ",
    "Start modulated TX carrier                                ",
    "Enter output Power('0'=+4 dBm, '1'=0 dBm,...,'7'=-40 dBm):",
    "Start RX sweep                                            ",
    "Start TX sweep                                            ",
    "Start RX carrier                                          "
};

/* Types of radio tests. */
typedef enum
{
    RADIO_TEST_NOP,      /**< No test running      */
    RADIO_TEST_TXCC,     /**< TX constant carrier  */
    RADIO_TEST_TXMC,     /**< TX modulated carrier */
    RADIO_TEST_TXSWEEP,  /**< TX sweep             */
    RADIO_TEST_RXC,      /**< RX constant carrier  */
    RADIO_TEST_RXSWEEP,  /**< RX sweep             */
} radio_tests_t;


/* Forward declaration. */
static void radio_test(void);


/** @brief Function for configuring ERROR_PIN as output for showing errors.
 */
static void gpio_config(void)
{
    // Configure ERROR_PIN as output to show errors.
    nrf_gpio_cfg_output(ERROR_PIN);
    nrf_gpio_pin_clear(ERROR_PIN);
}


/**
 * @brief Function for starting up the needed peripherals.
 */
static void radio_init(void)
{
    /*  Start the RNG. */
    NRF_RNG->TASKS_START = 1;
  
    /* Start 16 MHz crystal oscillator. */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up. */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        // Do nothing.
    }  

    // Enable timer interrupt.
    NVIC_EnableIRQ(TIMER0_IRQn);
    __enable_irq();
}


/** @brief Function for setting ERROR_PIN to one and enter an infinite loop. This function is called if any of the
 *  nRF6350 functions fail.
 */
static void show_error(void)
{
    nrf_gpio_pin_set(ERROR_PIN);
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for waiting until the joystick status has changed.
*   Performs radio_test while joystick state has not changed.
*/
static void wait_for_joystick_movement(void)
{
    while (js_state == js_state_last)
    {
        radio_test();

        // Get the current status of the joystick
        if (!nrf6350_js_get_status(&js_state))
        {
            show_error();
        }
    }
    // update the last known js_state
    js_state_last = js_state;
}


typedef enum
{
    decrement=0,
    increment
}change_direction_t;


/** @brief Function for identifying the change in menu position. */
static void change_selected_variable(change_direction_t cd)
{
    switch (current_menu_pos)
    {
        case 0:
            if ((cd == decrement) && (channel_start_ > 0))  
            { 
                --channel_start_; 
            }
            else if ((cd == increment) && (channel_start_ < 80))  
            { 
                ++channel_start_; 
            }
            break;
        
        case 1:
            if ((cd == decrement) && (channel_end_ > 0))  
            { 
                --channel_end_; 
            }
            else if ((cd == increment) && (channel_end_ < 80))  
            { 
                ++channel_end_; 
            }
            break;
        
        case 3:
            if ((cd == decrement) && (delayms_ > 1))  
            { 
                --delayms_; 
            }
            else if ((cd == increment) && (delayms_ < 99))  
            { 
                ++delayms_; 
            }
            break;
        
        case 5:
            if ((cd == decrement) && (data_rate_ > 0))  
            { 
                --data_rate_; 
            }
            else if ((cd == increment) && (data_rate_ < 2))  
            { 
                ++data_rate_;
            }
            break;
        
        case 7:
            if ((cd == decrement) && (txpower_menu_ > 0))  
            { 
                --txpower_menu_; 
            }
            else if ((cd == increment) && (txpower_menu_ < 7))  
            { 
                ++txpower_menu_; 
            }
            break;
        
        case 2: 
            // Fall through.
        case 4:
            // Fall through.
        case 6:
            // Fall through.
        default:
            // No implementation needed.
            break;
    }
}


/** @brief Function that waits for joystick movement, then updates cursor and menu list position.
*/
static void joystick_wait_change_update_pos_variables(void)
{
    wait_for_joystick_movement();
    switch (js_state)
    {
        case JS_BUTTON_LEFT:
            // Browsing of options can be done while in the menu or in the menu selection browse mode.
            if ((in_menu) && (current_scroll_pos > 0))
            {
                --current_scroll_pos;
            }
            else
            {
                change_selected_variable(decrement);
            }
            break;
        
        case JS_BUTTON_RIGHT:
            // browsing of options can be done while in the menu or in the menu selection browse mode.
            if ((in_menu) && (current_scroll_pos < (MAX_CURSOR_POS_IN_STRING - 1)))
            {
                ++current_scroll_pos;
            }
            else
            {
                change_selected_variable(increment);
            }
            break;
        
        case JS_BUTTON_PUSH:
            // change modes.
            in_menu            = !(in_menu);
            current_scroll_pos = 0; // reset the scroll position.
            break;
        
        case JS_BUTTON_DOWN:
            // browse down while in the menu list, does not effect other modes.
            if(in_menu && current_menu_pos < (MAX_MENU_OPTIONS - 1))
            {
                ++current_menu_pos;
                current_scroll_pos = 0; // reset the scroll position
            }
            break;
            
        case JS_BUTTON_UP:
            // browse up while in menu list, does not effect other modes.
            if(in_menu && current_menu_pos > 0)
            {
                --current_menu_pos;
                current_scroll_pos = 0; // reset the scroll position.
            }
            break;
            
        case JS_BUTTON_NONE:
            // fall through
        default:
            // No implementation needed.
            break;
    }
}


/**
 * @brief Function for displaying text of the correct section with correct offset on the display.
 */
static void display_text(void)
{
    if (!nrf6350_lcd_clear())
    {
        show_error();
    }

    if (in_menu)
    {
        if(!nrf6350_lcd_write_string(&help_menu[current_menu_pos][current_scroll_pos],  \
                                     MAX_CHARACTERS_PER_LINE, LCD_UPPER_LINE, 0))
        {
            show_error();
        }
    }
    else
    {
        if (p_variables[current_menu_pos] != 0)
        {
            char buffer[3];
            sprintf(buffer, "%lu", (int32_t)(*(uint8_t *)p_variables[current_menu_pos]));

            if (!nrf6350_lcd_write_string(buffer, 3, LCD_UPPER_LINE, 7))
            {
                show_error();
            }
        }
    }
}


/**
 * @brief Function for the output menu to the display port.
 * @details The display can only show one line at a time, scroll right or left to see more text.
 * Also displays value of the variables to be changed in editing mode.
 */
static void menu_help_testrun_when_idle(void)
{
    while (true)
    {
        display_text();
        joystick_wait_change_update_pos_variables();
    }
}


/**
 * @brief Function for initializing timer0 in 24 bit timer mode and 1 us resolution.
 */
static void timer0_init(uint8_t delayms)
{
    NRF_TIMER0->TASKS_STOP  = 1;
    // Create an Event-Task shortcut to clear TIMER1 on COMPARE[0] event
    NRF_TIMER0->SHORTS      = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER0->MODE        = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER0->BITMODE     = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
    NRF_TIMER0->PRESCALER   = 4;  // 1us resolution
    NRF_TIMER0->INTENSET    = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos);

    NRF_TIMER0->CC[0]       = (uint32_t)delayms * 1000;
    NRF_TIMER0->TASKS_START = 1;
}


/** @brief Function for disabling the radio.
*/
static void radio_disable(void)
{
    NRF_RADIO->SHORTS          = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TEST            = 0;
    NRF_RADIO->TASKS_DISABLE   = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0)
    {
        // Do nothing.
    }
    NRF_RADIO->EVENTS_DISABLED = 0;
}


/**
 * @brief Function for stopping Timer0 and disabling the radio to end radio sweep.
 */
static void radio_sweep_end(void)
{
    NRF_TIMER0->TASKS_STOP = 1;
    radio_disable();
}


/**
 * @brief Function for turning on TX carrier test mode.
 */
static void radio_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel)
{
    radio_disable();
    NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk << RADIO_SHORTS_READY_START_Pos;
    NRF_RADIO->TXPOWER    = ((uint32_t)txpower << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->MODE       = ((uint32_t)mode << RADIO_MODE_MODE_Pos);
    NRF_RADIO->FREQUENCY  = channel;
    NRF_RADIO->TEST       = (RADIO_TEST_CONST_CARRIER_Enabled << RADIO_TEST_CONST_CARRIER_Pos)
                            | (RADIO_TEST_PLL_LOCK_Enabled << RADIO_TEST_PLL_LOCK_Pos);
    NRF_RADIO->TASKS_TXEN = 1;
}


/**
 * @brief Function for generating an 8 bit random number using the internal random generator.
 */
static uint32_t rnd8(void)
{
    NRF_RNG->EVENTS_VALRDY = 0;
    while (NRF_RNG->EVENTS_VALRDY == 0)
    {
        // Do nothing.
    }
    return  NRF_RNG->VALUE;
}


/**
 * @brief Function for generating a 32 bit random number using the internal random generator.
 */
static uint32_t rnd32(void)
{
    uint8_t i;
    uint32_t val = 0;

    for (i = 0; i < 4; i++)
    {
        val <<= 8;
        val  |= rnd8();
    }
    return val;
}


/**
 * @brief Function for configuring the radio to use a random address and a 254 bytes random payload.
 * The S0 and S1 fields are not used.
 */
static void generate_modulated_rf_packet(void)
{
    uint8_t i;

    NRF_RADIO->PREFIX0 = rnd8();
    NRF_RADIO->BASE0   = rnd32();

    // Packet configuration.
    // S1 size = 0 bits, S0 size = 0 bytes, payload length size = 8 bits
    NRF_RADIO->PCNF0 = (0UL << RADIO_PCNF0_S1LEN_Pos)
                       | (0UL << RADIO_PCNF0_S0LEN_Pos)
                       | (8UL << RADIO_PCNF0_LFLEN_Pos);
    // Packet configuration.
    // Bit 25: 1 Whitening enabled,
    // Bit 24: 1 Big endian,
    // 4 byte base address length (5 byte full address length),
    // 0 byte static length, max 255 byte payload.
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos)
                       | (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos)
                       | (4UL << RADIO_PCNF1_BALEN_Pos)
                       | (0UL << RADIO_PCNF1_STATLEN_Pos)
                       | (255UL << RADIO_PCNF1_MAXLEN_Pos);
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Disabled << RADIO_CRCCNF_LEN_Pos);
    packet[0] = 254;  // 254 bytes payload.
    // Fill payload with random data:
    for ( i = 0 ; i < 254 ; i++)
    {
        packet[i + 1] = rnd8();
    }
    NRF_RADIO->PACKETPTR = (uint32_t)packet;
}


/**
 * @brief Function for starting modulated TX carrier. This is done by repeatedly sending a packet with a random address and
 *  payload.
 */
static void radio_modulated_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel)
{
    radio_disable();
    generate_modulated_rf_packet();
    NRF_RADIO->SHORTS     = RADIO_SHORTS_END_DISABLE_Msk 
                            | RADIO_SHORTS_READY_START_Msk 
                            | RADIO_SHORTS_DISABLED_TXEN_Msk;
    NRF_RADIO->TXPOWER    = (txpower << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->MODE       = (mode << RADIO_MODE_MODE_Pos);
    NRF_RADIO->FREQUENCY  = channel;
    NRF_RADIO->TASKS_TXEN = 1;
}


/**
 * @brief Function for turning on the RX carrier.
 */
static void radio_rx_carrier(uint8_t channel)
{
    radio_disable();
    NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk;
    NRF_RADIO->FREQUENCY  = channel;
    NRF_RADIO->TASKS_RXEN = 1;
}


/**
 * @brief Function for turning on a TX carrier sweep. This test uses Timer 0 to restart the TX carrier above at different channels.
 */
static void radio_tx_sweep_start(uint8_t channel_start, uint8_t delayms)
{
    channel_  = channel_start;
    sweep_tx_ = true;
    timer0_init(delayms);
}


/**
 * @brief Function for turning on an RX carrier sweep. This test uses Timer 0 to restart the RX carrier above at different channels.
 */
static void radio_rx_sweep_start(uint8_t channel_start, uint8_t delayms)
{
    channel_  = channel_start;
    sweep_tx_ = false;
    timer0_init(delayms);
}


/**
 * @brief Function for handling the Timer 0 interrupt used for TX/RX sweep. The carrier is started with the new channel,
 * and the channel is incremented for next interrupt.
 */
void TIMER0_IRQHandler(void)
{
    // Check if Timer 0 interrupts are enabled and if this interrupt is generated by Timer 0.
    if ( (NRF_TIMER0->EVENTS_COMPARE[0] != 0) &&
         (NRF_TIMER0->INTENSET | (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos)))
    {
        if (sweep_tx_)
        {
            radio_tx_carrier(txpower_, mode_, channel_);
        }
        else
        {
            radio_rx_carrier(channel_);
        }
        channel_++;
        if (channel_ > channel_end_)
        {
            channel_ = channel_start_;
        }
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    }
}


static void radio_test(void)
{
    radio_tests_t test_radio = RADIO_TEST_NOP;   /*!< Continuous radio tests to run. */
    radio_tests_t cur_test   = RADIO_TEST_NOP;   /*!< Current selection of test. */

    // Do not change test while browsing the menu.
    if (!in_menu)
    {
        switch (current_menu_pos)
        {
            case 0:  // Start Channel number.
            
            case 1:  // End Channel number.

            case 3:  // Delay time in ms.
                test_radio = cur_test;
                break;

            case 5:  // Data rate
                if (data_rate_ == 0)
                {
                    mode_ = RADIO_MODE_MODE_Nrf_250Kbit;
                }
                else if (data_rate_ == 1)
                {
                    mode_ = RADIO_MODE_MODE_Nrf_1Mbit;
                }
                else
                {
                    mode_ = RADIO_MODE_MODE_Nrf_2Mbit;
                }
                test_radio = cur_test;
            break;

            case 7:  // Power
                switch(txpower_menu_)
                {
                    case 0:
                        txpower_ =  RADIO_TXPOWER_TXPOWER_Pos4dBm;
                        break;
                
                    case 1:
                        txpower_ =  RADIO_TXPOWER_TXPOWER_0dBm;
                        break;
                
                    case 2:
                        txpower_ = RADIO_TXPOWER_TXPOWER_Neg4dBm;
                        break;
                
                    case 3:
                        txpower_ = RADIO_TXPOWER_TXPOWER_Neg8dBm;
                        break;
                
                    case 4:
                        txpower_ = RADIO_TXPOWER_TXPOWER_Neg12dBm;
                        break;
                
                    case 5:
                        txpower_ = RADIO_TXPOWER_TXPOWER_Neg16dBm;
                        break;
                
                    case 6:
                        txpower_ = RADIO_TXPOWER_TXPOWER_Neg20dBm;
                        break;
                
                    case 7:
                        // Fall through.
                    default:
                        txpower_ = RADIO_TXPOWER_TXPOWER_Neg30dBm;
                        break;
                }
            test_radio = cur_test;
            break;

            // START TX Carrier.
            case 2:
                test_radio = RADIO_TEST_TXCC;
                break;

            // Cancel Sweep/Carrier.
            case 4:
                radio_sweep_end();
                cur_test = RADIO_TEST_NOP;
                break;

            // Start modulated TX carrier.
            case 6:
                test_radio = RADIO_TEST_TXMC;
                break;

            // Start RX Sweep.
            case 8:
                test_radio = RADIO_TEST_RXSWEEP;
                break;

            // Start RX Sweep.
            case 9:
                test_radio = RADIO_TEST_TXSWEEP;
                break;

            // Start RX carrier.
            case 10:
                test_radio = RADIO_TEST_RXC;
                break;

            default:
                // No implementation needed.
                break;
        }
    }

    switch(test_radio)
    {
        case RADIO_TEST_TXCC:
            if (sweep)
            {
                radio_sweep_end();
                sweep = false;
            }
            radio_tx_carrier(txpower_, mode_, channel_start_);
            cur_test   = test_radio;
            test_radio = RADIO_TEST_NOP;
        break;

        case RADIO_TEST_TXMC:
            if (sweep)
            {
                radio_sweep_end();
                sweep = false;
            }
            radio_modulated_tx_carrier(txpower_, mode_, channel_start_);
            cur_test = test_radio;
            test_radio = RADIO_TEST_NOP;
        break;

        case RADIO_TEST_TXSWEEP:
            radio_tx_sweep_start(channel_start_, delayms_);
            sweep    = true;
            cur_test = test_radio;
            test_radio = RADIO_TEST_NOP;
            break;

        case RADIO_TEST_RXC:
            if (sweep)
            {
                radio_sweep_end();
                sweep = false;
            }
            radio_rx_carrier(channel_start_);
            cur_test   = test_radio;
            test_radio = RADIO_TEST_NOP;
            break;

        case RADIO_TEST_RXSWEEP:
            radio_rx_sweep_start(channel_start_, delayms_);
            sweep    = true;
            cur_test = test_radio;
            test_radio = RADIO_TEST_NOP;
            break;

        case RADIO_TEST_NOP:
            // Fall through.
        default:
            // No implementation needed.
            break;
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    gpio_config();

    //Initialize the LCD display.
    if (!nrf6350_lcd_init())
    {
        show_error();
    }

    // Clear the display and print a welcome message.
    if (!nrf6350_lcd_write_string("    RF TEST     ", MAX_CHARACTERS_PER_LINE, LCD_UPPER_LINE, 0))
    {
        show_error();
    }
  
    if (!nrf6350_lcd_write_string("   SCROLL DOWN  ", MAX_CHARACTERS_PER_LINE, LCD_LOWER_LINE, 0))
    {
        show_error();
    }

    // Inititialize peripherals needed by radio.
    radio_init();

    wait_for_joystick_movement();     // Test title is visible on display until joystick status is changed.

    menu_help_testrun_when_idle();  // This function will never return.
}

/**
 *@}
 **/
