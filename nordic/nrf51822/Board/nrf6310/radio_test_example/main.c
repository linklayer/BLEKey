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
* @defgroup nrf_radio_test_example_main main.c
* @{
* @ingroup nrf_radio_test_example
* @brief Radio Test Example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO, and is controlled through the serial port.
*
* @image html example_board_setup_a.jpg "Use board setup A for this example."
*
*/


#include <stdint.h>
#include <stdbool.h>
#include "boards.h"
#include "nrf.h"
#include "radio_test.h"
#include "uart.h"
#include "nrf51_bitfields.h"

static uint8_t mode_          = RADIO_MODE_MODE_Nrf_2Mbit;
static uint8_t txpower_       = RADIO_TXPOWER_TXPOWER_0dBm;
static uint8_t channel_start_ = 0;
static uint8_t channel_end_   = 80;
static uint8_t delayms_       = 10;

static bool sweep = false;

typedef enum
{
    RADIO_TEST_NOP,      /**< No test running.      */
    RADIO_TEST_TXCC,     /**< TX constant carrier.  */
    RADIO_TEST_TXMC,     /**< TX modulated carrier. */
    RADIO_TEST_TXSWEEP,  /**< TX sweep.             */
    RADIO_TEST_RXC,      /**< RX constant carrier.  */
    RADIO_TEST_RXSWEEP,  /**< RX sweep.             */
} radio_tests_t;


#define BELL 7 // Bell
#define BS   8 // Backspace


/** @brief Function for configuring all peripherals used in this example.
*/
static void init(void)
{
    NRF_RNG->TASKS_START = 1;
    
    // Start 16 MHz crystal oscillator
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;

    // Wait for the external oscillator to start up
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }  
}


/** @brief Function for outputting usage info to the serial port.
*/
static void help(void)
{
    uart_putstring((const uint8_t *)"Usage:\r\n");
    uart_putstring((const uint8_t *)"a: Enter start channel for sweep/channel for constant carrier\r\n");
    uart_putstring((const uint8_t *)"b: Enter end channel for sweep\r\n");
    uart_putstring((const uint8_t *)"c: Start TX carrier\r\n");
    uart_putstring((const uint8_t *)"d: Enter time on each channel (1ms-99ms)\r\n");
    uart_putstring((const uint8_t *)"e: Cancel sweep/carrier\r\n");
    uart_putstring((const uint8_t *)"m: Enter data rate\r\n");
    uart_putstring((const uint8_t *)"o: Start modulated TX carrier\r\n");
    uart_putstring((const uint8_t *)"p: Enter output power\r\n");
    uart_putstring((const uint8_t *)"s: Print current delay, channels and so on\r\n");
    uart_putstring((const uint8_t *)"r: Start RX sweep\r\n");
    uart_putstring((const uint8_t *)"t: Start TX sweep\r\n");
    uart_putstring((const uint8_t *)"x: Start RX carrier\r\n");
}


/** @brief Function for reading two digit decimal numbers from the serial port. Backspace is supported on the first char.
*/
static uint8_t get_dec2(void)
{
    uint8_t buf[2];
    uint8_t i = 0;
    uint8_t c;
  
    buf[0] = buf[1] = 0;
    while (i < 2)
    {
        c = uart_get();
        if ((i > 0) && (c == BS))
        {
            uart_put(c);
            i--;
        }
        else if ((c >= '0') && (c <= '9'))
        {
            uart_put(c);
            buf[i] = c - '0';
            i++;
        }
        else
        {
            uart_put(BELL);
        }
    }
    uart_putstring((const uint8_t *)"\r\n");
    return buf[0] * 10 + buf[1];
}


/** @brief Function for reading the data rate.
*/
void get_datarate(void)
{
    uint8_t c;

    uart_putstring((const uint8_t *)"Enter data rate ('0'=250 Kbit/s, '1'=1 Mbit/s and '2'=2 Mbit/s):");
    while (true)
    {
        c = uart_get();
        if ((c >= '0') && (c <= '2'))
        {
            uart_put(c);
            break;
        }
        else
        {
            uart_put(BELL);
        }
    }
    if (c == '0')
    {
        mode_ = RADIO_MODE_MODE_Nrf_250Kbit;
    }
    else if (c == '1')
    {
        mode_ = RADIO_MODE_MODE_Nrf_1Mbit;
    }
    else
    {
        mode_ = RADIO_MODE_MODE_Nrf_2Mbit;
    }
    uart_putstring((const uint8_t *)"\r\n");
}


/** @brief Function for reading the output power.
*/
void get_power(void)
{
    uint8_t c;

    uart_putstring((const uint8_t *)"Enter output power ('0'=+4 dBm, '1'=0 dBm,...,'7'=-30 dBm):");
    while (true)
    {
        c = uart_get();
        if ((c >= '0') && (c <= '7'))
        {
            uart_put(c);
            break;
        }
        else
        {
            uart_put(BELL);
        }
    }
    
    switch(c)
    {
        case '0':
            txpower_ =  RADIO_TXPOWER_TXPOWER_Pos4dBm;
            break;
        
        case '1':
            txpower_ =  RADIO_TXPOWER_TXPOWER_0dBm;
            break;
        
        case '2':
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg4dBm;
            break;
        
        case '3':
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg8dBm;
            break;
        
        case '4':
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg12dBm;
            break;
        
        case '5':
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg16dBm;
            break;
        
        case '6':
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg20dBm;
            break;
        
        case '7':
            // fall through 
        
        default:
            txpower_ = RADIO_TXPOWER_TXPOWER_Neg30dBm;
            break;
    }
    uart_putstring((const uint8_t *)"\r\n");
}


/** @brief Function for printing two digit decimal numbers.
*/
static void print_dec2(uint8_t val)
{
    uart_put(val / 10 + '0');
    uart_put(val % 10 + '0');
}


/** @brief Function for printing parameters to the serial port.
*/
void print_parameters(void)
{
    uart_putstring((const uint8_t *)"Parameters:\r\n");
    switch(mode_)
    {
        case RADIO_MODE_MODE_Nrf_250Kbit:
            uart_putstring((const uint8_t *)"Data rate...........: 250 Kbit/s\r\n");
            break;
        
        case RADIO_MODE_MODE_Nrf_1Mbit:
            uart_putstring((const uint8_t *)"Data rate...........: 1 Mbit/s\r\n");
            break;
        
        case RADIO_MODE_MODE_Nrf_2Mbit:
            uart_putstring((const uint8_t *)"Data rate...........: 2 Mbit/s\r\n");
            break;
    }
    
    switch(txpower_)
    {
        case RADIO_TXPOWER_TXPOWER_Pos4dBm:
            uart_putstring((const uint8_t *)"TX Power............: +4 dBm\r\n");
            break;
        
        case RADIO_TXPOWER_TXPOWER_0dBm:
            uart_putstring((const uint8_t *)"TX Power............: 0 dBm\r\n");
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg4dBm:
            uart_putstring((const uint8_t *)"TX Power............: -4 dBm\r\n");
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg8dBm:
            uart_putstring((const uint8_t *)"TX Power............: -8 dBm\r\n");
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg12dBm:
            uart_putstring((const uint8_t *)"TX Power............: -12 dBm\r\n");
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg16dBm:
            uart_putstring((const uint8_t *)"TX Power............: -16 dBm\r\n");
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg20dBm:
            uart_putstring((const uint8_t *)"TX Power............: -20 dBm\r\n");
            break;
        
        case RADIO_TXPOWER_TXPOWER_Neg30dBm:
            uart_putstring((const uint8_t *)"TX Power............: -30 dBm\r\n");
            break;
        
        default:
            // No implementation needed.
            break;
        
    }
    uart_putstring((const uint8_t *)"(Start) Channel.....: ");
    print_dec2(channel_start_);
    uart_putstring((const uint8_t *)"\r\nEnd Channel.........: ");
    print_dec2(channel_end_);
    uart_putstring((const uint8_t *)"\r\nTime on each channel: ");
    print_dec2(delayms_);
    uart_putstring((const uint8_t *)" ms\r\n");
}


/** @brief Function for main application entry.
 */
int main(void)
{ 
    radio_tests_t test     = RADIO_TEST_NOP;
    radio_tests_t cur_test = RADIO_TEST_NOP;

    init();
    uart_config(TX_PIN_NUMBER, RX_PIN_NUMBER);
    uart_putstring((const uint8_t *)"RF Test\r\n");
    
    NVIC_EnableIRQ(TIMER0_IRQn);
    __enable_irq();
    
    while (true)
    {
        __WFI();
        switch (uart_get())
        {
            case 'a':
                while (true)
                {
                    uart_putstring((const uint8_t *)"Enter start channel \
                                   (two decimal digits, 00 to 80):");
                    channel_start_ = get_dec2();
                    if (channel_start_ <= 80)
                    break;
                    uart_putstring((const uint8_t *)"Channel must be between 0 and 80\r\n");
                }
                test = cur_test;
                break;

            case 'b':
                while (true)
                {
                    uart_putstring((const uint8_t *)"Enter end channel \
                                   (two decimal digits, 00 to 80):");
                    channel_end_ = get_dec2();
                    if (channel_end_ <= 80)
                    {
                        break;
                    }
                uart_putstring((const uint8_t *)"Channel must be between 0 and 80\r\n");
                }
                test = cur_test;
                break;

            case 'c':
                test = RADIO_TEST_TXCC;
                break;

            case 'd':
                while (true)
                {
                    uart_putstring((const uint8_t *)"Enter delay in ms \
                                   (two decimal digits, 01 to 99):");
                    delayms_ = get_dec2();
                    if ((delayms_ > 0) && (delayms_ < 100))   
                    {
                        break;
                    }
                    uart_putstring((const uint8_t *)"Delay must be between 1 and 99\r\n");
                }
                test = cur_test;
                break;

            case 'e':
                radio_sweep_end();
                cur_test = RADIO_TEST_NOP;
                break;

            case 'm':
                get_datarate();
                test = cur_test;
                break;

            case 'o':
                test = RADIO_TEST_TXMC;
                uart_putstring((const uint8_t *)"TX modulated carrier\r\n");
                break;

            case 'p':
                get_power();
                test = cur_test;
                break;

            case 'r':
                test = RADIO_TEST_RXSWEEP;
                uart_putstring((const uint8_t *)"RX Sweep\r\n");
                break;

            case 's':
                print_parameters();
                break;

            case 't':
                test = RADIO_TEST_TXSWEEP;
                uart_putstring((const uint8_t *)"TX Sweep\r\n");
                break;

            case 'x':
                test = RADIO_TEST_RXC;
                uart_putstring((const uint8_t *)"RX constant carrier\r\n");
                break;

            case 'h':
                // Fall through.
        
            default:
                help();
                break;
        }
    
        switch (test)
        {
            case RADIO_TEST_TXCC:
                if (sweep)
                {
                    radio_sweep_end();
                    sweep = false;
                }
                radio_tx_carrier(txpower_, mode_, channel_start_);
                cur_test = test;
                test     = RADIO_TEST_NOP;
                break;

            case RADIO_TEST_TXMC:
                if (sweep)
                {
                    radio_sweep_end();
                    sweep = false;
                }
                radio_modulated_tx_carrier(txpower_, mode_, channel_start_);
                cur_test = test;
                test     = RADIO_TEST_NOP;
                break;

            case RADIO_TEST_TXSWEEP:
                radio_tx_sweep_start(txpower_, mode_, channel_start_, channel_end_, delayms_);
                sweep    = true;
                cur_test = test;
                test     = RADIO_TEST_NOP;
                break;

            case RADIO_TEST_RXC:
                if (sweep)
                {
                    radio_sweep_end();
                    sweep = false;
                }
                radio_rx_carrier(mode_, channel_start_);
                cur_test = test;
                test     = RADIO_TEST_NOP;
                break;  

            case RADIO_TEST_RXSWEEP:
                radio_rx_sweep_start(mode_, channel_start_, channel_end_, delayms_);
                sweep    = true;
                cur_test = test;
                test     = RADIO_TEST_NOP;
                break;

            case RADIO_TEST_NOP:
                // Fall through.
            default:
                // No implementation needed.
                break;
        }
    }
}

/** @} */
