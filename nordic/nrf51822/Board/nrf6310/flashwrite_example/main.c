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
 * @defgroup flashwrite_example_main main.c
 * @{
 * @ingroup flashwrite_example
 *
 * @brief This file contains the source code for a sample application using the Flash Write Application.
 *
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */

#include <stdbool.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "boards.h"

/** @brief Function for configuring pins 0-7 as inputs and 8-15 as outputs.
 *
 */
static void init(void)
{
    // Port0 (0-7 as inputs)
    nrf_gpio_range_cfg_input(BUTTON_START, BUTTON_STOP, BUTTON_PULL);

    // Port1 (8-15 as outputs)
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
}

/** @brief Function for erasing a page in flash.
 *
 * @param page_address Address of the first word in the page to be erased.
 */
static void flash_page_erase(uint32_t *page_address)
{
  // Turn on flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
    
    // Erase page:
    NRF_NVMC->ERASEPAGE = (uint32_t)page_address;
    
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
    
    // Turn off flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
    
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}


/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
static void flash_word_write(uint32_t *address, uint32_t value)
{
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
    
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
  
    *address = value;
  
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
  
    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
  
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t *addr;
    uint8_t   patwr;
    uint8_t   patrd;
    uint8_t   patold;
    uint32_t  i;
    uint32_t  pg_size;
    uint32_t  pg_num;

    init();
    
    patold  = 0;
    pg_size = NRF_FICR->CODEPAGESIZE;
    pg_num  = NRF_FICR->CODESIZE - 1;  // Use last page in flash

    while (true)
    {
        // Start address:
        addr = (uint32_t *)(pg_size * pg_num);
        // Erase page:
        flash_page_erase(addr);
        i = 0;
        do
        {
            // Read pattern from port 0 (pins0-7), and write it to flash:
            patwr =  nrf_gpio_port_read(NRF_GPIO_PORT_SELECT_PORT0);
            if (patold != patwr)
            {
                patold = patwr;
                flash_word_write(++addr, (uint32_t)patwr);
                i += 4;
            }
            // Read pattern from flash and write it to port 1 (pins8-15):
            patrd = (uint8_t)*addr;
            nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, patrd);
        } while (i < pg_size);
    }
}


/** @} */
