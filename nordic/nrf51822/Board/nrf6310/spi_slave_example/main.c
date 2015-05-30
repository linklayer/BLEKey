/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
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

/**@file
 *
 * @defgroup spi_slave_example_main SPI slave example application.
 * @{
 * @ingroup  spi_slave_example
 *
 * @brief    SPI slave example application main file.
 */
 
#include "spi_slave_example.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdint.h>

 
/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // Set LED2 high to indicate that error handler has been called.
    nrf_gpio_pin_set(LED_2);
    
    for (;;)
    {
        // No implementation needed.
    }
}


/**@brief Function for application main entry. Does not return.
 */ 
int main(void)
{
    // Configure all LED as outputs. 
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
    
    const uint32_t err_code = spi_slave_example_init();
    APP_ERROR_CHECK(err_code);    
    
    // Set LED0 high to indicate that the application is running. 
    nrf_gpio_pin_set(LED_0);
    
    // Enter application main processing loop.
    for (;;)
    {
        // No implementation needed.
    }   
}

/** @} */
