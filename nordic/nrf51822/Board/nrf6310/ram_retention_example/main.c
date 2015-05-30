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
 * @defgroup ram_retention_example_main main.c
 * @{
 * @ingroup ram_retention_example
 * @brief RAM Retention Example Application main file.
 *
 * This file contains the source code for a sample application using RAM retention.
 * 
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"


#define RAM_MEMORY_TEST_ADDRESS (0x20002000UL)  /**< Address in RAM where test word (RAM_MEMORY_TEST_WORD) is written before System OFF and checked after System RESET.*/
#define RAM_MEMORY_TEST_WORD    (0xFEEDBEEFUL)  /**< Test word that is written to RAM address RAM_MEMORY_TEST_ADDRESS. */
#define RESET_MEMORY_TEST_BYTE  (0x0DUL)        /**< Known sequence written to a special register to check if this wake up is from System OFF. */
#define MAX_TEST_ITERATIONS     (1)             /**< Maximum number of iterations this example will run. */
#define SUCCESS_OUTPUT_VALUE    (0xAB)          /**< If RAM retention is tested for MAX_TEST_ITERATIONS, this value will be given as output.*/
#define PIN_GPIO_WAKEUP         (BUTTON_STOP)   /**< GPIO pin configured to wake up system from System OFF on falling edge. It can be connected to a button which is high when not pressed. */

/** @brief Function for handling HardFaults. In case something went wrong
 * or System OFF did not work and reached the end of the program.
 */
void HardFault_Handler()
{
    nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT1, 0xFF);

    while (true)
    {
        nrf_gpio_pin_toggle(LED_0);
        nrf_delay_ms(100);
    }
}


/** @brief Function for configuring I/O pins 8 - 16 as output, and pin 7 as wake up source.
 */
static void gpio_config(void)
{
    // Set board specific range as output.
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    // This pin is used for waking up from System OFF and is active low, enabling sense capabilities.
    nrf_gpio_cfg_sense_input(PIN_GPIO_WAKEUP, BUTTON_PULL, NRF_GPIO_PIN_SENSE_LOW);
}


/** @brief Function for indicating failure by turning on all LEDs.
 */
static void display_failure(void)
{
    nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0XFF);

    // Loop forever.
    while (true)
    {
        //Do nothing.
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t * volatile p_ram_test = (uint32_t *)RAM_MEMORY_TEST_ADDRESS;
    uint32_t            loop_count = 0;
  
    // GPIO Configuration.
    gpio_config();

    // Workaround for PAN_028 rev1.1 anomaly 22 - System: Issues with disable System OFF mechanism
    nrf_delay_ms(1);

    // Check if the system woke up from System OFF mode by reading the NRF_POWER->GPREGRET register which has
    // retained the value written before going to System OFF. Below is the layout for usage for
    // NRF_POWER->GPREGRET register.
    //  BITS |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0
    //  --------------------------------------------------------------------
    //       |        SPECIAL_SEQUENCE       |          LOOP_COUNT
    //  --------------------------------------------------------------------
    ///

    if ((NRF_POWER->GPREGRET >> 4) == RESET_MEMORY_TEST_BYTE)
    {
        // Take the loop_count value.
        loop_count          = (uint8_t)(NRF_POWER->GPREGRET & 0xFUL);
        NRF_POWER->GPREGRET = 0;

        if (loop_count >= (uint8_t)MAX_TEST_ITERATIONS)
        {
            // clear GPREGRET register before exit.
            NRF_POWER->GPREGRET = 0;
            nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, SUCCESS_OUTPUT_VALUE);
            while (true)
            {
                // Do nothing.
            }
        }
        if (*p_ram_test != RAM_MEMORY_TEST_WORD)
        {
            display_failure();
        }
        else
        {
            // @note This loop is just to display that we have verified RAM retention after waking
            // up from System OFF, not needed in real application.
            for (uint32_t i = 8; i > 0; i--)
            {
                nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)(0X01 << i));
                nrf_delay_ms(50);
            }
        }
    
        *p_ram_test = 0;
    }

    // Write the known sequence + loop_count to the GPREGRET register.
    loop_count++;
    NRF_POWER->GPREGRET = ( (RESET_MEMORY_TEST_BYTE << 4) | loop_count);

    // Write the known value to the known address in RAM, enable RAM retention, set System OFF, and wait
    // for GPIO wakeup from external source.
    // The LED lights up to indicate that the system will go to System OFF with RAM retention enabled.
    // @note This loop is just to display that we are preparing for system OFF and is not needed in a real
    // application.
    for (uint32_t i = 0; i < 8; i++)
    {
        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)(0X01 << i));
        nrf_delay_ms(50);
    }

    // Switch on both RAM banks when in System OFF mode.
    NRF_POWER->RAMON |= (POWER_RAMON_OFFRAM0_RAM0On << POWER_RAMON_OFFRAM0_Pos) |
                        (POWER_RAMON_OFFRAM1_RAM1On << POWER_RAMON_OFFRAM1_Pos);

    // Write test word to RAM memory.
    *p_ram_test = RAM_MEMORY_TEST_WORD;
    
    // Enter System OFF and wait for wake up from GPIO detect signal.
    NRF_POWER->SYSTEMOFF = 0x1;
    
    // This code will only be reached if System OFF did not work and will trigger a hard-fault which will 
    // be handled in HardFault_Handler(). If wake the up condition is already active while System OFF is triggered,
    // then the system will go to System OFF and wake up immediately with a System RESET.
    display_failure();
}
/** @} */
