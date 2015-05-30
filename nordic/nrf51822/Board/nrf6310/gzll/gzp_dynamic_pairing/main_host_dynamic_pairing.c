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
* $LastChangedRevision: 17930 $
*/


 /** @file
 * @brief Gazell Pairing Host with Dynamic Pairing example 
 * @defgroup gzp_host_dynamic_pairing_example Gazell Pairing Host with Dynamic Pairing 
 * @{
 * @ingroup gzp_03_examples
 * 
 * @brief Gazell Link Layer Device using Gazell Pairing for adding 
 * dynamic pairing functionality. 
 :
 * This project requires a running counterpart project, which is either a:
 *
 * 1) nRF24Lxx Device running the gzll_device_w_dynamic_pairing example from the 
 * compatible version of the nRFgo SDK, or a
 *
 * 2) nRF51 Device running the gzp_device_dynamic_pairing_example example.
 * 
 * The application listens for packets continuously, monitoring for pairing 
 * requests as well as normal user data. 
 *
 * The Gazell pairing library uses pipe 0 and pipe 1 for encrypted communication.
 * The application will grant any request for a Host ID, thus granting pairing.
 * Unencrypted packets can be received on pipe 2.
 *
 * When DATA is received, the contents of the first payload byte of 
 * the are output on GPIO Port LEDS. 
 *
 */

#include "nrf_gzll.h"
#include "nrf_gzp.h"
#include "nrf_gpio.h"

/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/

#define UNENCRYPTED_DATA_PIPE 2 ///< Pipes 0 and 1 are reserved for GZP pairing and data. See nrf_gzp.h.
#define NRF_GZLLDE_RXPERIOD_DIV_2 504      ///< RXPERIOD/2 on LU1 = timeslot period on nRF51
#define BUTTONS NRF_GPIO_PORT_SELECT_PORT0 ///< GPIO port for reading from buttons
#define LEDS    NRF_GPIO_PORT_SELECT_PORT1 ///< GPIO port for writing to LEDs

/** @} */


/*****************************************************************************/
/** 
 * @brief Main function. 
 * 
 * @return ANSI required int return type.
 */
/*****************************************************************************/
int main(void)
{
    // Debug helper variables
    uint32_t length;

    // Data and acknowledgement payloads
    uint8_t payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; ///< Payload to send to Host. 

    // Setup port directions
    nrf_gpio_port_dir_set(BUTTONS, NRF_GPIO_PORT_DIR_INPUT);
    nrf_gpio_port_dir_set(LEDS, NRF_GPIO_PORT_DIR_OUTPUT);

    // Initialize Gazell Link Layer
    (void)nrf_gzll_init(NRF_GZLL_MODE_HOST);
    (void)nrf_gzll_set_timeslot_period(NRF_GZLLDE_RXPERIOD_DIV_2);  // Half RX period on nRF24Lxx device
    
    // Initialize Gazell Pairing Library
    gzp_init();
    (void)nrf_gzll_set_rx_pipes_enabled(nrf_gzll_get_rx_pipes_enabled() | (1 << UNENCRYPTED_DATA_PIPE));
    gzp_pairing_enable(true);

    (void)nrf_gzll_enable();

    for(;;)
    {
        gzp_host_execute();

        // If Host ID request received
        if(gzp_id_req_received())
        {
            // Always grant request
            gzp_id_req_grant();
        }

        length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;
        
        if( nrf_gzll_get_rx_fifo_packet_count(UNENCRYPTED_DATA_PIPE) )
        {
            if(nrf_gzll_fetch_packet_from_rx_fifo(UNENCRYPTED_DATA_PIPE, payload, &length))
            {
                nrf_gpio_port_write(LEDS, payload[0]);
            }
        }
        else if (gzp_crypt_user_data_received())
        {
            if(gzp_crypt_user_data_read(payload, (uint8_t*)&length))
            {
                nrf_gpio_port_write(LEDS, payload[0]);
            }
        }
    }
}

/** @} */
/** @} */
