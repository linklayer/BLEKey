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
* $LastChangedRevision: 40042 $
*/

/** 
 * @file
 * @brief Gazell Link Layer Host with Payload in ACK example
 * @defgroup gzll_host_ack_payload_example Gazell Link Layer Host with Payload in ACK
 * @{
 * @ingroup gzll_03_examples
 *
 * This project requires that a Device running the 
 * @ref gzll_device_ack_payload_example be used as a counterpart for 
 * receiving the data. This can be on either nRF51 device or a nRF24Lxx device
 * running the \b gzll_device_ack_payload example in the nRFgo SDK. 
 * 
 * This example listens for a packet and sends an ACK
 * when a packet is received. The contents of the first payload byte of 
 * the received packet is output on the GPIO Port BUTTONS. 
 * The contents of GPIO Port LEDS are sent in the first payload byte (byte 0) 
 * of the ACK packet.
 */


#include "nrf_gzll.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

/*****************************************************************************/
/** @name Configuration  */
/*****************************************************************************/

// Define pipe
#define PIPE_NUMBER 0 ///< We use pipe 0 in this example

// GPIO
#define BUTTONS NRF_GPIO_PORT_SELECT_PORT0 ///< GPIO port for reading from buttons
#define LEDS    NRF_GPIO_PORT_SELECT_PORT1 ///< GPIO port for writing to LEDs

// Define payload length
#define TX_PAYLOAD_LENGTH 1 ///< We use 1 byte payload length when transmitting

// Data and acknowledgement payloads
static uint8_t data_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];  ///< Placeholder for data payload received from host. 
static uint8_t ack_payload[TX_PAYLOAD_LENGTH];                   ///< Payload to attach to ACK sent to device.

// Debug helper variables
extern nrf_gzll_error_code_t nrf_gzll_error_code;   ///< Error code
static bool init_ok, enable_ok, push_ok, pop_ok, packet_received;  

/** @} */

/*****************************************************************************/
/**
* @brief Main function.
* @return ANSI required int return type.
*/
/*****************************************************************************/
int main()
{
    uint8_t debug_led_output;

    // Setup port directions
    nrf_gpio_port_dir_set(BUTTONS,NRF_GPIO_PORT_DIR_INPUT);
    nrf_gpio_port_dir_set(LEDS,NRF_GPIO_PORT_DIR_OUTPUT);
    
    // Initialize Gazell
    init_ok = nrf_gzll_init(NRF_GZLL_MODE_HOST);  
    
    // Load data into TX queue
    ack_payload[0] = nrf_gpio_port_read(BUTTONS);  // Button logic is inverted.
    push_ok = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, data_payload, TX_PAYLOAD_LENGTH);

    // Enable Gazell to start sending over the air
    enable_ok = nrf_gzll_enable();

    while(1)
    {
        // Error handling
        debug_led_output = ((uint8_t)packet_received << 4) + ((uint8_t)pop_ok << 3) + ((uint8_t)push_ok << 2) + ((uint8_t)enable_ok << 1) + (uint8_t)init_ok;                
        
        // If an error has occured
        if(debug_led_output != 0x1F)
        {
            nrf_gpio_port_write(LEDS,0xFF);      
            nrf_delay_us(1000000); // 1 second delay 
            nrf_gpio_port_write(LEDS, debug_led_output);
            nrf_delay_us(1000000); // 1 second delay 
            nrf_gpio_port_write(LEDS,0xF0 + (uint32_t)nrf_gzll_get_error_code()); 
            nrf_delay_us(1000000); // 1 second delay 
        }

        // Optionally send the CPU to sleep while waiting for a callback.
        // __WFI();
    }
}


/*****************************************************************************/
/** @name Gazell callback function definitions  */
/*****************************************************************************/


// If a data packet was received, we write the first byte to LEDS. 
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{   
    uint32_t data_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;
    packet_received = true;

    // Pop packet and write first byte of the payload to the GPIO port.
    pop_ok = nrf_gzll_fetch_packet_from_rx_fifo(pipe, data_payload, &data_payload_length);
    if (data_payload_length > 0)
    {
        nrf_gpio_port_write(LEDS,~data_payload[0]);
    }

    // Read buttons and load ACK payload into TX queue
    ack_payload[0] = nrf_gpio_port_read(BUTTONS);  // Button logic is inverted.
    push_ok = nrf_gzll_add_packet_to_tx_fifo(pipe, ack_payload, TX_PAYLOAD_LENGTH);

}

// Callbacks not needed in this example.
void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_disabled() {}

/** @} */
/** @} */

