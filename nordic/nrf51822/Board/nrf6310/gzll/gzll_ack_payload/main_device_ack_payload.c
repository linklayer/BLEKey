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
 * @brief Gazell Link Layer Device with Payload in ACK example
 * @defgroup gzll_device_ack_payload_example Gazell Link Layer Device with Payload in ACK
 * @{
 * @ingroup gzll_03_examples
 *
 * This project requires that a Host running the 
 * @ref gzll_host_ack_payload_example example be used as a counterpart for 
 * receiving the data. This can be on either nRF51 device or a nRF24Lxx device
 * running the \b gzll_host_ack_payload example in the nRFgo SDK. 
 * 
 * This example sends a packet and adds a new packet to the TX queue every time
 * it receives an ACK. Before adding a packet to the TX queue, the contents of 
 * the GPIO Port BUTTONS is copied to the first payload byte (byte 0). 
 * When an ACK is received, the contents of the first payload byte of 
 * the ACK are output on GPIO Port LEDS. 
 */


#include "nrf_gzll.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/

// Define pipe
#define PIPE_NUMBER 0 ///< We use pipe 0 in this example

// GPIO
#define BUTTONS NRF_GPIO_PORT_SELECT_PORT0 ///< GPIO port for reading from buttons
#define LEDS    NRF_GPIO_PORT_SELECT_PORT1 ///< GPIO port for writing to LEDs

// Define payload length
#define TX_PAYLOAD_LENGTH 1 ///< We use 1 byte payload length when transmitting

// Data and acknowledgement payloads
static uint8_t data_payload[TX_PAYLOAD_LENGTH];                ///< Payload to send to Host. 
static uint8_t ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; ///< Placeholder for received ACK payloads from Host.

// Debug helper variables
static volatile bool init_ok, enable_ok, push_ok, pop_ok, tx_success;  

/** @} */


/*****************************************************************************/
/** 
* @brief Main function. 
* 
* @return ANSI required int return type.
*/
/*****************************************************************************/
//lint -save -e514 Unusual use of a boolean expression (use of &= assignment).

int main()
{
    uint8_t debug_led_output;

    // Setup port directions
    nrf_gpio_port_dir_set(BUTTONS, NRF_GPIO_PORT_DIR_INPUT);
    nrf_gpio_port_dir_set(LEDS, NRF_GPIO_PORT_DIR_OUTPUT);

    // Initialize Gazell
    init_ok = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    
    // Attempt sending every packet up to 100 times    
    init_ok &= nrf_gzll_set_max_tx_attempts(100);

    // Load data into TX queue
    data_payload[0] = nrf_gpio_port_read(BUTTONS);  
    push_ok = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, data_payload, TX_PAYLOAD_LENGTH);
    
    // Enable Gazell to start sending over the air
    enable_ok = nrf_gzll_enable();         

    while(1)
    {
        // Error handling
        debug_led_output = ((uint8_t)tx_success << 4) | ((uint8_t)pop_ok << 3) | ((uint8_t)push_ok << 2) | ((uint8_t)enable_ok << 1) | (uint8_t)init_ok;                
        
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


// If an ACK was received, we send another packet. 
void  nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    uint32_t ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;    
    tx_success = true;

    if (tx_info.payload_received_in_ack)
    {
        // Pop packet and write first byte of the payload to the GPIO port.
        pop_ok = nrf_gzll_fetch_packet_from_rx_fifo(pipe, ack_payload, &ack_payload_length);
        if (ack_payload_length > 0)
        {
            nrf_gpio_port_write(LEDS, ~ack_payload[0]);  // Button press is active low. 
        }
    }
    
    // Read buttons and load data payload into TX queue
    data_payload[0] = nrf_gpio_port_read(BUTTONS);
    push_ok = nrf_gzll_add_packet_to_tx_fifo(pipe, data_payload, TX_PAYLOAD_LENGTH);
}


/* If the transmission failed, send a new packet.
 * This callback does not occur by default since NRF_GZLL_DEFAULT_MAX_TX_ATTEMPTS 
 * is 0 (inifinite retransmits)
 */
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    tx_success = false;

    // Load data into TX queue
    data_payload[0] = nrf_gpio_port_read(BUTTONS);
    push_ok = nrf_gzll_add_packet_to_tx_fifo(pipe, data_payload, TX_PAYLOAD_LENGTH);
}



// Callbacks not needed in this example.
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{}
void nrf_gzll_disabled()
{}

/** @} */
/** @} */



