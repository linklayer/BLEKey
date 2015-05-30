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
* $LastChangedRevision: 15516 $
*/

/** 
 * @file
 * @brief Enhanced Shockburst Transmitter (PTX) example
 * @defgroup esb_ptx_example Enhanced Shockburst Transmitter (PTX)
 * @{
 * @ingroup esb_03_examples
 *
 * This project requires that a device running the 
 * @ref esb_ptx_example example be used as a counterpart for 
 * receiving the data. This can be on either nRF51 device or a nRF24Lxx device
 * running the \b esb_ptx_example in the nRFgo SDK.
 * 
 * This example sends a packet and adds a new packet to the TX queue every time
 * it receives an ACK. The contents of GPIO Port BUTTONS are 
 * sent in the first payload byte (byte 0). The host sends an acknowledgement
 * that is received by the device. The contents of the first payload byte of 
 * the acknowledgement is output on GPIO Port LEDS.
 */


#include "nrf_esb.h"
#include "nrf_gpio.h"

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
static uint8_t my_tx_payload[TX_PAYLOAD_LENGTH];                ///< Payload to send to PRX. 
static uint8_t my_rx_payload[NRF_ESB_CONST_MAX_PAYLOAD_LENGTH]; ///< Placeholder for received ACK payloads from PRX.

/** @} */


/*****************************************************************************/
/** 
* @brief Main function. 
* 
* @return ANSI required int return type.
*/
/*****************************************************************************/
int main()
{
    // Setup port directions
    nrf_gpio_port_dir_set(BUTTONS, NRF_GPIO_PORT_DIR_INPUT);
    nrf_gpio_port_dir_set(LEDS, NRF_GPIO_PORT_DIR_OUTPUT);

    // Initialize ESB
    (void)nrf_esb_init(NRF_ESB_MODE_PTX);

    (void)nrf_esb_enable();  

    // Add packet into TX queue
    my_tx_payload[0] = nrf_gpio_port_read(BUTTONS);
    (void)nrf_esb_add_packet_to_tx_fifo(PIPE_NUMBER, my_tx_payload, TX_PAYLOAD_LENGTH, NRF_ESB_PACKET_USE_ACK);

    while(1)
    {
        // Optionally set the CPU to sleep while waiting for a callback.
        // __WFI();
    }
}


/*****************************************************************************/
/** @name ESB callback function definitions  */
/*****************************************************************************/


// If an ACK was received, we send another packet. 
void nrf_esb_tx_success(uint32_t tx_pipe, int32_t rssi){
    // Read buttons and load data payload into TX queue
    my_tx_payload[0] = nrf_gpio_port_read(BUTTONS);  
    (void)nrf_esb_add_packet_to_tx_fifo(PIPE_NUMBER, my_tx_payload, TX_PAYLOAD_LENGTH, NRF_ESB_PACKET_USE_ACK);
}


// If the transmission failed, send a new packet.
void nrf_esb_tx_failed(uint32_t tx_pipe){
    (void)nrf_esb_add_packet_to_tx_fifo(PIPE_NUMBER, my_tx_payload, TX_PAYLOAD_LENGTH, NRF_ESB_PACKET_USE_ACK);}


void nrf_esb_rx_data_ready(uint32_t rx_pipe, int32_t rssi){
    uint32_t my_rx_payload_length;
    // Pop packet and write first byte of the payload to the GPIO port.
    (void)nrf_esb_fetch_packet_from_rx_fifo(PIPE_NUMBER, my_rx_payload, &my_rx_payload_length);  
    if (my_rx_payload_length > 0)
    {
        nrf_gpio_port_write(LEDS, ~my_rx_payload[0]); // Button press is active low.
    }
}

// Callbacks not needed in this example.
void nrf_esb_disabled(void)
{}

/** @} */
/** @} */



