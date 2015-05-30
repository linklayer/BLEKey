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
* $LastChangedRevision: 25419 $
*/

/**
 * @file
 * @brief Gazell Pairing Desktop Device Emulator example.
 *
 * @defgroup gzp_desktop_device_emulator_example Gazell Pairing Desktop Device Emulator
 * @{
 * @ingroup gzp_03_examples
 *
 * This project can be used as a starting point for developing a nRF51 series
 * mouse or keyboard using Gazell for communicating with a legacy nRF24LU1
 * USB dongle. It can communicate "out of the box" with the 
 * Dongle reference design and the nRFreadySimplePairing.exe application, 
 * that can be found in the nRFready Desktop v1.2.3. 
 * 
 * This project sends mouse movement packets to the USB dongle when pin 1
 * is low (button connected to pin 1 pressed) and sends the 'a' keyboard
 * character to the USB dongle when pin 2 goes from high to low (button
 * connected to pin 2 pressed).
 */

#include "nrf_gzll.h"
#include "nrf_gzp.h"
#include "nrf_gpio.h"
#include "nrf_gzllde_params.h"
#include "mouse_sensor_emulator.h"
#include "keyboard_emulator.h"
#include "nrf_delay.h"

/*****************************************************************************/
/** @name Configuration                                                      */
/*****************************************************************************/

#define BUTTON_SEND_KEYBOARD_DATA   0  ///< GPIO pin for reading from a button to emulate a keypress.
#define BUTTON_SEND_MOUSE_DATA      1  ///< GPIO pin for reading from a button to emulate a mouse movement.

#define LED_PORT NRF_GPIO_PORT_SELECT_PORT1
#define KEYBOARD_LED     8
#define MOUSE_LED        9
#define SYS_ADDR_OK_LED 10
#define HOST_ID_OK_LED  11
#define DYN_KEY_OK_LED  12

/** @} */


/*****************************************************************************/
/** @name Static (internal) functions. */
/*****************************************************************************/

/**
 * Checks to see whether the mouse sensor has data to send. If so, adds this
 * to the (unencrypted) Gazell TX FIFO on pipe NRFR_MOUSE_EP.
 */
static void read_mouse_and_send(void);


/**
 * Checks to see whether the mouse sensor has data and send unencrypted.
 *
 * If so, adds this to the (unencrypted) Gazell TX FIFO on pipe NRFR_MOUSE_EP.
 */

/**
 * Checks to see whether the keyboard has data and send encrypted.
 * 
 * If the Device does net yet have the system address it will try to 
 * obtain it. After obtaining the system address it will attempt to obtain
 * the Host ID.
 * The keyboard data is discarded if pairing is not successful.
 * It may take a few attempts to obtain the Host ID as this may not be
 * yet generated at the Host.
 */
static void read_keyboard_and_send(void);

/**
 * Send a Host ID request and process the response.
 * 
 * If the request was rejected or failed (i.e. timed out), system_addr_received
 * will be reset and the pairing process will begin on the next keypress.
 *
 * If the request was received, subsequent keyboard data will be transmitted
 * on an encrypted link. 
 *
 * If teh request is still pending, nothing is done. Further keypresses
 * will initiate another 
 * 
 * @return The result of the Host ID request.
 */
static gzp_id_req_res_t send_host_id_req(void);
/** @} */


static bool host_id_received = false;     ///< Host ID received.
static bool system_addr_received = false; ///< System address receivedfrom Host.
static bool dyn_key_ok = false;           ///< Dynamic key is up to date. 
/*****************************************************************************/
/**
 * @brief Main function.
 *
 * @return ANSI required int return type.
 */
/*****************************************************************************/


int main()
{
     bool init_ok = false;
//lint -save -e514 Unusual use of a boolean expression (use of &= assignment).

    // Configure input pins
    nrf_gpio_pin_dir_set(BUTTON_SEND_MOUSE_DATA, NRF_GPIO_PIN_DIR_INPUT);
    nrf_gpio_pin_dir_set(BUTTON_SEND_KEYBOARD_DATA, NRF_GPIO_PIN_DIR_INPUT);
    nrf_gpio_port_dir_set(LED_PORT, NRF_GPIO_PORT_DIR_OUTPUT);

    // Initialize and enable "mouse sensor"
    init_ok = mouse_sensor_init(MOUSE_SENSOR_SAMPLE_PERIOD_8_MS);
    mouse_sensor_enable();

    // Initialize and enable Gazell
    init_ok &= nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    
    // Ensure Gazell parameters are configured.
    init_ok &= nrf_gzll_set_max_tx_attempts(150);
    init_ok &= nrf_gzll_set_device_channel_selection_policy(NRF_GZLLDE_DEVICE_CHANNEL_SELECTION_POLICY);
    init_ok &= nrf_gzll_set_timeslot_period(NRF_GZLLDE_RXPERIOD_DIV_2);
    init_ok &= nrf_gzll_set_sync_lifetime(0); // Asynchronous mode, more efficient for pairing.

    switch(gzp_get_pairing_status())    
    {
      case -2:
        host_id_received = false;
        system_addr_received = false;
        break;
      case -1: 
        host_id_received = false;
        system_addr_received = true;
        break;
      default:
        host_id_received = true;
        system_addr_received = true;
    }
    
    gzp_init();

    init_ok &= nrf_gzll_enable();

    if(init_ok)
    {
        while(1)
        {
            // If BUTTON_SEND_MOUSE_DATA button is pressed.
            if(nrf_gpio_pin_read(BUTTON_SEND_MOUSE_DATA) == 0)
            {
                read_mouse_and_send();
            }
            else
            {
                nrf_gpio_pin_clear(MOUSE_LED);
            }

            // If BUTTON_SEND_KEYBOARD_DATA button is pressed
            if(nrf_gpio_pin_read(BUTTON_SEND_KEYBOARD_DATA) == 0)
            {
                read_keyboard_and_send();
            }
            else
            {
                nrf_gpio_pin_clear(KEYBOARD_LED);
            }
            
            nrf_gpio_pin_write(SYS_ADDR_OK_LED,system_addr_received);
            nrf_gpio_pin_write(HOST_ID_OK_LED,host_id_received);
            nrf_gpio_pin_write(DYN_KEY_OK_LED,dyn_key_ok);
            
            /*
            CPU sleep.
            We will wake up from all enabled interrupts, which here are the
            internal Gazell interrupts and the "mouse sensor" internal timer
            interrupt.
            */
            //__WFI();
            
        }
    }
    else
    {
        /*
        The initialization failed. Use nrf_gzll_get_error_code() 
        to investigate the cause.
        */
    }
//lint -restore
}



void mouse_sensor_new_sample_generated_cb()
{
    /*
    This callback is called every time the mouse sensor
    generates a new sample. We could select to add mouse packets to the
    TX FIFO here.
    */
}


static void read_mouse_and_send(void)
{
//lint -save -e514 Unusual use of a boolean expression (use of &= assignment).
    bool mouse_send_ok;

    // If the "mouse sensor" has data ready for read-out.
    if(mouse_sensor_data_is_ready())
    {
        uint8_t mouse_packet[NRFR_MOUSE_MOV_PACKET_LENGTH];

        // Get packet from "mouse sensor".
        if(mouse_sensor_read(mouse_packet))
        {
            // Wait in case the FIFOs are full.
            while(!nrf_gzll_ok_to_add_packet_to_tx_fifo(NRFR_MOUSE_EP))
            {}

            // Add mouse packet to the mouse pipe's TX FIFO.
            mouse_send_ok = nrf_gzll_add_packet_to_tx_fifo(NRFR_MOUSE_EP, mouse_packet, NRFR_MOUSE_MOV_PACKET_LENGTH);
            nrf_gpio_pin_write(MOUSE_LED,mouse_send_ok);
        }
    }
       

   

//lint -restore
}

static void read_keyboard_and_send(void)
{
    uint8_t keyboard_packet[NRFR_KEYBOARD_PACKET_LENGTH];

    // "Scan" keyboard.
    keyboard_get_non_empty_packet(keyboard_packet);

    // Send address request if required.
    if(!host_id_received  && !system_addr_received)
    {
        system_addr_received = gzp_address_req_send();
    }

    /* Send Host ID request if required. This may take a few attempts
     * as the Host may require some time to generate the Host ID. */
    if(!host_id_received  && system_addr_received )
    {
        while(send_host_id_req() == GZP_ID_RESP_PENDING);
    }

    /* After receiving the Host ID we send one packet in order
     * to update the dynamic key. 
     */
    if(host_id_received && !dyn_key_ok)
    {
        bool keyboard_send_ok = true;
        keyboard_send_ok = gzp_crypt_data_send(keyboard_packet, NRFR_KEYBOARD_PACKET_LENGTH);
        
        if(!keyboard_send_ok)
        {
            host_id_received = false;
        }
        else
        {
            dyn_key_ok = true;
        }
    }
    
    /* If we have the Host ID and dynamic key we can transmit encrypted data.
     */
    if(host_id_received && dyn_key_ok)
    {
        bool keyboard_send_ok = true;
        keyboard_send_ok = gzp_crypt_data_send(keyboard_packet, NRFR_KEYBOARD_PACKET_LENGTH);
        nrf_gpio_pin_write(KEYBOARD_LED,keyboard_send_ok);
        if(keyboard_send_ok)
        {
           // Wait until button is depressed.
            while(nrf_gpio_pin_read(BUTTON_SEND_KEYBOARD_DATA) == 0)
            {}
            // Send empty keyboard packet to release all keys.
            keyboard_get_empty_packet(keyboard_packet);
            keyboard_send_ok = gzp_crypt_data_send(keyboard_packet, NRFR_KEYBOARD_PACKET_LENGTH);
        }

        if(!keyboard_send_ok)
        {
            dyn_key_ok = false;
        }
    }

}

static gzp_id_req_res_t send_host_id_req(void)
{
    gzp_id_req_res_t id_resp;
    
    // Try sending "Host ID" request
    id_resp = gzp_id_req_send(); 

    switch(id_resp)
    {
        case GZP_ID_RESP_REJECTED:
        case GZP_ID_RESP_FAILED:
            host_id_received     = false;
            system_addr_received = false;
            break;
        case GZP_ID_RESP_GRANTED:
            host_id_received = true;
            system_addr_received = true;
            break;
        case GZP_ID_RESP_PENDING:
            default:
            break; 
    }
   
     return id_resp; 
}

/** @} */
/** @} */



