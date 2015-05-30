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
 
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "dfu.h"
#include "dfu_types.h"
#include "dfu_bank_internal.h"
#include "app_util.h"
#include "app_error.h"
#include "bootloader.h"
#include "bootloader_types.h"
#include "ble_flash.h"
#include "crc16.h"
#include "nrf.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "pstorage.h"
#include "nrf_mbr.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "nordic_common.h"

static dfu_state_t             m_dfu_state;                  /**< Current DFU state. */
static uint32_t                m_image_size;                 /**< Size of the image that will be transmitted. */

static dfu_start_packet_t      m_start_packet;               /**< Start packet received for this update procedure. Contains update mode and image sizes information to be used for image transfer. */
static uint32_t                m_init_packet[16];            /**< Init packet, can hold CRC, Hash, Signed Hash and similar, for image validation, integrety check and authorization checking. */
static uint8_t                 m_init_packet_length;         /**< Length of init packet received. */
static uint16_t                m_image_crc;                  /**< Calculated CRC of the image received. */
static app_timer_id_t          m_dfu_timer_id;               /**< Application timer id. */
static bool                    m_dfu_timed_out = false;      /**< Boolean flag value for tracking DFU timer timeout state. */
static pstorage_handle_t       m_storage_handle_app;
static pstorage_module_param_t m_storage_module_param;
static dfu_callback_t          m_data_pkt_cb;


static void pstorage_callback_handler(pstorage_handle_t * handle, 
                                      uint8_t             op_code, 
                                      uint32_t            result, 
                                      uint8_t           * p_data, 
                                      uint32_t            data_len)
{
    if (m_data_pkt_cb != NULL)
    {
        switch (op_code)
        {
            case PSTORAGE_STORE_OP_CODE:
                if (m_dfu_state == DFU_STATE_RX_DATA_PKT)
                {
                    m_data_pkt_cb(DATA_PACKET, result, p_data);
                }
                break;
            
            case PSTORAGE_CLEAR_OP_CODE:
                if (m_dfu_state == DFU_STATE_PREPARING)
                {
                    dfu_update_status_t update_status = {DFU_BANK_0_ERASED, };
                    bootloader_dfu_update_process(update_status);

                    m_dfu_state = DFU_STATE_RDY;
                    m_data_pkt_cb(START_PACKET, result, p_data);
                }
                break;
            
            default:
                break;
        }
    }
    APP_ERROR_CHECK(result);
}


/**@brief Function for handling the DFU timeout.
 *
 * @param[in] p_context The timeout context.
 */
static void dfu_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    dfu_update_status_t update_status;
    
    m_dfu_timed_out           = true;
    update_status.status_code = DFU_TIMEOUT;

    bootloader_dfu_update_process(update_status);
}


/**@brief   Function for restarting the DFU Timer.
 *
 * @details This function will stop and restart the DFU timer. This function will be called by the 
 *          functions handling any DFU packet received from the peer that is transferring a firmware 
 *          image.
 */
static uint32_t dfu_timer_restart(void)
{
    if (m_dfu_timed_out)
    {
        // The DFU timer had already timed out.
        return NRF_ERROR_INVALID_STATE;
    }

    uint32_t err_code = app_timer_stop(m_dfu_timer_id);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_dfu_timer_id, DFU_TIMEOUT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);    
    
    return err_code;
}


uint32_t dfu_init(void)
{        
    uint32_t  err_code = NRF_SUCCESS;
    
    m_dfu_state          = DFU_STATE_IDLE;    
    m_init_packet_length = 0;
    m_image_crc          = 0;    
    m_data_received      = 0;

    m_storage_module_param.cb          = pstorage_callback_handler;

    err_code = pstorage_raw_register(&m_storage_module_param, &m_storage_handle_app);
    if (err_code != NRF_SUCCESS)
    {
        m_dfu_state = DFU_STATE_INIT_ERROR;
        return err_code;
    }

    m_storage_handle_app.block_id   = CODE_REGION_1_START;

    // Create the timer to monitor the activity by the peer doing the firmware update.
    err_code = app_timer_create(&m_dfu_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                dfu_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Start the DFU timer.
    err_code = app_timer_start(m_dfu_timer_id, DFU_TIMEOUT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    
    return NRF_SUCCESS;
}


void dfu_register_callback(dfu_callback_t callback_handler)
{
    m_data_pkt_cb = callback_handler;
}


static void dfu_app_erase(uint32_t image_size)
{
    dfu_update_status_t update_status;
    uint32_t            err_code;
    
    update_status.status_code = DFU_BANK_0_ERASED;
    bootloader_dfu_update_process(update_status);
                
    err_code = pstorage_raw_clear(&m_storage_handle_app, m_image_size);
    APP_ERROR_CHECK(err_code);

}


uint32_t dfu_start_pkt_handle(dfu_update_packet_t * p_packet)
{
    uint32_t err_code;        
    
    memcpy(&m_start_packet, p_packet->params.start_packet, sizeof(dfu_start_packet_t));

    // Check that the requested update procedure is supported.
    // Currently the following combinations are allowed:
    // - Application
    if (IS_UPDATING_APP(m_start_packet) && 
        (IS_UPDATING_SD(m_start_packet) || 
         IS_UPDATING_BL(m_start_packet) || 
         ((m_start_packet.app_image_size & (sizeof(uint32_t) - 1)) != 0)))
    {
        // Image_size is not a multiple of 4 (word size).
        return NRF_ERROR_NOT_SUPPORTED;
    }
    
    if (IS_UPDATING_SD(m_start_packet))
    {
        // SoftDevice update not supported.
        return NRF_ERROR_NOT_SUPPORTED;
    }

    if (IS_UPDATING_BL(m_start_packet))
    {
        // Bootloader update not supported.
        return NRF_ERROR_NOT_SUPPORTED;
    }
    
    m_image_size = m_start_packet.sd_image_size + m_start_packet.bl_image_size + m_start_packet.app_image_size;


    if (m_image_size > DFU_IMAGE_MAX_SIZE_FULL)
    {
        return NRF_ERROR_DATA_SIZE;
    }
    else
    {
        // Do nothing.
    }


    switch (m_dfu_state)
    {
        case DFU_STATE_IDLE:    
            // Valid peer activity detected. Hence restart the DFU timer.
            err_code = dfu_timer_restart();
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }        
            
            if (IS_UPDATING_APP(m_start_packet))
            {
                dfu_app_erase(m_image_size);
            }
            
            m_dfu_state  = DFU_STATE_RDY;    
            break;
            
        default:
            err_code = NRF_ERROR_INVALID_STATE;
            break;
    }
    
    return err_code;    
}


uint32_t dfu_data_pkt_handle(dfu_update_packet_t * p_packet)
{
    uint32_t  err_code;
    uint32_t  data_length;
    uint8_t * p_data;
    
    if (p_packet == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Check pointer alignment.
    if (!is_word_aligned(p_packet->params.data_packet.p_data_packet))
    {
        // The p_data_packet is not word aligned address.
        return NRF_ERROR_INVALID_ADDR;
    }

    switch (m_dfu_state)
    {
        case DFU_STATE_RDY:
            // Fall-through.
        
        case DFU_STATE_RX_INIT_PKT:
            m_dfu_state = DFU_STATE_RX_DATA_PKT;
        
            // Fall-through.
        
        case DFU_STATE_RX_DATA_PKT:
            data_length = p_packet->params.data_packet.packet_length  * sizeof(uint32_t);
            if ((uint32_t)(m_data_received + data_length) > m_image_size)
            {
                // The caller is trying to write more bytes into the flash than the size provided to 
                // the dfu_image_size_set function.
                return NRF_ERROR_DATA_SIZE;
            }
            
            // Valid peer activity detected. Hence restart the DFU timer.
            err_code = dfu_timer_restart();
            
            p_data   = (uint8_t *)p_packet->params.data_packet.p_data_packet;            
            err_code = pstorage_raw_store(&m_storage_handle_app, p_data, data_length, m_data_received);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
            m_data_received += data_length;        
            
            break;
            
        default:
            err_code = NRF_ERROR_INVALID_STATE;
            break;
    }
    
    return err_code;
}


uint32_t dfu_init_pkt_handle(dfu_update_packet_t * p_packet)
{
    uint32_t err_code;
    uint32_t i;
    
    switch (m_dfu_state)
    {
        case DFU_STATE_RDY:
            m_dfu_state = DFU_STATE_RX_INIT_PKT;
            // Fall-through.     
            
        case DFU_STATE_RX_INIT_PKT:
            // DFU initialization has been done and a start packet has been received.
            if (IMAGE_WRITE_IN_PROGRESS())
            {
                // Image write is already in progress. Cannot handle an init packet now.
                return NRF_ERROR_INVALID_STATE;
            }

            // Valid peer activity detected. Hence restart the DFU timer.
            err_code = dfu_timer_restart();
            
            for (i = 0; i < p_packet->params.data_packet.packet_length; i++)
            {
                m_init_packet[m_init_packet_length++] = p_packet->params.data_packet.p_data_packet[i];
            }
            
            err_code = NRF_SUCCESS;
            break;

        default:
            // Either the start packet was not received or dfu_init function was not called before.
            err_code = NRF_ERROR_INVALID_STATE;        
            break;
    }
    
    return err_code;    
}


uint32_t dfu_image_validate()
{
    uint32_t err_code;
    uint16_t received_crc;    

    switch (m_dfu_state)
    {
        case DFU_STATE_RX_DATA_PKT:
            m_dfu_state = DFU_STATE_VALIDATE;
            
            // Check if the application image write has finished.
            if (m_data_received != m_image_size)
            {
                // Image not yet fully transferred by the peer.
                err_code = NRF_ERROR_INVALID_STATE;
            }
            else
            {
                // Valid peer activity detected. Hence restart the DFU timer.
                err_code = dfu_timer_restart();
                
                // Calculate CRC from DFU_BANK_0_REGION_START to mp_app_write_address.
                m_image_crc  = crc16_compute((uint8_t*)DFU_BANK_0_REGION_START, 
                                             m_image_size, 
                                             NULL);
                received_crc = uint16_decode((uint8_t*)&m_init_packet[0]);
                    
                if ((m_init_packet_length != 0) && (m_image_crc != received_crc))
                {
                    return NRF_ERROR_INVALID_DATA;
                }
                m_dfu_state = DFU_STATE_WAIT_4_ACTIVATE;
                err_code    = NRF_SUCCESS;
            }

            break;            
            
        default:
            err_code = NRF_ERROR_INVALID_STATE;
            break;  
    }
    
    return err_code;    
}


uint32_t dfu_image_activate()
{
    uint32_t            err_code;
    dfu_update_status_t update_status;
    
    switch (m_dfu_state)
    {    
        case DFU_STATE_WAIT_4_ACTIVATE:            
            // Stop the DFU Timer because the peer activity need not be monitored any longer.
            err_code = app_timer_stop(m_dfu_timer_id);
        
            update_status.status_code = DFU_UPDATE_APP_COMPLETE;
            update_status.app_crc     = m_image_crc;
            update_status.app_size    = m_image_size;

            bootloader_dfu_update_process(update_status);        
            err_code = NRF_SUCCESS;
            break;
            
        default:
            err_code = NRF_ERROR_INVALID_STATE;
            break;
    }
    
    return err_code;
}


void dfu_reset(void)
{
    dfu_update_status_t update_status;
    
    update_status.status_code = DFU_RESET;

    bootloader_dfu_update_process(update_status);
}


uint32_t dfu_sd_image_swap(void)
{
    // Not implemented.
    return NRF_SUCCESS;
}


uint32_t dfu_bl_image_swap(void)
{
    // Not implemented.
    return NRF_SUCCESS;
}


uint32_t dfu_bl_image_validate(void)
{
    // Not implemented.
    return NRF_SUCCESS;
}


uint32_t dfu_sd_image_validate(void)
{
    // Not implemented.
    return NRF_SUCCESS;
}
