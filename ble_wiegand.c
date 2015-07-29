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

/* Attention!
 *  To maintain compliance with Nordic Semiconductor ASAs Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#include "ble_wiegand.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "nrf_gpio.h"
#include "wiegand.h"

#define BLE_UUID_WIEGAND_SERVICE        0xABCD

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_wiegand       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_wiegand_t * p_wiegand, ble_evt_t * p_ble_evt)
{
    p_wiegand->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_wiegand       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_wiegand_t * p_wiegand, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_wiegand->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_wiegand       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_wiegand_t * p_wiegand, ble_evt_t * p_ble_evt)
{
    uint8_t read_data[10];
    uint16_t len;

    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_wiegand->last_cards_handles.cccd_handle)
    {
        return;
    }
    if (p_evt_write->handle == p_wiegand->replay_handles.value_handle)
    {
	sd_ble_gatts_value_get(p_wiegand->replay_handles.value_handle, 0, &len, 
			       read_data);
    	send_wiegand(read_data[0]);
    	return;
    }
    if (p_evt_write->handle == p_wiegand->send_data_handles.value_handle)
    {
        return;
    }
    if (p_evt_write->handle == p_wiegand->data_length_handles.value_handle)
    {
  return;
    }

}


void ble_wiegand_on_ble_evt(ble_wiegand_t * p_wiegand, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_wiegand, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_wiegand, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_wiegand, p_ble_evt);
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for reading most recently read cards
 *
 * @param[in]   p_wiegand        Heart Rate Service structure.
 * @param[in]   p_wiegand_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t last_cards_char_add(ble_wiegand_t            * p_wiegand,
                                    const ble_wiegand_init_t * p_wiegand_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_WIEGAND_LAST_CARDS);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_wiegand_init->wiegand_last_cards_attr_md.read_perm;
    attr_md.write_perm = p_wiegand_init->wiegand_last_cards_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_MAX_TX_LEN;
    attr_char_value.p_value   = 0;

    return sd_ble_gatts_characteristic_add(p_wiegand->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_wiegand->last_cards_handles);
}

/**@brief Function for adding the Body Sensor Location characteristic.
 *
 * @param[in]   p_wiegand        Heart Rate Service structure.
 * @param[in]   p_wiegand_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t replay_char_add(ble_wiegand_t * p_wiegand, const ble_wiegand_init_t * p_wiegand_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_WIEGAND_REPLAY);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_wiegand_init->wiegand_replay_attr_md.read_perm;
    attr_md.write_perm = p_wiegand_init->wiegand_replay_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof (uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof (uint8_t);
    attr_char_value.p_value   = 0;

    return sd_ble_gatts_characteristic_add(p_wiegand->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_wiegand->replay_handles);
}

/**@brief Function for adding the Body Sensor Location characteristic.
 *
 * @param[in]   p_wiegand        Heart Rate Service structure.
 * @param[in]   p_wiegand_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t send_data_char_add(ble_wiegand_t * p_wiegand,
                                   const ble_wiegand_init_t * p_wiegand_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_WIEGAND_SEND_DATA);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_wiegand_init->wiegand_send_data_attr_md.read_perm;
    attr_md.write_perm = p_wiegand_init->wiegand_send_data_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = BLE_UUID_WIEGAND_SEND_DATA_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_UUID_WIEGAND_SEND_DATA_LEN;
    attr_char_value.p_value   = 0;

    return sd_ble_gatts_characteristic_add(p_wiegand->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_wiegand->send_data_handles);
}



/**@brief Function for adding the Body Sensor Location characteristic.
 *
 * @param[in]   p_wiegand        Heart Rate Service structure.
 * @param[in]   p_wiegand_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t data_length_char_add(ble_wiegand_t * p_wiegand, const ble_wiegand_init_t * p_wiegand_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_WIEGAND_DATA_LENGTH);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_wiegand_init->wiegand_data_length_attr_md.read_perm;
    attr_md.write_perm = p_wiegand_init->wiegand_data_length_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof (uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof (uint8_t);
    attr_char_value.p_value   = 0;

    return sd_ble_gatts_characteristic_add(p_wiegand->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_wiegand->data_length_handles);
}


uint32_t ble_wiegand_init(ble_wiegand_t * p_wiegand, const ble_wiegand_init_t * p_wiegand_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_wiegand->evt_handler                 = p_wiegand_init->evt_handler;
    p_wiegand->conn_handle                 = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_WIEGAND_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_wiegand->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add last_card characteristic
    err_code = last_cards_char_add(p_wiegand, p_wiegand_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add replay characteristic
    err_code = replay_char_add(p_wiegand, p_wiegand_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add send_data characteristic
    err_code = send_data_char_add(p_wiegand, p_wiegand_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add data_length characteristic
    err_code = data_length_char_add(p_wiegand, p_wiegand_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }


    return NRF_SUCCESS;
}

uint32_t ble_wiegand_sensor_contact_supported_set(ble_wiegand_t * p_wiegand, bool is_sensor_contact_supported)
{
    // Check if we are connected to peer
    if (p_wiegand->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        p_wiegand->is_sensor_contact_supported = is_sensor_contact_supported;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}

void ble_wiegand_sensor_contact_detected_update(ble_wiegand_t * p_wiegand, bool is_sensor_contact_detected)
{
    p_wiegand->is_sensor_contact_detected = is_sensor_contact_detected;
}


uint32_t ble_wiegand_body_sensor_location_set(ble_wiegand_t * p_wiegand, uint8_t body_sensor_location)
{
    uint16_t len = sizeof(uint8_t);
    return sd_ble_gatts_value_set(p_wiegand->replay_handles.value_handle, 0, &len, &body_sensor_location);
}

uint32_t ble_wiegand_last_cards_set(ble_wiegand_t * p_wiegand, uint8_t *cards, uint16_t len)
{
    return sd_ble_gatts_value_set(p_wiegand->last_cards_handles.value_handle,
                                  0, &len, cards);
}
