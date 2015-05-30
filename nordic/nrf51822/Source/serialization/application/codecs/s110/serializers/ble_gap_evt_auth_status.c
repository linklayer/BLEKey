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

#include "ble_gap_evt_app.h"
#include <string.h>
#include "ble_serialization.h"
#include "app_util.h"


uint32_t ble_gap_evt_auth_status_dec(uint8_t const * const p_buf,
                                     uint32_t              packet_len,
                                     ble_evt_t * const     p_event,
                                     uint32_t * const      p_event_len)
{
    uint32_t index = 0;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_event_len);

    uint32_t event_len = sizeof (ble_gap_evt_auth_status_t) +
                         sizeof (p_event->evt.gap_evt.conn_handle);

    if (p_event == NULL)
    {
        *p_event_len = event_len;
        return NRF_SUCCESS;
    }

    SER_ASSERT(event_len <= *p_event_len, NRF_ERROR_DATA_SIZE);

    SER_ASSERT_LENGTH_LEQ(2 + 1 + 1 + 1 + 1 + 1 + 2, packet_len);
    p_event->header.evt_len = event_len;
    uint16_dec(p_buf, packet_len, &index, &p_event->evt.gap_evt.conn_handle);

    ble_gap_evt_auth_status_t * p_decoded_evt = &(p_event->evt.gap_evt.params.auth_status);

    p_decoded_evt->auth_status = p_buf[index++];
    p_decoded_evt->error_src   = p_buf[index++];

    p_decoded_evt->sm1_levels.lv3 = (p_buf[index] >> 5) & 0x01;
    p_decoded_evt->sm1_levels.lv2 = (p_buf[index] >> 4) & 0x01;
    p_decoded_evt->sm1_levels.lv1 = (p_buf[index] >> 3) & 0x01;
    p_decoded_evt->sm2_levels.lv3 = (p_buf[index] >> 2) & 0x01;
    p_decoded_evt->sm2_levels.lv2 = (p_buf[index] >> 1) & 0x01;
    p_decoded_evt->sm2_levels.lv1 = (p_buf[index] >> 0) & 0x01;
    index++;

    p_decoded_evt->periph_kex.csrk      = (p_buf[index] >> 4) & 0x01;
    p_decoded_evt->periph_kex.address   = (p_buf[index] >> 3) & 0x01;
    p_decoded_evt->periph_kex.irk       = (p_buf[index] >> 2) & 0x01;
    p_decoded_evt->periph_kex.ediv_rand = (p_buf[index] >> 1) & 0x01;
    p_decoded_evt->periph_kex.ltk       = (p_buf[index] >> 0) & 0x01;
    index++;

    p_decoded_evt->central_kex.ltk       = (p_buf[index] >> 4) & 0x01;
    p_decoded_evt->central_kex.ediv_rand = (p_buf[index] >> 3) & 0x01;
    p_decoded_evt->central_kex.irk       = (p_buf[index] >> 2) & 0x01;
    p_decoded_evt->central_kex.address   = (p_buf[index] >> 1) & 0x01;
    p_decoded_evt->central_kex.csrk      = (p_buf[index] >> 0) & 0x01;
    index++;

    uint16_dec(p_buf, packet_len, &index, &p_decoded_evt->periph_keys.enc_info.div);

    SER_ASSERT_LENGTH_LEQ(index + BLE_GAP_SEC_KEY_LEN + 1 +
                          BLE_GAP_SEC_KEY_LEN + 1 + BLE_GAP_ADDR_LEN,
                          packet_len);
    memcpy(&p_decoded_evt->periph_keys.enc_info.ltk[0], &p_buf[index], BLE_GAP_SEC_KEY_LEN);
    index += BLE_GAP_SEC_KEY_LEN;

    p_decoded_evt->periph_keys.enc_info.ltk_len = (p_buf[index] >> 1);
    p_decoded_evt->periph_keys.enc_info.auth    = (p_buf[index] >> 0) & 0x01;
    index++;

    memcpy(&p_decoded_evt->central_keys.irk.irk[0], &p_buf[index], BLE_GAP_SEC_KEY_LEN);
    index += BLE_GAP_SEC_KEY_LEN;

    p_decoded_evt->central_keys.id_info.addr_type = p_buf[index++];

    memcpy(&p_decoded_evt->central_keys.id_info.addr[0], &p_buf[index], BLE_GAP_ADDR_LEN);
    index += BLE_GAP_ADDR_LEN;

    SER_ASSERT_LENGTH_EQ(index, packet_len);

    *p_event_len = event_len;

    return NRF_SUCCESS;
}
