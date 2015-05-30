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


uint32_t ble_gap_evt_connected_dec(uint8_t const * const p_buf,
                                   uint32_t              packet_len,
                                   ble_evt_t * const     p_event,
                                   uint32_t * const      p_event_len)
{
    uint32_t index = 0;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_event_len);

    SER_ASSERT_LENGTH_LEQ(18, packet_len);

    uint32_t event_len = sizeof (ble_gap_evt_connected_t) +
                         sizeof (p_event->evt.gap_evt.conn_handle);

    if (p_event == NULL)
    {
        *p_event_len = event_len;
        return NRF_SUCCESS;
    }

    SER_ASSERT(event_len <= *p_event_len, NRF_ERROR_DATA_SIZE);

    p_event->header.evt_len = event_len;
    uint16_dec(p_buf, packet_len, &index, &p_event->evt.gap_evt.conn_handle);

    ble_gap_evt_connected_t * p_decoded_evt = &(p_event->evt.gap_evt.params.connected);

    p_decoded_evt->peer_addr.addr_type = p_buf[index++];
    memcpy(p_decoded_evt->peer_addr.addr, &p_buf[index], BLE_GAP_ADDR_LEN);
    index += BLE_GAP_ADDR_LEN;

    p_decoded_evt->irk_match     = (p_buf[index] & 0x01);
    p_decoded_evt->irk_match_idx = (p_buf[index] & 0xFE) >> 1;
    index++;

    uint16_dec(p_buf, packet_len, &index, &p_decoded_evt->conn_params.min_conn_interval);
    uint16_dec(p_buf, packet_len, &index, &p_decoded_evt->conn_params.max_conn_interval);
    uint16_dec(p_buf, packet_len, &index, &p_decoded_evt->conn_params.slave_latency);
    uint16_dec(p_buf, packet_len, &index, &p_decoded_evt->conn_params.conn_sup_timeout);

    SER_ASSERT_LENGTH_EQ(index, packet_len);
    *p_event_len = event_len;

    return NRF_SUCCESS;
}
