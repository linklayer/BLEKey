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

#include "ble_gap_app.h"
#include "ble_serialization.h"
#include "app_util.h"


uint32_t ble_gap_authenticate_req_enc(uint16_t                           conn_handle,
                                      ble_gap_sec_params_t const * const p_sec_params,
                                      uint8_t * const                    p_buf,
                                      uint32_t * const                   p_buf_len)
{
    uint32_t index = 0;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_buf_len);

    SER_ASSERT_LENGTH_LEQ(1 + 2 + 1, *p_buf_len);

    p_buf[index++] = SD_BLE_GAP_AUTHENTICATE;
    index         += uint16_encode(conn_handle, &p_buf[index]);

    p_buf[index++] = (p_sec_params != NULL) ? SER_FIELD_PRESENT : SER_FIELD_NOT_PRESENT;

    if (p_sec_params != NULL)
    {
        SER_ASSERT_LENGTH_LEQ(index + 2 + 1 + 1 + 1, *p_buf_len);

        index         += uint16_encode(p_sec_params->timeout, &p_buf[index]);
        p_buf[index++] = ((p_sec_params->oob << 5) |
                          (p_sec_params->io_caps << 2) |
                          (p_sec_params->mitm << 1) |
                          (p_sec_params->bond << 0));
        p_buf[index++] = p_sec_params->min_key_size;
        p_buf[index++] = p_sec_params->max_key_size;
    }

    *p_buf_len = index;

    return NRF_SUCCESS;
}


uint32_t ble_gap_authenticate_rsp_dec(uint8_t const * const p_buf,
                                      uint32_t              packet_len,
                                      uint32_t * const      p_result_code)
{
    return ser_ble_cmd_rsp_dec(p_buf, packet_len, SD_BLE_GAP_AUTHENTICATE, p_result_code);
}
