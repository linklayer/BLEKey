/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

#include "ble_gap_conn.h"
#include <string.h>
#include "ble_serialization.h"
#include "app_util.h"


uint32_t ble_gap_sec_params_reply_req_dec(uint8_t const * const          p_buf,
                                          uint32_t                       packet_len,
                                          uint16_t *                     p_conn_handle,
                                          uint8_t *                      p_sec_status,
                                          ble_gap_sec_params_t * * const pp_sec_params)
{
    uint32_t index = SER_CMD_HEADER_SIZE;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_conn_handle);
    SER_ASSERT_NOT_NULL(p_sec_status);
    SER_ASSERT_NOT_NULL(pp_sec_params);
    SER_ASSERT_NOT_NULL(*pp_sec_params);

    SER_ASSERT_LENGTH_LEQ(SER_CMD_HEADER_SIZE + 4, packet_len);

    uint16_dec(p_buf, packet_len, &index, p_conn_handle);
    uint8_dec(p_buf, packet_len, &index, p_sec_status);

    if (p_buf[index] == SER_FIELD_PRESENT)
    {
        index++;
        SER_ASSERT_LENGTH_LEQ(index + 5, packet_len);
        uint16_dec(p_buf, packet_len, &index, &((*pp_sec_params)->timeout));
        (*pp_sec_params)->bond    = (p_buf[index] >> 0) & 0x01;
        (*pp_sec_params)->mitm    = (p_buf[index] >> 1) & 0x01;
        (*pp_sec_params)->io_caps = (p_buf[index] >> 2) & 0x07;
        (*pp_sec_params)->oob     = (p_buf[index] >> 5) & 0x01;
        index++;
        uint8_dec(p_buf, packet_len, &index, &(*pp_sec_params)->min_key_size);
        uint8_dec(p_buf, packet_len, &index, &(*pp_sec_params)->max_key_size);
    }
    else if (p_buf[index] == SER_FIELD_NOT_PRESENT)
    {
        index++;
        *pp_sec_params = NULL;
    }
    else
    {
        return NRF_ERROR_INVALID_DATA;
    }

    SER_ASSERT_LENGTH_EQ(index, packet_len);

    return NRF_SUCCESS;
}

//opcode: 0x7F
uint32_t ble_gap_sec_params_reply_rsp_enc(uint32_t         return_code,
                                          uint8_t * const  p_buf,
                                          uint32_t * const p_buf_len)
{
    return ser_ble_cmd_rsp_status_code_enc(SD_BLE_GAP_SEC_PARAMS_REPLY,
                                           return_code,
                                           p_buf,
                                           p_buf_len);
}
