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

#include "ble_gatts_conn.h"
#include <string.h>
#include "ble_serialization.h"
#include "cond_field_serialization.h"
#include "app_util.h"


uint32_t ble_gatts_value_get_req_dec(uint8_t const * const p_buf,
                                     uint16_t              packet_len,
                                     uint16_t * const      handle,
                                     uint16_t * const      offset,
                                     uint16_t * * const    pp_len,
                                     uint8_t * * const     pp_data)
{
    uint32_t index = 0;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_LENGTH_LEQ(SER_CMD_HEADER_SIZE + 6, packet_len);

    SER_ASSERT(p_buf[index] == SD_BLE_GATTS_VALUE_GET, NRF_ERROR_INVALID_PARAM);
    index++;

    uint16_dec(p_buf, packet_len, &index, handle);
    uint16_dec(p_buf, packet_len, &index, offset);

    if (p_buf[index++] == SER_FIELD_PRESENT)
    {
        uint16_dec(p_buf, packet_len, &index, *pp_len);
    }
    else
    {
        *pp_len = NULL;
    }

    if (p_buf[index++] == SER_FIELD_NOT_PRESENT)
    {
        *pp_data = NULL;
    }

    return NRF_SUCCESS;
}


uint32_t ble_gatts_value_get_rsp_enc(uint32_t         return_code,
                                     uint8_t * const  p_buf,
                                     uint32_t * const p_buf_len,
                                     uint8_t * const  p_value,
                                     uint16_t * const p_value_len)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_buf_len);

    SER_ASSERT_LENGTH_LEQ(SER_CMD_RSP_HEADER_SIZE, *p_buf_len);

    uint32_t total_len = *p_buf_len;

    uint32_t err_code = ser_ble_cmd_rsp_status_code_enc(SD_BLE_GATTS_VALUE_GET,
                                                        return_code,
                                                        p_buf,
                                                        p_buf_len);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    uint32_t index = *p_buf_len;

    if (return_code == NRF_SUCCESS) /* Add value and it's length. */
    {
        err_code = cond_field_enc(p_value_len, p_buf, total_len, &index, uint16_t_enc);
        SER_ASSERT(err_code == NRF_SUCCESS, err_code);

        uint16_t value_len = (p_value_len) ? *p_value_len : 0;

        err_code = buf_enc(p_value, value_len, p_buf, total_len, &index);
        SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    }

    *p_buf_len = index;

    return NRF_SUCCESS;
}
