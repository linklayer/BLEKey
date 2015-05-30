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

#include "ble_gatts_app.h"
#include <string.h>
#include "nrf_error.h"
#include "ble_serialization.h"
#include "app_util.h"


uint32_t ble_gatts_value_set_req_enc(uint16_t              handle,
                                     uint16_t              offset,
                                     uint16_t * const      p_value_len,
                                     uint8_t const * const p_value,
                                     uint8_t * const       p_buf,
                                     uint32_t * const      p_buf_len)
{
    uint32_t index = 0;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_buf_len);

    SER_ASSERT_LENGTH_LEQ(1 + 2 + 2, *p_buf_len);

    p_buf[index++] = SD_BLE_GATTS_VALUE_SET;
    index         += uint16_encode(handle, &p_buf[index]);
    index         += uint16_encode(offset, &p_buf[index]);

    if (p_value_len == NULL)
    {
        SER_ASSERT_LENGTH_LEQ(index + 2, *p_buf_len);
        p_buf[index++] = SER_FIELD_NOT_PRESENT;
        p_buf[index++] = SER_FIELD_NOT_PRESENT;
        *p_buf_len     = index;
        return NRF_SUCCESS;
    }

    SER_ASSERT_LENGTH_LEQ(index + 1 + 2 + 1, *p_buf_len);

    p_buf[index++] = SER_FIELD_PRESENT;
    index         += uint16_encode(*p_value_len, &p_buf[index]);

    if (p_value != NULL)
    {
        SER_ERROR_CHECK(*p_value_len <= BLE_GATTS_VAR_ATTR_LEN_MAX, NRF_ERROR_INVALID_PARAM);
        p_buf[index++] = SER_FIELD_PRESENT;
        SER_ASSERT_LENGTH_LEQ(index + *p_value_len, *p_buf_len);
        memcpy(&(p_buf[index]), p_value, *p_value_len);
        index += *p_value_len;
    }
    else
    {
        p_buf[index++] = SER_FIELD_NOT_PRESENT;
    }

    *p_buf_len = index;

    return NRF_SUCCESS;
}


uint32_t ble_gatts_value_set_rsp_dec(uint8_t const * const p_buf,
                                     uint32_t              packet_len,
                                     uint16_t * const      p_value_len,
                                     uint32_t * const      p_result_code)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_result_code);

    uint32_t index         = 0;
    uint32_t decode_result = ser_ble_cmd_rsp_result_code_dec(p_buf,
                                                             &index,
                                                             packet_len,
                                                             SD_BLE_GATTS_VALUE_SET,
                                                             p_result_code);

    if (decode_result != NRF_SUCCESS)
    {
        return decode_result;
    }

    if (*p_result_code != NRF_SUCCESS)
    {
        SER_ASSERT_LENGTH_EQ(index, packet_len);
        return NRF_SUCCESS;
    }

    SER_ASSERT_LENGTH_LEQ(index + sizeof (uint16_t), packet_len);
    uint16_dec(p_buf, packet_len, &index, p_value_len);

    SER_ASSERT_LENGTH_EQ(index, packet_len);

    return NRF_SUCCESS;
}
