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

#include "ble_gatts_conn.h"
#include <string.h>
#include "ble_serialization.h"
#include "app_util.h"


uint32_t ble_gatts_value_set_req_dec(uint8_t const * const p_buf,
                                     uint16_t              packet_len,
                                     uint16_t *            p_handle,
                                     uint16_t *            p_offset,
                                     uint16_t * * const    pp_value_len,
                                     uint8_t * * const     pp_value)
{
    SER_ASSERT_NOT_NULL(p_buf);        //check if *p_buf is allocated
    SER_ASSERT_NOT_NULL(p_handle);     //check if *p_handle exist
    SER_ASSERT_NOT_NULL(p_offset);     //check if *pp_dev_name exist
    SER_ASSERT_NOT_NULL(pp_value_len); //check if *p_value_len exist
    SER_ASSERT_NOT_NULL(pp_value);     //check if *p_value exist

    uint32_t index = SER_CMD_DATA_POS;
    uint32_t status_code;

    SER_ASSERT_LENGTH_LEQ(6, packet_len - index); //make sure that payload length is at least 6 bytes
    uint16_dec(p_buf, packet_len, &index, p_handle);
    uint16_dec(p_buf, packet_len, &index, p_offset);
    //decode optional attribute length optional attribute value
    status_code = cond_len16_cond_data_dec(p_buf, packet_len, &index, pp_value, pp_value_len);
    SER_ASSERT(status_code == NRF_SUCCESS, status_code);

    SER_ASSERT_LENGTH_EQ(index, packet_len);

    return status_code;
}


uint32_t ble_gatts_value_set_rsp_enc(uint32_t         return_code,
                                     uint8_t * const  p_buff,
                                     uint32_t * const p_buff_len,
                                     uint16_t         value_len)
{
    uint32_t index = 0;

    return op_status_cond_uint16_enc(SD_BLE_GATTS_VALUE_SET,
                                     return_code,
                                     value_len,
                                     p_buff,
                                     p_buff_len,
                                     &index);
}
