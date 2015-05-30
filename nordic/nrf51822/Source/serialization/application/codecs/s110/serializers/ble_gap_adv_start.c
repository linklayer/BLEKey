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

#include "ble_gap_app.h"
#include <string.h>
#include "ble_serialization.h"
#include "ble_gap.h"
#include "app_util.h"

#define WHITELIST_ENCODE_LEN(p_whitelist) (1 + ((p_whitelist)->addr_count * (1 + BLE_GAP_ADDR_LEN)) \
                                           + 1 + ((p_whitelist)->irk_count * BLE_GAP_SEC_KEY_LEN))


static uint32_t whitelist_encode(uint8_t *                         p_packet,
                                 ble_gap_whitelist_t const * const p_whitelist)
{
    uint32_t index = 0, i = 0;

    p_packet[index++] = p_whitelist->addr_count;

    for (i = 0; i < p_whitelist->addr_count; i++)
    {
        p_packet[index++] = p_whitelist->pp_addrs[i]->addr_type;
        memcpy(&p_packet[index], &p_whitelist->pp_addrs[i]->addr[0], BLE_GAP_ADDR_LEN);
        index += BLE_GAP_ADDR_LEN;
    }

    p_packet[index++] = p_whitelist->irk_count;

    for (i = 0; i < p_whitelist->irk_count; i++)
    {
        memcpy(&p_packet[index], &p_whitelist->pp_irks[i]->irk[0], BLE_GAP_SEC_KEY_LEN);
        index += BLE_GAP_SEC_KEY_LEN;
    }

    return index;
}


uint32_t ble_gap_adv_start_req_enc(ble_gap_adv_params_t const * const p_adv_params,
                                   uint8_t * const                    p_buf,
                                   uint32_t * const                   p_buf_len)
{
    uint32_t index = 0;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_buf_len);

    SER_ASSERT_LENGTH_LEQ(index + 2, *p_buf_len);
    p_buf[index++] = SD_BLE_GAP_ADV_START;
    p_buf[index++] = (p_adv_params == NULL) ? SER_FIELD_NOT_PRESENT : SER_FIELD_PRESENT;

    if (p_adv_params != NULL)
    {
        SER_ASSERT_LENGTH_LEQ(index + 2, *p_buf_len);
        p_buf[index++] = p_adv_params->type;
        p_buf[index++] = (p_adv_params->p_peer_addr != NULL) ?
                         SER_FIELD_PRESENT : SER_FIELD_NOT_PRESENT;

        if (p_adv_params->p_peer_addr != NULL)
        {
            SER_ASSERT_LENGTH_LEQ(index + 1 + BLE_GAP_ADDR_LEN, *p_buf_len);
            p_buf[index++] = p_adv_params->p_peer_addr->addr_type;
            memcpy(&p_buf[index], &p_adv_params->p_peer_addr->addr[0], BLE_GAP_ADDR_LEN);
            index += BLE_GAP_ADDR_LEN;
        }

        SER_ASSERT_LENGTH_LEQ(index + 2, *p_buf_len);
        p_buf[index++] = p_adv_params->fp;
        p_buf[index++] = (p_adv_params->p_whitelist != NULL) ?
                         SER_FIELD_PRESENT : SER_FIELD_NOT_PRESENT;

        if (p_adv_params->p_whitelist != NULL)
        {
            ble_gap_whitelist_t * p_whitelist = p_adv_params->p_whitelist;

            SER_ERROR_CHECK(p_whitelist->addr_count <= BLE_GAP_WHITELIST_ADDR_MAX_COUNT,
                            NRF_ERROR_INVALID_PARAM);
            SER_ERROR_CHECK(p_whitelist->irk_count <= BLE_GAP_WHITELIST_IRK_MAX_COUNT,
                            NRF_ERROR_INVALID_PARAM);
            SER_ASSERT_LENGTH_LEQ(index + WHITELIST_ENCODE_LEN(p_whitelist), *p_buf_len);

            index += whitelist_encode(&p_buf[index], p_whitelist);
        }

        SER_ASSERT_LENGTH_LEQ(index + 4, *p_buf_len);
        index += uint16_encode(p_adv_params->interval, &p_buf[index]);
        index += uint16_encode(p_adv_params->timeout, &p_buf[index]);
    }

    *p_buf_len = index;

    return NRF_SUCCESS;
}


uint32_t ble_gap_adv_start_rsp_dec(uint8_t const * const p_buf,
                                   uint32_t              packet_len,
                                   uint32_t * const      p_result_code)
{
    return ser_ble_cmd_rsp_dec(p_buf, packet_len, SD_BLE_GAP_ADV_START, p_result_code);
}
