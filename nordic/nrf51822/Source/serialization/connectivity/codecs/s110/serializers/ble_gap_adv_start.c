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


uint32_t ble_gap_adv_start_req_dec(uint8_t const * const          p_buf,
                                   uint32_t                       packet_len,
                                   ble_gap_adv_params_t * * const pp_adv_params)
{
    uint32_t index = 0, i = 0;

    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(pp_adv_params);
    SER_ASSERT_NOT_NULL(*pp_adv_params);
    SER_ASSERT_NOT_NULL((*pp_adv_params)->p_peer_addr);
    SER_ASSERT_NOT_NULL((*pp_adv_params)->p_whitelist);
    SER_ASSERT_NOT_NULL((*pp_adv_params)->p_whitelist->pp_addrs);
    SER_ASSERT_NOT_NULL((*pp_adv_params)->p_whitelist->pp_irks);

    for (i = 0; i < BLE_GAP_WHITELIST_ADDR_MAX_COUNT; i++)
    {
        SER_ASSERT_NOT_NULL((*pp_adv_params)->p_whitelist->pp_addrs[i]);
    }

    for (i = 0; i < BLE_GAP_WHITELIST_IRK_MAX_COUNT; i++)
    {
        SER_ASSERT_NOT_NULL((*pp_adv_params)->p_whitelist->pp_irks[i]);
    }

    /* Packet with variable length. */
    /* For now check: opcode + indicator showing if ble_gap_adv_params_t struct is present. */
    SER_ASSERT_LENGTH_LEQ(SER_CMD_HEADER_SIZE + 1, packet_len);
    SER_ASSERT(p_buf[index] == SD_BLE_GAP_ADV_START, NRF_ERROR_INVALID_PARAM);
    index++;

    if (p_buf[index++] == SER_FIELD_PRESENT)
    {
        /* Check: Type + Peer Address Present. */
        SER_ASSERT_LENGTH_LEQ(index + 1 + 1, packet_len);
        (*pp_adv_params)->type = p_buf[index++];

        if (p_buf[index++] == SER_FIELD_PRESENT)
        {
            /* Check: Peer Address Type + Peer Address. */
            SER_ASSERT_LENGTH_LEQ(index + 1 + BLE_GAP_ADDR_LEN, packet_len);
            (*pp_adv_params)->p_peer_addr->addr_type = p_buf[index++];
            memcpy(&(*pp_adv_params)->p_peer_addr->addr[0], &p_buf[index], BLE_GAP_ADDR_LEN);
            index += BLE_GAP_ADDR_LEN;
        }
        else
        {
            (*pp_adv_params)->p_peer_addr = NULL;
        }

        /* Check: Filter Policy + GAP Whitelist Present. */
        SER_ASSERT_LENGTH_LEQ(index + 1 + 1, packet_len);
        (*pp_adv_params)->fp = p_buf[index++];

        if (p_buf[index++] == SER_FIELD_PRESENT)
        {
            /* Check: Address Counter. */
            SER_ASSERT_LENGTH_LEQ(index + 1, packet_len);
            (*pp_adv_params)->p_whitelist->addr_count = p_buf[index++];
            SER_ASSERT_LENGTH_LEQ((*pp_adv_params)->p_whitelist->addr_count,
                                  BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

            if ((*pp_adv_params)->p_whitelist->addr_count > 0)
            {
                /* Check: Address list. */
                SER_ASSERT_LENGTH_LEQ(index + ((*pp_adv_params)->p_whitelist->addr_count *
                                               (BLE_GAP_ADDR_LEN + 1)), packet_len);

                for (i = 0; i < (*pp_adv_params)->p_whitelist->addr_count; i++)
                {
                    (*pp_adv_params)->p_whitelist->pp_addrs[i]->addr_type = p_buf[index++];
                    memcpy(&(*pp_adv_params)->p_whitelist->pp_addrs[i]->addr[0], &p_buf[index],
                           BLE_GAP_ADDR_LEN);
                    index += BLE_GAP_ADDR_LEN;
                }
            }
            else
            {
                (*pp_adv_params)->p_whitelist->pp_addrs = NULL;
            }

            /* Check: IRK Counter. */
            SER_ASSERT_LENGTH_LEQ(index + 1, packet_len);
            (*pp_adv_params)->p_whitelist->irk_count = p_buf[index++];
            SER_ASSERT_LENGTH_LEQ((*pp_adv_params)->p_whitelist->irk_count,
                                  BLE_GAP_WHITELIST_IRK_MAX_COUNT);

            if ((*pp_adv_params)->p_whitelist->irk_count > 0)
            {
                /* Check: IRK list. */
                SER_ASSERT_LENGTH_LEQ(index + ((*pp_adv_params)->p_whitelist->irk_count *
                                               BLE_GAP_SEC_KEY_LEN), packet_len);

                for (i = 0; i < (*pp_adv_params)->p_whitelist->irk_count; i++)
                {
                    memcpy(&(*pp_adv_params)->p_whitelist->pp_irks[i]->irk[0], &p_buf[index],
                           BLE_GAP_SEC_KEY_LEN);
                    index += BLE_GAP_SEC_KEY_LEN;
                }
            }
            else
            {
                (*pp_adv_params)->p_whitelist->pp_irks = NULL;
            }
        }
        else
        {
            (*pp_adv_params)->p_whitelist = NULL;
        }

        /* Check: Interval + Timeout. */
        SER_ASSERT_LENGTH_EQ(index + 2 + 2, packet_len);
        uint16_dec(p_buf, packet_len, &index, &(*pp_adv_params)->interval);
        uint16_dec(p_buf, packet_len, &index, &(*pp_adv_params)->timeout);
    }
    else
    {
        *pp_adv_params = NULL;
    }
    SER_ASSERT_LENGTH_EQ(index, packet_len);

    return NRF_SUCCESS;
}

uint32_t ble_gap_adv_start_rsp_enc(uint32_t         return_code,
                                   uint8_t * const  p_buf,
                                   uint32_t * const p_buf_len)
{
    return ser_ble_cmd_rsp_status_code_enc(SD_BLE_GAP_ADV_START, return_code, p_buf, p_buf_len);
}
