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

#include "ble_gap_struct_serialization.h"
#include "ble_serialization.h"
#include "cond_field_serialization.h"
#include "app_util.h"
#include "string.h"

uint32_t ble_gap_irk_enc(void const * const p_data,
                         uint8_t * const    p_buf,
                         uint32_t           buf_len,
                         uint32_t * const   p_index)
{
    ble_gap_irk_t * p_gap_irk = (ble_gap_irk_t *)p_data;

    SER_ASSERT_LENGTH_LEQ(BLE_GAP_SEC_KEY_LEN, buf_len - *p_index);

    memcpy(&p_buf[*p_index], p_gap_irk->irk, BLE_GAP_SEC_KEY_LEN);

    *p_index += BLE_GAP_SEC_KEY_LEN;

    return NRF_SUCCESS;
}

uint32_t ble_gap_irk_dec(uint8_t const * const p_buf,
                         uint32_t              buf_len,
                         uint32_t * const      p_index,
                         void * const          p_data)
{
    ble_gap_irk_t * p_gap_irk = (ble_gap_irk_t *)p_data;

    SER_ASSERT_LENGTH_LEQ(BLE_GAP_SEC_KEY_LEN, buf_len - *p_index);

    memcpy(p_gap_irk->irk, &p_buf[*p_index], BLE_GAP_SEC_KEY_LEN);

    *p_index += BLE_GAP_SEC_KEY_LEN;

    return NRF_SUCCESS;
}

uint32_t ble_gap_addr_enc(void const * const p_data,
                          uint8_t * const    p_buf,
                          uint32_t           buf_len,
                          uint32_t * const   p_index)
{
    ble_gap_addr_t * p_addr = (ble_gap_addr_t *)p_data;

    SER_ASSERT_LENGTH_LEQ(1 + BLE_GAP_ADDR_LEN, buf_len - *p_index);

    p_buf[*p_index] = p_addr->addr_type;
    (*p_index)++;
    memcpy(&p_buf[*p_index], p_addr->addr, BLE_GAP_ADDR_LEN);
    *p_index += BLE_GAP_ADDR_LEN;

    return NRF_SUCCESS;
}

uint32_t ble_gap_addr_dec(uint8_t const * const p_buf,
                          uint32_t              buf_len,
                          uint32_t * const      p_index,
                          void * const          p_addr)
{
    ble_gap_addr_t * p_address = (ble_gap_addr_t *) p_addr;

    SER_ASSERT_LENGTH_LEQ(sizeof (ble_gap_addr_t), buf_len - *p_index);
    memcpy(p_address, &p_buf[*p_index], sizeof (ble_gap_addr_t));
    *p_index += sizeof (ble_gap_addr_t);

    return NRF_SUCCESS;
}

uint32_t ble_gap_sec_levels_enc(void const * const p_data,
                                uint8_t * const    p_buf,
                                uint32_t           buf_len,
                                uint32_t * const   p_index)
{
    ble_gap_sec_levels_t * p_sec_levels = (ble_gap_sec_levels_t *)p_data;

    SER_ASSERT_LENGTH_LEQ(1, buf_len - *p_index);

    p_buf[*p_index] = (p_sec_levels->lv1 << 0) | (p_sec_levels->lv2 << 1) | (p_sec_levels->lv3 << 2);
    (*p_index)++;

    return NRF_SUCCESS;
}

uint32_t ble_gap_sec_keys_enc(void const * const p_data,
                              uint8_t * const    p_buf,
                              uint32_t           buf_len,
                              uint32_t * const   p_index)
{
    ble_gap_sec_keys_t * p_sec_keys = (ble_gap_sec_keys_t *)p_data;

    SER_ASSERT_LENGTH_LEQ(1, buf_len - *p_index);

    p_buf[*p_index] = (p_sec_keys->ltk << 0) | (p_sec_keys->ediv_rand << 1) | (p_sec_keys->irk << 2)
                      | (p_sec_keys->address << 3) | (p_sec_keys->csrk << 4);
    (*p_index)++;

    return NRF_SUCCESS;
}

uint32_t ble_gap_enc_info_enc(void const * const p_data,
                              uint8_t * const    p_buf,
                              uint32_t           buf_len,
                              uint32_t * const   p_index)
{
    uint32_t             err_code;
    ble_gap_enc_info_t * p_enc_info = (ble_gap_enc_info_t *)p_data;

    SER_ASSERT_LENGTH_LEQ(2 + BLE_GAP_SEC_KEY_LEN + 1, buf_len - *p_index);

    err_code = uint16_t_enc(&p_enc_info->div, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    memcpy(&p_buf[*p_index], p_enc_info->ltk, BLE_GAP_SEC_KEY_LEN);
    *p_index += BLE_GAP_SEC_KEY_LEN;

    p_buf[*p_index]  = p_enc_info->auth & 0x01;
    p_buf[*p_index] |= (p_enc_info->ltk_len & 0x7F) << 1;
    (*p_index)++;

    return err_code;
}

uint32_t ble_gap_enc_info_dec(uint8_t const * const p_buf,
                              uint32_t              buf_len,
                              uint32_t * const      p_index,
                              void * const          p_enc_info)
{
    ble_gap_enc_info_t * p_enc_info2 = (ble_gap_enc_info_t *)p_enc_info;

    SER_ASSERT_LENGTH_LEQ(2, buf_len - *p_index);
    uint16_dec(p_buf, buf_len, p_index, &p_enc_info2->div);

    SER_ASSERT_LENGTH_LEQ(BLE_GAP_SEC_KEY_LEN, buf_len - *p_index);
    memcpy(p_enc_info2->ltk, &p_buf[*p_index], BLE_GAP_SEC_KEY_LEN);
    *p_index += BLE_GAP_SEC_KEY_LEN;

    SER_ASSERT_LENGTH_LEQ(1, buf_len - *p_index);
    p_enc_info2->auth    = p_buf[*p_index] & 0x01;
    p_enc_info2->ltk_len = (p_buf[*p_index] >> 1) & 0x7F;
    *p_index            += 1;

    return NRF_SUCCESS;
}

uint32_t ble_gap_sign_info_dec(uint8_t const * const p_buf,
                               uint32_t              buf_len,
                               uint32_t * const      p_index,
                               void * const          p_sign_info)
{
    SER_ASSERT_LENGTH_LEQ(sizeof (ble_gap_sign_info_t), buf_len - *p_index);
    memcpy(p_sign_info, &p_buf[*p_index], sizeof (ble_gap_sign_info_t));
    *p_index += sizeof (ble_gap_sign_info_t);

    return NRF_SUCCESS;
}

uint32_t ble_gap_evt_auth_status_t_enc(void const * const p_data,
                                       uint8_t * const    p_buf,
                                       uint32_t           buf_len,
                                       uint32_t * const   p_index)
{
    uint32_t err_code = NRF_SUCCESS;
    uint32_t tmp;

    ble_gap_evt_auth_status_t * auth_status = (ble_gap_evt_auth_status_t *)p_data;

    SER_ASSERT_LENGTH_LEQ(2, buf_len - *p_index);

    err_code = uint8_t_enc(&auth_status->auth_status, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint8_t_enc(&auth_status->error_src, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    tmp      = *p_index;
    err_code = ble_gap_sec_levels_enc(&auth_status->sm1_levels, p_buf, buf_len, &tmp);

    SER_ASSERT(err_code == NRF_SUCCESS, err_code);
    tmp      = p_buf[*p_index]; //read encoded sm1_levels fields
    err_code = ble_gap_sec_levels_enc(&auth_status->sm2_levels,
                                      p_buf,
                                      buf_len,
                                      p_index);
    p_buf[(*p_index) - 1] |= tmp << 3; //add previously encoded sm1_levels fields

    err_code = ble_gap_sec_keys_enc(&auth_status->periph_kex, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = ble_gap_sec_keys_enc(&auth_status->central_kex, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = ble_gap_enc_info_enc(&auth_status->periph_keys.enc_info, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = ble_gap_irk_enc(&auth_status->central_keys.irk, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = ble_gap_addr_enc(&auth_status->central_keys.id_info, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_conn_sec_mode_enc(void const * const p_void_sec_mode,
                                   uint8_t * const    p_buf,
                                   uint32_t           buf_len,
                                   uint32_t * const   p_index)
{
    ble_gap_conn_sec_mode_t * p_sec_mode = (ble_gap_conn_sec_mode_t *)p_void_sec_mode;
    uint32_t                  err_code   = NRF_SUCCESS;
    uint8_t                   temp8      = p_sec_mode->sm | (p_sec_mode->lv << 4);

    err_code = uint8_t_enc(&temp8, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_conn_sec_mode_dec(uint8_t const * const p_buf,
                                   uint32_t              buf_len,
                                   uint32_t * const      p_index,
                                   void * const          p_void_sec_mode)
{
    ble_gap_conn_sec_mode_t * p_sec_mode = (ble_gap_conn_sec_mode_t *)p_void_sec_mode;
    uint32_t                  err_code   = NRF_SUCCESS;
    uint8_t                   temp8;

    SER_ASSERT_LENGTH_LEQ(1, buf_len - *p_index);
    uint8_dec(p_buf, buf_len, p_index, &temp8);

    p_sec_mode->sm = temp8;
    p_sec_mode->lv = temp8 >> 4;

    return err_code;
}

uint32_t ble_gap_evt_conn_sec_update_t_enc(void const * const p_void_conn_sec_update,
                                           uint8_t * const    p_buf,
                                           uint32_t           buf_len,
                                           uint32_t * const   p_index)
{
    return ble_gap_conn_sec_t_enc(p_void_conn_sec_update, p_buf, buf_len, p_index);
}

uint32_t ble_gap_evt_conn_sec_update_t_dec(uint8_t const * const p_buf,
                                           uint32_t              buf_len,
                                           uint32_t * const      p_index,
                                           void * const          p_void_conn_sec_update)
{
    return ble_gap_conn_sec_t_dec(p_buf, buf_len, p_index, p_void_conn_sec_update);
}

uint32_t ble_gap_conn_sec_t_enc(void const * const p_void_sec,
                                uint8_t * const    p_buf,
                                uint32_t           buf_len,
                                uint32_t * const   p_index)
{
    ble_gap_conn_sec_t * p_conn_sec = (ble_gap_conn_sec_t *)p_void_sec;
    uint32_t             err_code   = NRF_SUCCESS;

    err_code = ble_gap_conn_sec_mode_enc(&p_conn_sec->sec_mode, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint8_t_enc(&p_conn_sec->encr_key_size, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_conn_sec_t_dec(uint8_t const * const p_buf,
                                uint32_t              buf_len,
                                uint32_t * const      p_index,
                                void * const          p_void_sec)
{
    ble_gap_conn_sec_t * p_conn_sec = (ble_gap_conn_sec_t *)p_void_sec;
    uint32_t             err_code   = NRF_SUCCESS;

    err_code = ble_gap_conn_sec_mode_dec(p_buf, buf_len, p_index, &p_conn_sec->sec_mode);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    SER_ASSERT_LENGTH_LEQ(1, buf_len - *p_index);
    uint8_dec(p_buf, buf_len, p_index, &p_conn_sec->encr_key_size);

    return err_code;
}

uint32_t ble_gap_evt_sec_info_request_t_enc(void const * const p_void_sec_info_request,
                                            uint8_t * const    p_buf,
                                            uint32_t           buf_len,
                                            uint32_t * const   p_index)
{
    ble_gap_evt_sec_info_request_t * p_conn_sec =
        (ble_gap_evt_sec_info_request_t *)p_void_sec_info_request;

    uint32_t err_code = NRF_SUCCESS;

    err_code = ble_gap_addr_enc(&p_conn_sec->peer_addr, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint16_t_enc(&p_conn_sec->div, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    uint8_t temp8 = p_conn_sec->enc_info |
                    (p_conn_sec->id_info << 1) |
                    (p_conn_sec->sign_info << 2);

    err_code = uint8_t_enc(&temp8, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_evt_sec_info_request_t_dec(uint8_t const * const p_buf,
                                            uint32_t              buf_len,
                                            uint32_t * const      p_index,
                                            void * const          p_void_sec_info_request)
{
    //ble_gap_evt_sec_info_request_t * p_conn_sec = (ble_gap_evt_sec_info_request_t *)p_void_sec_info_request;
    uint32_t err_code = NRF_SUCCESS;

    return err_code;
}

uint32_t ble_gap_evt_connected_t_enc(void const * const p_void_struct,
                                     uint8_t * const    p_buf,
                                     uint32_t           buf_len,
                                     uint32_t * const   p_index)
{
    ble_gap_evt_connected_t * p_evt_conn = (ble_gap_evt_connected_t *)p_void_struct;
    uint32_t                  err_code   = NRF_SUCCESS;

    err_code = ble_gap_addr_enc((void *)&p_evt_conn->peer_addr, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    p_buf[*p_index]  = p_evt_conn->irk_match & 0x01;
    p_buf[*p_index] |= (p_evt_conn->irk_match_idx & 0x7F) << 1;
    (*p_index)++;

    err_code = ble_gap_conn_params_t_enc((void *)&p_evt_conn->conn_params, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_sec_params_t_enc(void const * const p_void_struct,
                                  uint8_t * const    p_buf,
                                  uint32_t           buf_len,
                                  uint32_t * const   p_index)
{
    ble_gap_sec_params_t * p_sec_params = (ble_gap_sec_params_t *)p_void_struct;
    uint32_t               err_code     = NRF_SUCCESS;
    uint8_t                temp8;

    err_code = uint16_t_enc((void *)&p_sec_params->timeout, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    temp8 = p_sec_params->bond |
            (p_sec_params->mitm << 1) |
            (p_sec_params->io_caps << 2) |
            (p_sec_params->oob << 5);

    err_code = uint8_t_enc((void *)&temp8, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint8_t_enc((void *)&p_sec_params->min_key_size, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint8_t_enc((void *)&p_sec_params->max_key_size, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_sec_params_t_dec(uint8_t const * const p_buf,
                                  uint32_t              buf_len,
                                  uint32_t * const      p_index,
                                  void * const          p_void_struct)
{
    ble_gap_sec_params_t * p_sec_params = (ble_gap_sec_params_t *)p_void_struct;
    uint32_t               err_code     = NRF_SUCCESS;
    uint8_t                temp8;

    SER_ASSERT_LENGTH_LEQ(2, buf_len - *p_index);
    uint16_dec(p_buf, buf_len, p_index, (void *)&p_sec_params->timeout);

    SER_ASSERT_LENGTH_LEQ(1, buf_len - *p_index);
    uint8_dec(p_buf, buf_len, p_index, (void *)&temp8);

    p_sec_params->bond    = temp8;
    p_sec_params->mitm    = temp8 >> 1;
    p_sec_params->io_caps = temp8 >> 2;
    p_sec_params->oob     = temp8 >> 5;

    SER_ASSERT_LENGTH_LEQ(1, buf_len - *p_index);
    uint8_dec(p_buf, buf_len, p_index, (void *)&p_sec_params->min_key_size);

    SER_ASSERT_LENGTH_LEQ(1, buf_len - *p_index);
    uint8_dec(p_buf, buf_len, p_index, (void *)&p_sec_params->max_key_size);

    return err_code;
}

uint32_t ble_gap_evt_sec_params_request_t_enc(void const * const p_void_struct,
                                              uint8_t * const    p_buf,
                                              uint32_t           buf_len,
                                              uint32_t * const   p_index)
{
    return ble_gap_sec_params_t_enc(p_void_struct, p_buf, buf_len, p_index);
}

uint32_t ble_gap_evt_sec_params_request_t_dec(uint8_t const * const p_buf,
                                              uint32_t              buf_len,
                                              uint32_t * const      p_index,
                                              void * const          p_void_struct)
{
    return ble_gap_sec_params_t_dec(p_buf, buf_len, p_index, p_void_struct);
}

uint32_t ble_gap_evt_conn_param_update_t_enc(void const * const p_void_evt_conn_param_update,
                                             uint8_t * const    p_buf,
                                             uint32_t           buf_len,
                                             uint32_t * const   p_index)
{
    return ble_gap_conn_params_t_enc(p_void_evt_conn_param_update, p_buf, buf_len, p_index);
}

uint32_t ble_gap_evt_conn_param_update_t_dec(uint8_t const * const p_buf,
                                             uint32_t              buf_len,
                                             uint32_t * const      p_index,
                                             void * const          p_void_evt_conn_param_update)
{
    return ble_gap_conn_params_t_dec(p_buf, buf_len, p_index, p_void_evt_conn_param_update);
}

uint32_t ble_gap_conn_params_t_enc(void const * const p_void_conn_params,
                                   uint8_t * const    p_buf,
                                   uint32_t           buf_len,
                                   uint32_t * const   p_index)
{
    ble_gap_conn_params_t * p_conn_params = (ble_gap_conn_params_t *)p_void_conn_params;
    uint32_t                err_code      = NRF_SUCCESS;

    err_code = uint16_t_enc(&p_conn_params->min_conn_interval, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint16_t_enc(&p_conn_params->max_conn_interval, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint16_t_enc(&p_conn_params->slave_latency, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint16_t_enc(&p_conn_params->conn_sup_timeout, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_conn_params_t_dec(uint8_t const * const p_buf,
                                   uint32_t              buf_len,
                                   uint32_t * const      p_index,
                                   void * const          p_void_conn_params)
{
    ble_gap_conn_params_t * p_conn_params = (ble_gap_conn_params_t *)p_void_conn_params;

    SER_ASSERT_LENGTH_LEQ(*p_index + 2, buf_len);
    uint16_dec(p_buf, buf_len, p_index, &p_conn_params->min_conn_interval);

    SER_ASSERT_LENGTH_LEQ(*p_index + 2, buf_len);
    uint16_dec(p_buf, buf_len, p_index, &p_conn_params->max_conn_interval);

    SER_ASSERT_LENGTH_LEQ(*p_index + 2, buf_len);
    uint16_dec(p_buf, buf_len, p_index, &p_conn_params->slave_latency);

    SER_ASSERT_LENGTH_LEQ(*p_index + 2, buf_len);
    uint16_dec(p_buf, buf_len, p_index, &p_conn_params->conn_sup_timeout);

    return NRF_SUCCESS;
}

uint32_t ble_gap_evt_disconnected_t_enc(void const * const p_void_disconnected,
                                        uint8_t * const    p_buf,
                                        uint32_t           buf_len,
                                        uint32_t * const   p_index)
{
    ble_gap_evt_disconnected_t * p_disconnected = (ble_gap_evt_disconnected_t *)p_void_disconnected;
    uint32_t                     err_code       = NRF_SUCCESS;

    err_code = uint8_t_enc(&p_disconnected->reason, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_evt_disconnected_t_dec(uint8_t const * const p_buf,
                                        uint32_t              buf_len,
                                        uint32_t * const      p_index,
                                        void * const          p_void_disconnected)
{
    ble_gap_evt_disconnected_t * p_disconnected = (ble_gap_evt_disconnected_t *)p_void_disconnected;
    uint32_t                     err_code       = NRF_SUCCESS;

    SER_ASSERT_LENGTH_LEQ(1, buf_len - *p_index);
    uint8_dec(p_buf, buf_len, p_index, &p_disconnected->reason);

    return err_code;
}

uint32_t ble_gap_opt_local_conn_latency_t_enc(void const * const p_void_local_conn_latency,
                                              uint8_t * const    p_buf,
                                              uint32_t           buf_len,
                                              uint32_t * const   p_index)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_index);

    ble_gap_opt_local_conn_latency_t * p_latency =
        (ble_gap_opt_local_conn_latency_t *)p_void_local_conn_latency;
    uint32_t err_code = NRF_SUCCESS;

    err_code = uint16_t_enc(&(p_latency->conn_handle), p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint16_t_enc(&(p_latency->requested_latency), p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = cond_field_enc(p_latency->p_actual_latency, p_buf, buf_len, p_index, uint16_t_enc);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_opt_local_conn_latency_t_dec(uint8_t const * const p_buf,
                                              uint32_t              buf_len,
                                              uint32_t * const      p_index,
                                              void * const          p_void_local_conn_latency)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_index);

    ble_gap_opt_local_conn_latency_t * p_latency =
        (ble_gap_opt_local_conn_latency_t *)p_void_local_conn_latency;
    uint32_t err_code = NRF_SUCCESS;

    err_code = uint16_t_dec(p_buf, buf_len, p_index, &(p_latency->conn_handle));
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint16_t_dec(p_buf, buf_len, p_index, &(p_latency->requested_latency));
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = cond_field_dec(p_buf, buf_len, p_index, (void **) &(p_latency->p_actual_latency),
                              uint16_t_dec);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_opt_passkey_t_enc(void const * const p_void_passkey,
                                   uint8_t * const    p_buf,
                                   uint32_t           buf_len,
                                   uint32_t * const   p_index)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_index);

    ble_gap_opt_passkey_t * p_opt_passkey  = (ble_gap_opt_passkey_t *)p_void_passkey;
    uint32_t   err_code                    = NRF_SUCCESS;
    uint16_t passkey_len                   = BLE_GAP_PASSKEY_LEN;

    err_code = buf_enc(p_opt_passkey->p_passkey, passkey_len, p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_opt_passkey_t_dec(uint8_t const * const p_buf,
                                   uint32_t              buf_len,
                                   uint32_t * const      p_index,
                                   void * const          p_void_passkey)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_index);

    ble_gap_opt_passkey_t * p_opt_passkey  = (ble_gap_opt_passkey_t *)p_void_passkey;
    uint32_t   err_code                    = NRF_SUCCESS;
    uint16_t passkey_len                   = BLE_GAP_PASSKEY_LEN;

    err_code = buf_dec(p_buf, buf_len, p_index, &p_opt_passkey->p_passkey, passkey_len,
                       passkey_len);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_opt_privacy_t_enc(void const * const p_void_privacy,
                                   uint8_t * const    p_buf,
                                   uint32_t           buf_len,
                                   uint32_t * const   p_index)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_index);

    ble_gap_opt_privacy_t * p_privacy = (ble_gap_opt_privacy_t *)p_void_privacy;
    uint32_t                 err_code = NRF_SUCCESS;

    err_code = cond_field_enc(p_privacy->p_irk, p_buf, buf_len, p_index, ble_gap_irk_enc);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint16_t_enc(&(p_privacy->interval_s), p_buf, buf_len, p_index);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}

uint32_t ble_gap_opt_privacy_t_dec(uint8_t const * const p_buf,
                                   uint32_t              buf_len,
                                   uint32_t * const      p_index,
                                   void * const          p_void_privacy)
{
    SER_ASSERT_NOT_NULL(p_buf);
    SER_ASSERT_NOT_NULL(p_index);

    ble_gap_opt_privacy_t * p_privacy = (ble_gap_opt_privacy_t *)p_void_privacy;
    uint32_t                 err_code = NRF_SUCCESS;

    err_code = cond_field_dec(p_buf, buf_len, p_index, (void **) &(p_privacy->p_irk), ble_gap_irk_dec);
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    err_code = uint16_t_dec(p_buf, buf_len, p_index, &(p_privacy->interval_s));
    SER_ASSERT(err_code == NRF_SUCCESS, err_code);

    return err_code;
}
