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
#include <stdlib.h>
#include <string.h>
#include "ble_app.h"
#include "app_mailbox.h"
#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "ser_sd_transport.h"
#include "ser_app_hal.h"
#include "ser_config.h"
#include "nrf_soc.h"

#define SD_BLE_EVT_MAILBOX_QUEUE_SIZE 5 /**< Size of mailbox queue. */

/** @brief Structure used to pass packet details through mailbox.
 */
typedef struct
{
    uint32_t evt_data[CEIL_DIV(BLE_STACK_EVT_MSG_BUF_SIZE, sizeof (uint32_t))]; /**< Buffer for decoded event */
} ser_sd_handler_evt_data_t;

/** @brief
 *   Mailbox used for communication between event handler (called from serial stream
 *   interrupt context) and event processing function (called from scheduler or interrupt context).
 */
APP_MAILBOX_DEF(sd_ble_evt_mailbox, SD_BLE_EVT_MAILBOX_QUEUE_SIZE, ser_sd_handler_evt_data_t);

static app_mailbox_id_t m_ble_evt_mailbox_id; /**< mailbox identifier. */

static void connectivity_reset_low(void)
{
    //Signal a reset to the nRF51822 by setting the reset pin on the nRF51822 low.
    ser_app_hal_nrf_reset_pin_clear();
    ser_app_hal_delay(CONN_CHIP_RESET_TIME);

}

static void connectivity_reset_high(void)
{

    //Set the reset level to high again.
    ser_app_hal_nrf_reset_pin_set();

    //Wait for nRF51822 to be ready.
    ser_app_hal_delay(CONN_CHIP_WAKEUP_TIME);
}

static void ser_softdevice_evt_handler(uint8_t * p_data, uint16_t length)
{
    ser_sd_handler_evt_data_t item;
    uint32_t                  err_code;
    uint32_t                  len32 = sizeof (item.evt_data);

    err_code = ble_event_dec(p_data, length, (ble_evt_t *)item.evt_data, &len32);
    APP_ERROR_CHECK(err_code);

    err_code = ser_sd_transport_rx_free(p_data);
    APP_ERROR_CHECK(err_code);

    err_code = app_mailbox_put(m_ble_evt_mailbox_id, &item);
    APP_ERROR_CHECK(err_code);

    ser_app_hal_nrf_evt_pending();
}

/**
 * @brief Function called while waiting for connectivity chip response. It handles incoming events.
 */
static void ser_sd_rsp_wait(void)
{
    do
    {
        ser_app_hal_wait_for_event();

        //intern_softdevice_events_execute();
    }
    while (ser_sd_transport_is_busy());
}

uint32_t sd_evt_get(uint32_t * p_evt_id)
{
    (void)p_evt_id;
    //current serialization doesn't support any events other than ble events
    return NRF_ERROR_NOT_FOUND;
}

uint32_t sd_ble_evt_get(uint8_t * p_data, uint16_t * p_len)
{
    uint32_t err_code;

    err_code = app_mailbox_get(m_ble_evt_mailbox_id, p_data);

    if (err_code == NRF_SUCCESS) //if anything in the mailbox
    {
        if (((ble_evt_t *)p_data)->header.evt_len > *p_len)
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
        else
        {
            *p_len = ((ble_evt_t *)p_data)->header.evt_len;
        }
    }
    else
    {
        err_code = NRF_ERROR_NOT_FOUND;
    }

    return err_code;
}

uint32_t sd_softdevice_enable(nrf_clock_lfclksrc_t           clock_source,
                              softdevice_assertion_handler_t assertion_handler)
{
    uint32_t err_code;

    err_code = ser_app_hal_hw_init();

    if (err_code == NRF_SUCCESS)
    {
        connectivity_reset_low();

        err_code = app_mailbox_create(APP_MAILBOX(sd_ble_evt_mailbox), &m_ble_evt_mailbox_id);

        if (err_code == NRF_SUCCESS)
        {
            err_code = ser_sd_transport_open(ser_softdevice_evt_handler,
                                             ser_sd_rsp_wait,
                                             NULL,
                                             NULL);
            if (err_code == NRF_SUCCESS)
            {
              connectivity_reset_high();
            }
        }
    }

    return err_code;
}

uint32_t sd_nvic_EnableIRQ(IRQn_Type IRQn)
{
    /*if interrupt is a softdevice event handle it as user implemented it.*/
    if (IRQn == SD_EVT_IRQn)
    {
        ser_app_hal_nrf_evt_irq_enable();
    }
    else
    {
        uint8_t  irq   = (uint8_t)IRQn;
        uint32_t irq32 = (uint32_t)irq;
        ser_app_hal_nrf_irq_enable(irq32);
    }
    return NRF_SUCCESS;
}

uint32_t sd_softdevice_disable(void)
{
    return ser_sd_transport_close();
}

uint32_t sd_app_evt_wait(void)
{
    ser_app_hal_wait_for_event();
    return NRF_SUCCESS;
}
