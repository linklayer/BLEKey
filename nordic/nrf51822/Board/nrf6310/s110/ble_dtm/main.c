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

/** @file
 *
 * @defgroup ble_sdk_app_dtm_main main.c
 * @{
 * @ingroup ble_sdk_app_dtm
 * @brief Serialized DTM Sample Application main file.
 *
 * This file contains the source code for an DTM activation example.
 */

#include <stdbool.h>
#include <stdint.h>
#include "app_button.h"
#include "app_gpiote.h"
#include "app_timer.h"
#include "ble_advdata.h"
#include "ble_dtm_app.h"
#include "boards.h"
#include "nordic_common.h"
#include "softdevice_handler.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT     0       /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DTM_INIT_BUTTON_PIN_NO      BUTTON_0        /**< Button to initializing DTM mode on connectivity chip. */

#define READY_LED_PIN_NO            LED_0           /**< LED indicating that the example is ready. */
#define DTM_READY_LED_PIN_NO        LED_1           /**< LED indicating that the connectivity chip is in DTM mode. */
#define ASSERT_LED_PIN_NO           LED_7           /**< Is on when application has asserted. */

#define DTM_TX_PIN_NO               25              /**< Pin used by DTM to transmitting a data. */
#define DTM_RX_PIN_NO               24              /**< Pin used by DTM to receiving a data. */

#define DEVICE_NAME                 "Nordic_DTM"    /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL            64              /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS  180             /**< The advertising timeout (in units of seconds). */

#define APP_GPIOTE_MAX_USERS        1               /**< Maximum number of users of the GPIOTE handler. */

#define APP_BUTTON_DETECTION_DELAY  100             /**< Delay of detecting button events. */

#define APP_TIMER_PRESCALER         0               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS        4               /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE     4               /**< Size of timer operation queues. */

#define DEAD_BEEF                   0xDEADBEEF      /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_adv_params_t m_adv_params;           /**< Parameters to be passed to the stack when starting advertising. */

static uint32_t        m_error_code;
static uint32_t        m_line_num;
static const uint8_t * mp_file_name;

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    m_error_code = error_code;
    m_line_num   = line_num;
    mp_file_name = p_file_name;

    (void)m_error_code;
    (void)m_line_num;
    (void)mp_file_name;

    //This call can be used for debug purposes during application development.
    //@note CAUTION: Activating this code will write the stack to flash on an error.
    //This function should NOT be used in a final product.
    //It is intended STRICTLY for development/debugging purposes.
    //The flash write will happen EVEN if the radio is active, thus interrupting
    //any communication.
    //Use with care. Un-comment the line below to use.
    //ble_debug_assert_handler(error_code, line_num, p_file_name);

    //On assert, the system can only recover on reset.
    NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(READY_LED_PIN_NO);
    nrf_gpio_cfg_output(DTM_READY_LED_PIN_NO);
    nrf_gpio_cfg_output(ASSERT_LED_PIN_NO);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t                err_code;
    ble_advdata_t           advdata;
    ble_gap_conn_sec_mode_t sec_mode;
    uint8_t                 flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    //Set GAP parameters
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code =
        sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_WATCH);
    APP_ERROR_CHECK(err_code);

    //Build and set advertising data
    memset(&advdata, 0, sizeof (advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags.size         = sizeof (flags);
    advdata.flags.p_data       = &flags;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    //Initialize advertising parameters (used when starting advertising)
    memset(&m_adv_params, 0, sizeof (m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL; //Undirected advertisement
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(READY_LED_PIN_NO);
}


/**@brief Function for initializing the DTM mode.
 */
static void dtm_init(void)
{
    uint32_t                      err_code;
    app_uart_stream_comm_params_t uart_params;

    uart_params.baud_rate = UART_BAUD_RATE_19200;
    uart_params.rx_pin_no = DTM_RX_PIN_NO;
    uart_params.tx_pin_no = DTM_TX_PIN_NO;

    err_code = ble_dtm_init(&uart_params);

    if (err_code == NRF_SUCCESS)
    {
        nrf_gpio_pin_set(DTM_READY_LED_PIN_NO);

        //Close serialization transport layer.
        (void)sd_softdevice_disable();

        //Enter to infinite loop.
        for (;;)
        {
            err_code = sd_app_evt_wait();
            APP_ERROR_CHECK(err_code);
        }
    }
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling button events.
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    switch (pin_no)
    {
        case DTM_INIT_BUTTON_PIN_NO:
            dtm_init();
            break;

        default:
            break;
    }
}


/**@brief Function for initializing buttons.
 */
static void buttons_init(void)
{
    uint32_t err_code;

    static app_button_cfg_t buttons[] =
    {
        {DTM_INIT_BUTTON_PIN_NO, false, BUTTON_PULL, button_event_handler}
    };

    APP_BUTTON_INIT(buttons,
                    sizeof (buttons) / sizeof (buttons[0]),
                    APP_BUTTON_DETECTION_DELAY,
                    false);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code = NRF_SUCCESS;

    //Initialize.
    leds_init();

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

		// Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
	
    buttons_init();
    advertising_init();

    //Start execution.
    advertising_start();

    //Enter main loop.
    for (;; )
    {
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}

/**
 * @}
 */
