/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup ble_sdk_app_proximity_main main.c
 * @{
 * @ingroup ble_sdk_app_proximity
 * @brief Proximity Sample Application main file.
 *
 * This file contains is the source code for a sample proximity application using the
 * Immediate Alert, Link Loss and Tx Power services.
 *
 * This application would accept pairing requests from any peer device.
 *
 * It demonstrates the use of fast and slow advertising intervals.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_tps.h"
#include "ble_ias.h"
#include "ble_lls.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "ble_sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "ble_ias_c.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "app_trace.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define SIGNAL_ALERT_BUTTON              BUTTON_0                                     /**< Button used for send or cancel High Alert to the peer. */
#define BOND_DELETE_ALL_BUTTON_ID        BUTTON_1                                     /**< Button used for deleting all bonded centrals during startup. */

#define ADVERTISING_LED_PIN_NO           LED_0                                        /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN_NO             LED_1                                        /**< Is on when device has connected. */
#define ALERT_LEVEL_MILD_LED_PIN_NO      LED_2                                        /**< Is on when we are in Mild Alert state. */
#define ALERT_LEVEL_HIGH_LED_PIN_NO      LED_3                                        /**< Is on when we are in High Alert state. */
#define ADV_INTERVAL_SLOW_LED_PIN_NO     LED_4                                        /**< Is on when we are doing slow advertising. */
#define PEER_SRV_DISC_LED_PIN_NO         LED_5                                        /**< Is on when the Immediate Alert Service has been discovered at the peer. */
#define ADV_WHITELIST_LED_PIN_NO         LED_6                                        /**< Is on when we are doing advertising with whitelist. */
#define ASSERT_LED_PIN_NO                LED_7                                        /**< Is on when application has asserted. */

#define DEVICE_NAME                      "Nordic_Prox"                                /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL_FAST            0x0028                                       /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_INTERVAL_SLOW            0x0C80                                       /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_SLOW_ADV_TIMEOUT             180                                          /**< The duration of the slow advertising period (in seconds). */
#define APP_FAST_ADV_TIMEOUT             40                                           /**< The duration of the fast advertising period (in seconds). */
#define APP_FAST_ADV_WHITELIST_TIMEOUT   20                                           /**< The duration of the fast advertising with whitelist period (in seconds). */

#define DEVICE_NAME                      "Nordic_Prox"                                /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL_FAST            0x0028                                       /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_INTERVAL_SLOW            0x0C80                                       /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_SLOW_ADV_TIMEOUT             180                                          /**< The duration of the slow advertising period (in seconds). */
#define APP_FAST_ADV_TIMEOUT             40                                           /**< The duration of the fast advertising period (in seconds). */
#define APP_FAST_ADV_WHITELIST_TIMEOUT   20                                           /**< The duration of the fast advertising with whitelist period (in seconds). */

#define APP_TIMER_PRESCALER              0                                            /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS             4                                            /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                            /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)   /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                81                                           /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                100                                          /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT          1                                            /**< Increment between each simulated battery level measurement. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(500, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(1000, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                    0                                            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)              /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS             1                                            /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY           APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT                30                                           /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                   1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                           /**< Maximum encryption key size. */

#define INITIAL_LLS_ALERT_LEVEL          BLE_CHAR_ALERT_LEVEL_NO_ALERT                /**< Initial value for the Alert Level characteristic in the Link Loss service. */
#define TX_POWER_LEVEL                   (-8)                                         /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */

#define DEAD_BEEF                        0xDEADBEEF                                   /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


typedef enum
{
    BLE_NO_ADV,                                                                       /**< No advertising running. */
    BLE_DIRECTED_ADV,                                                                 /**< Direct advertising to the latest central. */
    BLE_FAST_ADV_WHITELIST,                                                           /**< Advertising with whitelist. */
    BLE_FAST_ADV,                                                                     /**< Fast advertising running. */
    BLE_SLOW_ADV,                                                                     /**< Slow advertising running. */
    BLE_SLEEP,                                                                        /**< Go to system-off. */
} ble_advertising_mode_t;

static ble_tps_t                         m_tps;                                       /**< Structure used to identify the TX Power service. */
static ble_ias_t                         m_ias;                                       /**< Structure used to identify the Immediate Alert service. */
static ble_lls_t                         m_lls;                                       /**< Structure used to identify the Link Loss service. */
static bool                              m_is_link_loss_alerting;                     /**< Variable to indicate if a link loss has been detected. */

static ble_bas_t                         m_bas;                                       /**< Structure used to identify the battery service. */
static ble_ias_c_t                       m_ias_c;                                     /**< Structure used to identify the client to the Immediate Alert Service at peer. */

static ble_sensorsim_cfg_t               m_battery_sim_cfg;                           /**< Battery Level sensor simulator configuration. */
static ble_sensorsim_state_t             m_battery_sim_state;                         /**< Battery Level sensor simulator state. */

static volatile bool                     m_is_high_alert_signalled;                   /**< Variable to indicate whether or not high alert is signalled to the peer. */

static app_timer_id_t                    m_battery_timer_id;                          /**< Battery timer. */
static dm_application_instance_t         m_app_handle;                                /**< Application identifier allocated by device manager */

static uint8_t                           m_advertising_mode;                          /**< Variable to keep track of when we are advertising. */

static bool                              m_memory_access_in_progress = false;         /**< Flag to keep track of ongoing operations on persisten memory. */

static void on_ias_evt(ble_ias_t * p_ias, ble_ias_evt_t * p_evt);
static void on_lls_evt(ble_lls_t * p_lls, ble_lls_evt_t * p_evt);
static void on_ias_c_evt(ble_ias_c_t * p_lls, ble_ias_c_evt_t * p_evt);
static void advertising_init(uint8_t adv_flags);


#if 0
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

    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Uncomment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}
#else

#include "app_util_platform.h"
//uint32_t m_error_code;
//uint32_t m_line_num;
//const uint8_t *m_p_file_name;

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // disable INTs
    CRITICAL_REGION_ENTER();
    /* Light a LED on error or warning. */
    nrf_gpio_cfg_output(SER_CONN_ASSERT_LED_PIN);
    nrf_gpio_pin_set(SER_CONN_ASSERT_LED_PIN);

//    m_p_file_name = p_file_name;
//    m_error_code = error_code;
//    m_line_num = line_num;

    /* To be able to see function parameters in a debugger. */
    uint32_t temp = 1;
    while(temp);
    CRITICAL_REGION_EXIT();    
}

#endif

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


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    ble_gap_whitelist_t  whitelist;
    uint32_t             count;

    // Clear all advertising LEDs
    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
    nrf_gpio_pin_clear(ADV_WHITELIST_LED_PIN_NO);
    nrf_gpio_pin_clear(ADV_INTERVAL_SLOW_LED_PIN_NO);

    // Verify if there is any flash access pending, if yes delay starting advertising until 
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);
    
    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }

    // Initialize advertising parameters with defaults values
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.p_whitelist = NULL;

    // Configure advertisement according to current advertising state.
    switch (m_advertising_mode)
    {
        case BLE_NO_ADV:
            m_advertising_mode = BLE_FAST_ADV_WHITELIST;
            // fall through.

        case BLE_FAST_ADV_WHITELIST:
        {
            ble_gap_addr_t       * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t        * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];
            
            whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
            whitelist.pp_addrs   = p_whitelist_addr;
            whitelist.pp_irks    = p_whitelist_irk;
            
            err_code = dm_whitelist_create(&m_app_handle, &whitelist);
            APP_ERROR_CHECK(err_code);

            if ((whitelist.addr_count != 0) || (whitelist.irk_count != 0))
            {
                adv_params.fp          = BLE_GAP_ADV_FP_FILTER_CONNREQ;
                adv_params.p_whitelist = &whitelist;

                advertising_init(BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
                m_advertising_mode = BLE_FAST_ADV;
                nrf_gpio_pin_set(ADV_WHITELIST_LED_PIN_NO);
            }
            else
            {
                m_advertising_mode = BLE_SLOW_ADV;
            }

            adv_params.interval = APP_ADV_INTERVAL_FAST;
            adv_params.timeout  = APP_FAST_ADV_TIMEOUT;
            break;
        }

        case BLE_FAST_ADV:
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);

            adv_params.interval = APP_ADV_INTERVAL_FAST;
            adv_params.timeout  = APP_FAST_ADV_TIMEOUT;
            m_advertising_mode  = BLE_SLOW_ADV;
            break;

        case BLE_SLOW_ADV:
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);

            adv_params.interval = APP_ADV_INTERVAL_SLOW;
            adv_params.timeout  = APP_SLOW_ADV_TIMEOUT;
            m_advertising_mode  = BLE_SLEEP;

            nrf_gpio_pin_set(ADV_INTERVAL_SLOW_LED_PIN_NO);
            break;

        default:
            // No implementation needed.
            break;
    }

    // Start advertising.
    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)ble_sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
    nrf_gpio_cfg_output(ASSERT_LED_PIN_NO);

    nrf_gpio_cfg_output(ALERT_LEVEL_MILD_LED_PIN_NO);
    nrf_gpio_cfg_output(ALERT_LEVEL_HIGH_LED_PIN_NO);
    nrf_gpio_cfg_output(ADV_INTERVAL_SLOW_LED_PIN_NO);
    nrf_gpio_cfg_output(ADV_WHITELIST_LED_PIN_NO);
    nrf_gpio_cfg_output(PEER_SRV_DISC_LED_PIN_NO);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Start battery timer.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_KEYRING);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 *
 * @param[in]  adv_flags  Indicates which type of advertisement to use, see @ref BLE_GAP_DISC_MODES.
 *
 */
static void advertising_init(uint8_t adv_flags)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    int8_t        tx_power_level = TX_POWER_LEVEL;

    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_TX_POWER_SERVICE,        BLE_UUID_TYPE_BLE},
        {BLE_UUID_IMMEDIATE_ALERT_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_LINK_LOSS_SERVICE,       BLE_UUID_TYPE_BLE}
    };

    m_advertising_mode = BLE_NO_ADV;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(adv_flags);
    advdata.flags.p_data            = &adv_flags;
    advdata.p_tx_power_level        = &tx_power_level;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Tx Power Service.
 */
static void tps_init(void)
{
    uint32_t       err_code;
    ble_tps_init_t tps_init_obj;

    memset(&tps_init_obj, 0, sizeof(tps_init_obj));
    tps_init_obj.initial_tx_power_level = TX_POWER_LEVEL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&tps_init_obj.tps_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&tps_init_obj.tps_attr_md.write_perm);

    err_code = ble_tps_init(&m_tps, &tps_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Immediate Alert Service.
 */
static void ias_init(void)
{
    uint32_t       err_code;
    ble_ias_init_t ias_init_obj;

    memset(&ias_init_obj, 0, sizeof(ias_init_obj));
    ias_init_obj.evt_handler = on_ias_evt;

    err_code = ble_ias_init(&m_ias, &ias_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Link Loss Service.
 */
static void lls_init(void)
{
    uint32_t       err_code;
    ble_lls_init_t lls_init_obj;

    // Initialize Link Loss Service
    memset(&lls_init_obj, 0, sizeof(lls_init_obj));

    lls_init_obj.evt_handler         = on_lls_evt;
    lls_init_obj.error_handler       = service_error_handler;
    lls_init_obj.initial_alert_level = INITIAL_LLS_ALERT_LEVEL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&lls_init_obj.lls_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&lls_init_obj.lls_attr_md.write_perm);

    err_code = ble_lls_init(&m_lls, &lls_init_obj);
    APP_ERROR_CHECK(err_code);

    m_is_link_loss_alerting = false;
}


/**@brief Function for initializing the Battery Service.
 */
static void bas_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_report_read_perm);

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the immediate alert service client.
 * @details This will initialize the client side functionality of the Find Me profile.
 */
static void ias_client_init(void)
{
    uint32_t            err_code;
    ble_ias_c_init_t    ias_c_init_obj;

    memset(&ias_c_init_obj, 0, sizeof(ias_c_init_obj));

    m_is_high_alert_signalled = false;

    ias_c_init_obj.evt_handler   = on_ias_c_evt;
    ias_c_init_obj.error_handler = service_error_handler;

    err_code = ble_ias_c_init(&m_ias_c, &ias_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the services that will be used by the application.
 */
static void services_init(void)
{
    tps_init();
    ias_init();
    lls_init();
    bas_init();
    ias_client_init();
}


/**@brief Function for initializing the battery sensor simulator.
 */
static void sensor_sim_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    ble_sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for signaling alert event from Immediate Alert or Link Loss services.
 *
 * @param[in]   alert_level  Requested alert level.
 */
static void alert_signal(uint8_t alert_level)
{
    switch (alert_level)
    {
        case BLE_CHAR_ALERT_LEVEL_NO_ALERT:
            m_is_link_loss_alerting = false;
            nrf_gpio_pin_clear(ALERT_LEVEL_MILD_LED_PIN_NO);
            nrf_gpio_pin_clear(ALERT_LEVEL_HIGH_LED_PIN_NO);
            break;

        case BLE_CHAR_ALERT_LEVEL_MILD_ALERT:
            nrf_gpio_pin_set(ALERT_LEVEL_MILD_LED_PIN_NO);
            nrf_gpio_pin_clear(ALERT_LEVEL_HIGH_LED_PIN_NO);
            break;

        case BLE_CHAR_ALERT_LEVEL_HIGH_ALERT:
            nrf_gpio_pin_clear(ALERT_LEVEL_MILD_LED_PIN_NO);
            nrf_gpio_pin_set(ALERT_LEVEL_HIGH_LED_PIN_NO);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling Immediate Alert events.
 *
 * @details This function will be called for all Immediate Alert events which are passed to the
 *          application.
 *
 * @param[in]   p_ias  Immediate Alert structure.
 * @param[in]   p_evt  Event received from the Immediate Alert service.
 */
static void on_ias_evt(ble_ias_t * p_ias, ble_ias_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_IAS_EVT_ALERT_LEVEL_UPDATED:
            alert_signal(p_evt->params.alert_level);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling Link Loss events.
 *
 * @details This function will be called for all Link Loss events which are passed to the
 *          application.
 *
 * @param[in]   p_lls  Link Loss structure.
 * @param[in]   p_evt  Event received from the Link Loss service.
 */
static void on_lls_evt(ble_lls_t * p_lls, ble_lls_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_LLS_EVT_LINK_LOSS_ALERT:
            m_is_link_loss_alerting = true;
            alert_signal(p_evt->params.alert_level);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling IAS Client events.
 *
 * @details This function will be called for all IAS Client events which are passed to the
 *          application.
 *
 * @param[in]   p_ias_c  IAS Client structure.
 * @param[in]   p_evt    Event received.
 */
static void on_ias_c_evt(ble_ias_c_t * p_ias_c, ble_ias_c_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_IAS_C_EVT_SRV_DISCOVERED:
            nrf_gpio_pin_set(PEER_SRV_DISC_LED_PIN_NO);
            break;

        case BLE_IAS_C_EVT_SRV_NOT_FOUND:
            // IAS is not found on peer. Do Nothing.
            break;

        case BLE_IAS_C_EVT_DISCONN_COMPLETE:
            nrf_gpio_pin_clear(PEER_SRV_DISC_LED_PIN_NO);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t        err_code = NRF_SUCCESS;
    static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            nrf_gpio_pin_clear(ADV_INTERVAL_SLOW_LED_PIN_NO);
            nrf_gpio_pin_clear(ADV_WHITELIST_LED_PIN_NO);
            
            // Start handling button presses.
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);

            m_advertising_mode = BLE_NO_ADV;
            m_conn_handle      = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);

            if (!m_is_link_loss_alerting)
            {
                nrf_gpio_pin_clear(ALERT_LEVEL_MILD_LED_PIN_NO);
                nrf_gpio_pin_clear(ALERT_LEVEL_HIGH_LED_PIN_NO);
            }

            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            APP_ERROR_CHECK(err_code);

            advertising_start();
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                if (m_advertising_mode == BLE_SLEEP)
                {
                    m_advertising_mode = BLE_NO_ADV;

                    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
                    nrf_gpio_pin_clear(ADV_WHITELIST_LED_PIN_NO);
                    nrf_gpio_pin_clear(ADV_INTERVAL_SLOW_LED_PIN_NO);

                    // Configure buttons with sense level low as wakeup source.
                    nrf_gpio_cfg_sense_input(SIGNAL_ALERT_BUTTON,
                                             BUTTON_PULL,
                                             NRF_GPIO_PIN_SENSE_LOW);
                    
                    nrf_gpio_cfg_sense_input(BOND_DELETE_ALL_BUTTON_ID,
                                             BUTTON_PULL,
                                             NRF_GPIO_PIN_SENSE_LOW);
                        
                    // Go to system-off mode.
                    // (this function will not return; wakeup will cause a reset).
                    err_code = sd_power_system_off();
                    APP_ERROR_CHECK(err_code);
                }
                else
                {
                    advertising_start();
                }
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                advertising_start();
            }
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_ias_on_ble_evt(&m_ias, p_ble_evt);
    ble_lls_on_ble_evt(&m_lls, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_ias_c_on_ble_evt(&m_ias_c, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           api_result_t           event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    uint32_t                err_code;
    dm_init_param_t         init_data;
    dm_application_param_t  register_param;
    
    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    // Clear all bonded centrals if the Bonds Delete button is pushed.
    init_data.clear_persistent_data = (nrf_gpio_pin_read(BOND_DELETE_ALL_BUTTON_ID) == 0);

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    
    register_param.sec_param.timeout      = SEC_PARAM_TIMEOUT;
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling button events.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;

    if (button_action == APP_BUTTON_PUSH)
    {
        switch (pin_no)
        {
            case SIGNAL_ALERT_BUTTON:
                if (!m_is_high_alert_signalled)
                {
                    err_code = ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_HIGH_ALERT);
                }
                else
                {
                    err_code = ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_NO_ALERT);
                }

                if (err_code == NRF_SUCCESS)
                {
                    m_is_high_alert_signalled = !m_is_high_alert_signalled;
                }
                else if ((err_code != BLE_ERROR_NO_TX_BUFFERS) &&
                         (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) &&
                         (err_code != NRF_ERROR_NOT_FOUND)
                )
                {
                    APP_ERROR_HANDLER(err_code);
                }
                else
                {
                    // Other errors are expected errors and should not be processed.
                }
                break;

            default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
    }
}


/**@brief Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for initializing the button module.
 */
static void buttons_init(void)
{
    static app_button_cfg_t buttons[] =
    {
        {SIGNAL_ALERT_BUTTON,       false, BUTTON_PULL, button_event_handler},
        {BOND_DELETE_ALL_BUTTON_ID, false, BUTTON_PULL, NULL}
    };

    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);
}


/**@brief Function for power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    app_trace_init();
    leds_init();
    timers_init();
    gpiote_init();
    buttons_init();		
    ble_stack_init();
    device_manager_init();
    gap_params_init();
    advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);
    services_init();
    sensor_sim_init();
    conn_params_init();

    // Start execution.
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}

/**
 * @}
 */
