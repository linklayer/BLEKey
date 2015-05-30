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
 * @defgroup ble_sdk_app_gzll_ui Multiprotocol Application User Interface
 * @{
 * @ingroup ble_sdk_app_gzll
 * @brief User Interface (buttons and LED) handling for the multiprotocol application
 */

#ifndef BLE_APP_GZLL_UI_H__
#define BLE_APP_GZLL_UI_H__

#include <stdbool.h>
#include "boards.h"

#define BLE_BUTTON_PIN_NO              BUTTON_0        /**<  Button used for switching to Bluetooth Heart Rate example. */

#define GZLL_BUTTON_PIN_NO             BUTTON_1        /**<  Button used for switching to Gazell example. */

#define ADVERTISING_LED_PIN_NO         LED_0           /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN_NO           LED_1           /**< Is on when device has connected. */
#define ASSERT_LED_PIN_NO              LED_7           /**< Is on when application has asserted. */

#define GZLL_TX_SUCCESS_LED_PIN_NO     LED_2           /**<  LED used to show successull Transmits.*/
#define GZLL_TX_FAIL_LED_PIN_NO        LED_3           /**<  LED used to show failed Transmits.*/

/**@brief Function for initializing GPIOTE module for detecting buttons.
 */
void buttons_init(void);

/**@brief Function for the LEDs initialization.
 *
 * @details Function for initializing all LEDs used by the application.
 */
void leds_init(void);

#endif // BLE_APP_GZLL_UI_H__
/** @} */

