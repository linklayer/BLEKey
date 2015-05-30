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

#include "ble_app_gzll_ui.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "app_util.h"
#include "app_button.h"
#include "app_timer.h"
#include "ble_app_gzll_common.h"


#define BUTTON_DETECTION_DELAY  APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */


void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
    nrf_gpio_cfg_output(ASSERT_LED_PIN_NO);
    nrf_gpio_cfg_output(GZLL_TX_SUCCESS_LED_PIN_NO);
    nrf_gpio_cfg_output(GZLL_TX_FAIL_LED_PIN_NO);
}


/**@brief Function for handling button events.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    if (button_action == APP_BUTTON_PUSH)
    {
        switch (pin_no)
        {
            case BLE_BUTTON_PIN_NO:
                running_mode = BLE;
                break;
                
            case GZLL_BUTTON_PIN_NO:
                running_mode = GAZELL;
                break;
                
            default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
    }
}


/**@brief Function for initializing the button handler module.
 */
void buttons_init(void)
{
    uint32_t err_code;
    
    // Configure buttons.
    static app_button_cfg_t buttons[] =
    {
        {BLE_BUTTON_PIN_NO,  false, BUTTON_PULL, button_event_handler},
        {GZLL_BUTTON_PIN_NO, false, BUTTON_PULL, button_event_handler}
    };
    
    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);

    // Start handling button presses immediately.
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}
