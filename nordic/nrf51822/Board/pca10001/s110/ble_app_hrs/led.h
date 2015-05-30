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

 /** @cond To make doxygen skip this file */
 
/** @file
 *
 * @defgroup ble_sdk_app_hrs_eval_led LED Handling
 * @{
 * @ingroup ble_sdk_app_hrs_eval
 * @brief LED Handling prototypes
 *
 */

#ifndef LED_H__
#define LED_H__


/**@brief   Function for starting flashing the LED.
 * @details This will start the TIMER1 and enable the GPIOTE task that toggles the LED.
 *          The PPI and GPIOTE configurations done by this app will make this action result in the
 *          flashing of the LED.
 * @pre Can only be called after the SoftDevice is enabled - uses nrf_soc API
 */
void led_start(void);

/**@brief  Function for stopping flashing the LED.
 * @details This will stop the TIMER1 and disable the GPIOTE task that toggles the LED.
 *          The PPI and GPIOTE configurations done by this app will
 *          make this action result in the turning off the LED.
 */
void led_stop(void);

#endif // LED_H__

/** @} */
/** @endcond */
